#include "yolo_obstacle_detection_ros2/gpu_preprocess.hpp"

#if HAS_TENSORRT
#include <cmath>

namespace {
__device__ inline float clampf_device(float v, float lo, float hi) {
    return fminf(hi, fmaxf(lo, v));
}

__global__ void bgr8_letterbox_to_nchw_kernel(
    const unsigned char* src,
    int src_w,
    int src_h,
    int src_stride,
    float* dst,
    int dst_w,
    int dst_h,
    float scale,
    int resized_w,
    int resized_h,
    int pad_left,
    int pad_top) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= dst_w || y >= dst_h) return;

    const int index = y * dst_w + x;
    float r = 114.0f / 255.0f;
    float g = r;
    float b = r;

    if (x >= pad_left && x < pad_left + resized_w &&
        y >= pad_top && y < pad_top + resized_h) {
        // Match OpenCV resize semantics closely using pixel-center mapping and
        // bilinear sampling.  This avoids the CPU cv::resize + color convert +
        // HWC->CHW loop on Jetson while preserving detector geometry.
        const float sx = ((static_cast<float>(x - pad_left) + 0.5f) / scale) - 0.5f;
        const float sy = ((static_cast<float>(y - pad_top) + 0.5f) / scale) - 0.5f;
        const float sx_clamped = clampf_device(sx, 0.0f, static_cast<float>(src_w - 1));
        const float sy_clamped = clampf_device(sy, 0.0f, static_cast<float>(src_h - 1));
        const int x0 = static_cast<int>(floorf(sx_clamped));
        const int y0 = static_cast<int>(floorf(sy_clamped));
        const int x1 = min(x0 + 1, src_w - 1);
        const int y1 = min(y0 + 1, src_h - 1);
        const float wx = sx_clamped - x0;
        const float wy = sy_clamped - y0;

        const unsigned char* p00 = src + y0 * src_stride + x0 * 3;
        const unsigned char* p01 = src + y0 * src_stride + x1 * 3;
        const unsigned char* p10 = src + y1 * src_stride + x0 * 3;
        const unsigned char* p11 = src + y1 * src_stride + x1 * 3;

        const float wb00 = (1.0f - wx) * (1.0f - wy);
        const float wb01 = wx * (1.0f - wy);
        const float wb10 = (1.0f - wx) * wy;
        const float wb11 = wx * wy;

        b = (wb00 * p00[0] + wb01 * p01[0] + wb10 * p10[0] + wb11 * p11[0]) / 255.0f;
        g = (wb00 * p00[1] + wb01 * p01[1] + wb10 * p10[1] + wb11 * p11[1]) / 255.0f;
        r = (wb00 * p00[2] + wb01 * p01[2] + wb10 * p10[2] + wb11 * p11[2]) / 255.0f;
    }

    const int plane = dst_w * dst_h;
    dst[index] = r;
    dst[plane + index] = g;
    dst[2 * plane + index] = b;
}
}  // namespace

cudaError_t launch_bgr8_letterbox_to_nchw(
    const unsigned char* src_bgr,
    int src_width,
    int src_height,
    int src_stride_bytes,
    float* dst_nchw,
    int dst_width,
    int dst_height,
    cudaStream_t stream) {
    if (!src_bgr || !dst_nchw || src_width <= 0 || src_height <= 0 ||
        dst_width <= 0 || dst_height <= 0 || src_stride_bytes < src_width * 3) {
        return cudaErrorInvalidValue;
    }
    const float scale = fminf(static_cast<float>(dst_width) / src_width,
                              static_cast<float>(dst_height) / src_height);
    const int resized_w = static_cast<int>(roundf(src_width * scale));
    const int resized_h = static_cast<int>(roundf(src_height * scale));
    const int pad_left = (dst_width - resized_w) / 2;
    const int pad_top = (dst_height - resized_h) / 2;
    dim3 block(16, 16);
    dim3 grid((dst_width + block.x - 1) / block.x,
              (dst_height + block.y - 1) / block.y);
    bgr8_letterbox_to_nchw_kernel<<<grid, block, 0, stream>>>(
        src_bgr, src_width, src_height, src_stride_bytes,
        dst_nchw, dst_width, dst_height, scale, resized_w, resized_h,
        pad_left, pad_top);
    return cudaGetLastError();
}
#endif
