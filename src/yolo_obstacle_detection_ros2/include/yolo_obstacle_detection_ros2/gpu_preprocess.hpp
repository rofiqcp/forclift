#pragma once

#if HAS_TENSORRT
#include <cuda_runtime.h>

// GPU letterbox + BGR8 -> normalized RGB NCHW float32.
// The caller owns src/dst device buffers and the CUDA stream.
cudaError_t launch_bgr8_letterbox_to_nchw(
    const unsigned char* src_bgr,
    int src_width,
    int src_height,
    int src_stride_bytes,
    float* dst_nchw,
    int dst_width,
    int dst_height,
    cudaStream_t stream);
#endif
