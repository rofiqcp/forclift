#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include "yolo_obstacle_detection_ros2/msg/obstacle.hpp"
#include "yolo_obstacle_detection_ros2/msg/obstacle_array.hpp"
#include <chrono>
#include <algorithm>
#include <atomic>
#include <limits>
#include <set>
#include <string>
#include <vector>
#include <mutex>
#include <fstream>
#include <memory>
#include <thread>
#include <utility>
#include <condition_variable>
#include <cmath>
#include <cstring>
#include <sstream>
#include <numeric>

// ─── AGV Forklift model contract ───────────────────────────────────────────
// Class count is application-specific, but TensorRT input/output dimensions are
// discovered from the serialized engine at runtime in PART-3.  This removes the
// previous hard dependency on one fixed input tensor shape and one fixed YOLO grid size.
static constexpr int DEFAULT_INPUT_SIZE = 640;
#ifdef WAREHOUSE_COCO_PROFILE
// Secondary low-rate semantic profile. The local YOLOv8n TensorRT engine is
// COCO-80, but only class 0 (person) is emitted. LiDAR remains the primary
// geometry obstacle detector, so this adds dynamic semantics with bounded load.
static constexpr int NUM_CLASSES = 80;
#else
static constexpr int NUM_CLASSES = 5;
#endif
static constexpr int EXPECTED_OUTPUT_CHANNELS = 4 + NUM_CLASSES;

static const char* class_name_for_id(int class_id) {
#ifdef WAREHOUSE_COCO_PROFILE
    return class_id == 0 ? "person" : "coco_other";
#else
    static constexpr const char* names[5] = {
        "floor marking", "block", "front", "hole pallet", "pallet"
    };
    return (class_id >= 0 && class_id < 5) ? names[class_id] : "unknown";
#endif
}

struct YOLODet {
    float bbox[4];  // cx, cy, width, height in model input space
    float cls_conf;
    int class_id;
};

#if HAS_TENSORRT
#include <NvInfer.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include "yolo_obstacle_detection_ros2/gpu_preprocess.hpp"

// ─── TensorRT Logger ─────────────────────────────────────────────────────────
class TensorRTLogger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING)
            fprintf(stderr, "[TensorRT] %s\n", msg);
    }
};
static TensorRTLogger gLogger;
#endif // HAS_TENSORRT

// ── Letterbox: resize keep aspect ratio, pad with (114,114,114) ──────────
// Returns { scale, pad_left, pad_top } so we can un-letterbox bbox later.
// Uses UNIFORM scale (the minimum) for both axes — critical for correct
// bbox coordinate mapping. A "full-height" detection in model space must
// map back to the full image height in original space.
struct LetterBox {
    float scale;   // uniform: min(target/orig_w, target/orig_h)
    int pad_left;
    int pad_top;
    int resized_w;
    int resized_h;
};
LetterBox compute_letterbox_params(int orig_w, int orig_h, int target_size) {
    LetterBox lb;
    lb.scale = std::min(static_cast<float>(target_size) / orig_w,
                        static_cast<float>(target_size) / orig_h);
    lb.resized_w = static_cast<int>(std::round(orig_w * lb.scale));
    lb.resized_h = static_cast<int>(std::round(orig_h * lb.scale));
    lb.pad_left  = (target_size - lb.resized_w) / 2;
    lb.pad_top   = (target_size - lb.resized_h) / 2;
    return lb;
}
void apply_letterbox(const cv::Mat& src, cv::Mat& dst, int target_size) {
    LetterBox lb = compute_letterbox_params(src.cols, src.rows, target_size);
    cv::Mat resized;
    cv::resize(src, resized, cv::Size(lb.resized_w, lb.resized_h));
    dst = cv::Mat(target_size, target_size, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(dst(cv::Rect(lb.pad_left, lb.pad_top, lb.resized_w, lb.resized_h)));
}
// Un-letterbox bbox from model-space back to original image-space.
// Model outputs are in the letterboxed canvas (target_size × target_size).
// We use UNIFORM scale for both X and Y — same as the forward letterbox.
cv::Rect unletterbox_bbox(float cx_model, float cy_model,
                          float bw_model, float bh_model,
                          int orig_w, int orig_h,
                          const LetterBox& lb) {
    float x1 = (cx_model - bw_model * 0.5f - lb.pad_left) / lb.scale;
    float y1 = (cy_model - bh_model * 0.5f - lb.pad_top)  / lb.scale;
    float x2 = (cx_model + bw_model * 0.5f - lb.pad_left) / lb.scale;
    float y2 = (cy_model + bh_model * 0.5f - lb.pad_top)  / lb.scale;
    int ix1 = std::max(0,    static_cast<int>(std::round(x1)));
    int iy1 = std::max(0,    static_cast<int>(std::round(y1)));
    int ix2 = std::min(orig_w, static_cast<int>(std::round(x2)));
    int iy2 = std::min(orig_h, static_cast<int>(std::round(y2)));
    return cv::Rect(ix1, iy1, ix2 - ix1, iy2 - iy1);
}

static void nms_yolo(std::vector<YOLODet>& dets, float iou_thresh,
                     std::vector<YOLODet>& out) {
    std::sort(dets.begin(), dets.end(),
              [](const YOLODet& a, const YOLODet& b){ return a.cls_conf > b.cls_conf; });
    std::vector<bool> suppressed(dets.size(), false);
    for (size_t i = 0; i < dets.size(); ++i) {
        if (suppressed[i]) continue;
        out.push_back(dets[i]);
        for (size_t j = i + 1; j < dets.size(); ++j) {
            if (suppressed[j]) continue;
            // Default Ultralytics NMS is class-aware.  Suppressing a pallet
            // because it overlaps a different class (e.g. hole pallet) loses
            // useful application semantics.
            if (dets[i].class_id != dets[j].class_id) continue;
            float ax = dets[i].bbox[0], ay = dets[i].bbox[1];
            float aw = dets[i].bbox[2], ah = dets[i].bbox[3];
            float bx = dets[j].bbox[0], by = dets[j].bbox[1];
            float bw = dets[j].bbox[2], bh = dets[j].bbox[3];
            float x1 = std::max(ax - aw*0.5f, bx - bw*0.5f);
            float y1 = std::max(ay - ah*0.5f, by - bh*0.5f);
            float x2 = std::min(ax + aw*0.5f, bx + bw*0.5f);
            float y2 = std::min(ay + ah*0.5f, by + bh*0.5f);
            float inter = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
            float area_a = aw * ah, area_b = bw * bh;
            const float denom = area_a + area_b - inter;
            float iou = denom > 1e-9f ? inter / denom : 0.0f;
            if (iou > iou_thresh) suppressed[j] = true;
        }
    }
}

// ─── Main Node ──────────────────────────────────────────────────────────────
class ObstacleDetectorNode final : public rclcpp::Node {
public:
    explicit ObstacleDetectorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("obstacle_detector_node", options) {
        // ── Declare parameters ──────────────────────────────────────────────
        declare_parameter("model_path",    std::string("/home/otomasi2/ros/models/yolov8n_agv_forklift.onnx"));
        declare_parameter("engine_path",   std::string(""));
        declare_parameter("confidence_threshold", 0.5);
        declare_parameter("iou_threshold",        0.45);
        declare_parameter("input_topic",          std::string("/camera/color/image_raw"));
        declare_parameter("output_topic",         std::string("/obstacle_detection/obstacles"));
        declare_parameter("visualization_topic",  std::string("/obstacle_detection/visualization"));
#ifdef WAREHOUSE_COCO_PROFILE
        declare_parameter("status_topic",         std::string("/warehouse_obstacle/status"));
        declare_parameter("performance_topic",    std::string("/warehouse_obstacle/performance"));
#else
        declare_parameter("status_topic",         std::string("/obstacle_detection/status"));
        declare_parameter("performance_topic",    std::string("/obstacle_detection/performance"));
#endif
        declare_parameter("danger_zone_distance", 2.0);
        declare_parameter("warning_distance",     1.5);
        declare_parameter("use_tensorrt",          false);
        declare_parameter("use_cuda",              true);
        declare_parameter("require_cuda",           true);
        declare_parameter("use_gpu_preprocess",     true);
        declare_parameter("allow_passthrough_without_model", true);
        declare_parameter("inference_size",       DEFAULT_INPUT_SIZE);
        declare_parameter("max_inference_fps",    30.0);
        declare_parameter("max_visualization_fps", 15.0);

        model_path_       = get_parameter("model_path").as_string();
        engine_path_      = get_parameter("engine_path").as_string();
        conf_thresh_      = get_parameter("confidence_threshold").as_double();
        iou_thresh_       = get_parameter("iou_threshold").as_double();
        danger_zone_      = get_parameter("danger_zone_distance").as_double();
        warning_distance_ = get_parameter("warning_distance").as_double();
        use_tensorrt_     = get_parameter("use_tensorrt").as_bool();
        use_cuda_         = get_parameter("use_cuda").as_bool();
        require_cuda_      = get_parameter("require_cuda").as_bool();
        use_gpu_preprocess_ = get_parameter("use_gpu_preprocess").as_bool();
        allow_passthrough_without_model_ =
            get_parameter("allow_passthrough_without_model").as_bool();
        inference_size_   = std::clamp(static_cast<int>(get_parameter("inference_size").as_int()), 320, 1280);
        max_inference_fps_ = std::max(1.0, get_parameter("max_inference_fps").as_double());
        max_visualization_fps_ = std::max(1.0, get_parameter("max_visualization_fps").as_double());

        // ── Publishers ──────────────────────────────────────────────────────
        obstacle_pub_ = create_publisher<yolo_obstacle_detection_ros2::msg::ObstacleArray>(
            get_parameter("output_topic").as_string(), 1);
        annotated_pub_ = create_publisher<sensor_msgs::msg::Image>(
            get_parameter("visualization_topic").as_string(),
            rclcpp::SensorDataQoS().keep_last(1));
        auto status_qos = rclcpp::QoS(1).transient_local().reliable();
        rclcpp::PublisherOptions status_options;
        // Preserve late-joiner status semantics while allowing the camera image
        // subscription to use intra-process communication when composable.
        status_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
        status_pub_ = create_publisher<std_msgs::msg::String>(
            get_parameter("status_topic").as_string(), status_qos, status_options);
        performance_pub_ = create_publisher<std_msgs::msg::String>(
            get_parameter("performance_topic").as_string(), rclcpp::QoS(10).best_effort());

        // ── Robust model init ───────────────────────────────────────────────
        // The camera/visualization pipeline must stay alive even when weights
        // are missing or an engine is stale.  TensorRT failure falls back to
        // ONNX; ONNX failure falls back to annotated camera passthrough.
#if HAS_TENSORRT
        if (use_tensorrt_) {
            if (init_tensorrt()) {
                model_ready_ = true;
                RCLCPP_INFO(get_logger(), "[YOLO] Backend: TensorRT FP16");
            } else {
                release_tensorrt_resources();
                RCLCPP_INFO(get_logger(),
                    "[YOLO-BACKEND] TensorRT engine unavailable/invalid; using OpenCV CUDA DNN");
                use_tensorrt_ = false;
            }
        }
#else
        if (use_tensorrt_) {
            RCLCPP_INFO(get_logger(),
                "[YOLO-BACKEND] TensorRT not compiled; using OpenCV CUDA DNN");
            use_tensorrt_ = false;
        }
#endif

        if (!use_tensorrt_ && !model_ready_) {
            std::ifstream model_file(model_path_, std::ios::binary);
            if (!model_file.good()) {
                RCLCPP_WARN(get_logger(),
                    "[YOLO-MODEL] model not found: %s", model_path_.c_str());
            } else {
                model_file.close();
                try {
                    net_ = cv::dnn::readNetFromONNX(model_path_);
                    if (net_.empty()) {
                        RCLCPP_WARN(get_logger(),
                            "[YOLO-MODEL] OpenCV returned an empty network for: %s",
                            model_path_.c_str());
                    } else {
                        if (use_cuda_) {
                            const int cuda_devices = cv::cuda::getCudaEnabledDeviceCount();
                            if (cuda_devices <= 0) {
                                if (require_cuda_) {
                                    throw std::runtime_error(
                                        "[YOLO-CUDA] no CUDA-enabled OpenCV device; CPU fallback is disabled");
                                }
                                use_cuda_ = false;
                            }
                        }
                        if (use_cuda_) {
                            cv::cuda::setDevice(0);
                            net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
                            net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);
                            // Force one real forward pass now. Merely selecting a CUDA
                            // backend does not prove this OpenCV build has CUDA DNN.
                            std::vector<cv::Rect> wb;
                            std::vector<float> ws;
                            std::vector<int> wi;
                            cv::Mat warmup(inference_size_, inference_size_, CV_8UC3, cv::Scalar(0, 0, 0));
                            infer_dnn(warmup, wb, ws, wi);
                            RCLCPP_INFO(get_logger(), "[YOLO-CUDA] OpenCV CUDA FP16 warmup PASS");
                        } else {
                            if (require_cuda_) {
                                throw std::runtime_error(
                                    "[YOLO-CUDA] CPU backend requested while require_cuda=true");
                            }
                            net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
                            net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
                            RCLCPP_INFO(get_logger(), "[YOLO] Backend: CPU (explicitly allowed)");
                        }
                        model_ready_ = true;
                    }
                } catch (const cv::Exception& e) {
                    if (require_cuda_) {
                        RCLCPP_ERROR(get_logger(),
                            "[YOLO-CUDA] CUDA ONNX initialization failed and CPU fallback is disabled: %s",
                            e.what());
                        throw;
                    }
                    RCLCPP_INFO(get_logger(),
                        "[YOLO-MODEL] ONNX initialization unavailable; passthrough remains active: %s",
                        e.what());
                } catch (const std::exception& e) {
                    if (require_cuda_) {
                        RCLCPP_ERROR(get_logger(),
                            "[YOLO-CUDA] model/GPU initialization failed and CPU fallback is disabled: %s",
                            e.what());
                        throw;
                    }
                    RCLCPP_INFO(get_logger(),
                        "[YOLO-MODEL] model initialization unavailable; passthrough remains active: %s",
                        e.what());
                }
            }
        }

        // ── Subscriber ───────────────────────────────────────────────────────
        sub_ = create_subscription<sensor_msgs::msg::Image>(
            get_parameter("input_topic").as_string(),
            rclcpp::SensorDataQoS().keep_last(1),
            [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { image_callback(msg); });

        // ── Inference thread ─────────────────────────────────────────────────
        inference_thread_ = std::thread(&ObstacleDetectorNode::inference_loop, this);

        if (model_ready_) {
            const char* backend = use_tensorrt_ ? "TensorRT FP16" :
                                  (use_cuda_ ? "OpenCV DNN CUDA" : "OpenCV DNN CPU");
            RCLCPP_INFO(get_logger(),
                "YOLO obstacle detector READY: backend=%s, classes=%d, input=%dx%d, infer_max=%.1fHz, viz_max=%.1fHz, model=%s",
                backend, NUM_CLASSES, inference_size_, inference_size_, max_inference_fps_,
                max_visualization_fps_, model_path_.c_str());
            publish_status(std::string("backend=") + backend +
                           " model=" + model_path_ +
                           " input=" + std::to_string(inference_size_) +
                           " infer_max_fps=" + std::to_string(max_inference_fps_) +
                           " viz_max_fps=" + std::to_string(max_visualization_fps_) +
                           " mode=OBJECT_DETECTION_ONLY");
#ifdef WAREHOUSE_COCO_PROFILE
            RCLCPP_INFO(get_logger(),
                "[YOLO-WAREHOUSE] COCO-80 engine loaded; emitting class 0=person only at bounded rate");
#else
            RCLCPP_INFO(get_logger(),
                "[YOLO-CLASSES] 0=floor marking, 1=block, 2=front, 3=hole pallet, 4=pallet");
#endif
        } else if (allow_passthrough_without_model_) {
            publish_status("backend=PASSTHROUGH model=NOT_LOADED detections=disabled mode=OBJECT_DETECTION_ONLY");
            RCLCPP_INFO(get_logger(),
                "[YOLO-PASSTHROUGH] camera image remains available; detections are disabled until a valid model is supplied");
        } else {
            RCLCPP_ERROR(get_logger(),
                "[YOLO] no valid model and passthrough disabled");
        }
    }

    ~ObstacleDetectorNode() override {
        running_.store(false);
        cond_var_.notify_all();
        if (inference_thread_.joinable()) inference_thread_.join();
#if HAS_TENSORRT
        release_tensorrt_resources();
#endif
    }
private:
    // ── TensorRT helpers / initialization ───────────────────────────────────
#if HAS_TENSORRT
    bool cuda_ok(cudaError_t code, const char* what) {
        if (code == cudaSuccess) return true;
        RCLCPP_ERROR(get_logger(), "[YOLO-CUDA] %s failed: %s", what, cudaGetErrorString(code));
        return false;
    }

    void release_tensorrt_resources() noexcept {
        // Safe both after successful initialization and after a partially-failed
        // engine startup.  This prevents a stale TensorRT allocation from being
        // retained when the node intentionally falls back to OpenCV CUDA.
        if (cuda_stream_)        { cudaStreamSynchronize(cuda_stream_); cudaStreamDestroy(cuda_stream_); cuda_stream_ = nullptr; }
        if (device_image_)       { cudaFree(device_image_); device_image_ = nullptr; image_buffer_capacity_ = 0; }
        if (device_input_)       { cudaFree(device_input_); device_input_ = nullptr; }
        if (device_output_)      { cudaFree(device_output_); device_output_ = nullptr; }
        if (host_image_pinned_)  { cudaFreeHost(host_image_pinned_); host_image_pinned_ = nullptr; image_buffer_capacity_ = 0; }
        if (host_input_pinned_)  { cudaFreeHost(host_input_pinned_); host_input_pinned_ = nullptr; }
        if (host_output_pinned_) { cudaFreeHost(host_output_pinned_); host_output_pinned_ = nullptr; }
        if (context_)            { delete context_; context_ = nullptr; }
        if (engine_)             { delete engine_; engine_ = nullptr; }
        if (runtime_)            { delete runtime_; runtime_ = nullptr; }
    }

    static std::string dims_to_string(const nvinfer1::Dims& dims) {
        std::ostringstream oss;
        oss << "[";
        for (int i = 0; i < dims.nbDims; ++i) {
            if (i) oss << ",";
            oss << dims.d[i];
        }
        oss << "]";
        return oss.str();
    }

    static bool dims_are_resolved(const nvinfer1::Dims& dims) {
        if (dims.nbDims <= 0) return false;
        for (int i = 0; i < dims.nbDims; ++i) {
            if (dims.d[i] <= 0) return false;
        }
        return true;
    }

    bool configure_trt_output_layout(const nvinfer1::Dims& dims) {
        std::vector<int64_t> d;
        for (int i = 0; i < dims.nbDims; ++i) d.push_back(dims.d[i]);
        if (!d.empty() && d.front() == 1) d.erase(d.begin());
        if (d.size() != 2) {
            RCLCPP_WARN(get_logger(), "[TRT] unsupported detector output shape %s; expected [1,C,N] or [1,N,C]",
                        dims_to_string(dims).c_str());
            return false;
        }
        if (d[0] == EXPECTED_OUTPUT_CHANNELS && d[1] > 0) {
            trt_output_channel_first_ = true;
            trt_output_channels_ = static_cast<int>(d[0]);
            trt_output_points_ = static_cast<int>(d[1]);
        } else if (d[1] == EXPECTED_OUTPUT_CHANNELS && d[0] > 0) {
            trt_output_channel_first_ = false;
            trt_output_points_ = static_cast<int>(d[0]);
            trt_output_channels_ = static_cast<int>(d[1]);
        } else {
            RCLCPP_WARN(get_logger(),
                "[TRT] output shape %s does not contain %d channels (=4 bbox + %d classes)",
                dims_to_string(dims).c_str(), EXPECTED_OUTPUT_CHANNELS, NUM_CLASSES);
            return false;
        }
        trt_output_elements_ = static_cast<size_t>(trt_output_channels_) * trt_output_points_;
        return true;
    }

    bool ensure_trt_image_buffers(const cv::Mat& img) {
        if (img.empty() || img.type() != CV_8UC3) return false;
        const size_t row_bytes = static_cast<size_t>(img.cols) * 3U;
        const size_t required = row_bytes * static_cast<size_t>(img.rows);
        if (required <= image_buffer_capacity_ && device_image_ && host_image_pinned_) return true;

        if (device_image_) { cudaFree(device_image_); device_image_ = nullptr; }
        if (host_image_pinned_) { cudaFreeHost(host_image_pinned_); host_image_pinned_ = nullptr; }
        image_buffer_capacity_ = 0;
        if (!cuda_ok(cudaMalloc(reinterpret_cast<void**>(&device_image_), required), "cudaMalloc(device image)")) return false;
        if (!cuda_ok(cudaHostAlloc(reinterpret_cast<void**>(&host_image_pinned_), required, cudaHostAllocDefault),
                     "cudaHostAlloc(host image staging)")) {
            cudaFree(device_image_); device_image_ = nullptr;
            return false;
        }
        image_buffer_capacity_ = required;
        return true;
    }
#endif

    bool init_tensorrt() {
#if HAS_TENSORRT
        std::string engine_file = engine_path_;
        if (engine_file.empty()) {
            engine_file = model_path_;
            const auto pos = engine_file.rfind(".onnx");
            if (pos != std::string::npos) engine_file.replace(pos, 5, ".engine");
        }
        RCLCPP_INFO(get_logger(), "Loading TensorRT engine: %s", engine_file.c_str());

        std::ifstream fs(engine_file, std::ios::binary);
        if (!fs) {
            RCLCPP_WARN(get_logger(), "Cannot open engine file: %s", engine_file.c_str());
            return false;
        }
        fs.seekg(0, std::ios::end);
        const std::streamoff end_pos = fs.tellg();
        if (end_pos <= 0) {
            RCLCPP_WARN(get_logger(), "TensorRT engine is empty: %s", engine_file.c_str());
            return false;
        }
        const size_t size = static_cast<size_t>(end_pos);
        fs.seekg(0, std::ios::beg);
        std::vector<char> engine_data(size);
        if (!fs.read(engine_data.data(), static_cast<std::streamsize>(size))) {
            RCLCPP_WARN(get_logger(), "Failed to read complete TensorRT engine: %s", engine_file.c_str());
            return false;
        }

        runtime_ = nvinfer1::createInferRuntime(gLogger);
        if (!runtime_) {
            RCLCPP_WARN(get_logger(), "Failed to create TensorRT runtime");
            return false;
        }
        engine_ = runtime_->deserializeCudaEngine(engine_data.data(), size);
        if (!engine_) {
            RCLCPP_WARN(get_logger(), "Failed to deserialize TensorRT engine (stale/incompatible engine?)");
            return false;
        }
        context_ = engine_->createExecutionContext();
        if (!context_) {
            RCLCPP_WARN(get_logger(), "Failed to create TensorRT execution context");
            return false;
        }

        const int nb_io = engine_->getNbIOTensors();
        int input_count = 0;
        int output_count = 0;
        for (int i = 0; i < nb_io; ++i) {
            const char* name = engine_->getIOTensorName(i);
            if (!name) continue;
            const auto mode = engine_->getTensorIOMode(name);
            const auto dtype = engine_->getTensorDataType(name);
            const auto shape = engine_->getTensorShape(name);
            RCLCPP_INFO(get_logger(), "[TRT-IO] %s mode=%s dtype=%d shape=%s",
                name, mode == nvinfer1::TensorIOMode::kINPUT ? "INPUT" : "OUTPUT",
                static_cast<int>(dtype), dims_to_string(shape).c_str());
            if (mode == nvinfer1::TensorIOMode::kINPUT && input_count++ == 0) trt_input_name_ = name;
            if (mode == nvinfer1::TensorIOMode::kOUTPUT && output_count++ == 0) trt_output_name_ = name;
        }
        if (input_count != 1 || output_count != 1 || trt_input_name_.empty() || trt_output_name_.empty()) {
            RCLCPP_WARN(get_logger(), "[TRT] detector expects exactly 1 input and 1 output, engine has inputs=%d outputs=%d",
                        input_count, output_count);
            return false;
        }
        if (engine_->getTensorDataType(trt_input_name_.c_str()) != nvinfer1::DataType::kFLOAT ||
            engine_->getTensorDataType(trt_output_name_.c_str()) != nvinfer1::DataType::kFLOAT) {
            RCLCPP_WARN(get_logger(), "[TRT] current optimized path requires FP32 I/O tensors; internal FP16 engine layers are supported");
            return false;
        }

        nvinfer1::Dims input_dims = engine_->getTensorShape(trt_input_name_.c_str());
        if (input_dims.nbDims != 4) {
            RCLCPP_WARN(get_logger(), "[TRT] unsupported input shape %s; expected NCHW", dims_to_string(input_dims).c_str());
            return false;
        }
        if (input_dims.d[0] <= 0) input_dims.d[0] = 1;
        if (input_dims.d[1] <= 0) input_dims.d[1] = 3;
        if (input_dims.d[2] <= 0) input_dims.d[2] = inference_size_;
        if (input_dims.d[3] <= 0) input_dims.d[3] = inference_size_;
        if (input_dims.d[0] != 1 || input_dims.d[1] != 3 || input_dims.d[2] != input_dims.d[3]) {
            RCLCPP_WARN(get_logger(), "[TRT] input must be batch=1, RGB, square; resolved=%s", dims_to_string(input_dims).c_str());
            return false;
        }
        if (!context_->setInputShape(trt_input_name_.c_str(), input_dims)) {
            RCLCPP_WARN(get_logger(), "[TRT] setInputShape rejected %s", dims_to_string(input_dims).c_str());
            return false;
        }
        trt_input_h_ = input_dims.d[2];
        trt_input_w_ = input_dims.d[3];
        inference_size_ = trt_input_w_;  // keep bbox unletterbox path consistent with engine geometry
        trt_input_elements_ = static_cast<size_t>(3) * trt_input_h_ * trt_input_w_;

        const nvinfer1::Dims output_dims = context_->getTensorShape(trt_output_name_.c_str());
        if (!dims_are_resolved(output_dims) || !configure_trt_output_layout(output_dims)) return false;

        if (!cuda_ok(cudaStreamCreateWithFlags(&cuda_stream_, cudaStreamNonBlocking), "cudaStreamCreateWithFlags")) return false;
        if (!cuda_ok(cudaMalloc(&device_input_, trt_input_elements_ * sizeof(float)), "cudaMalloc(input)")) return false;
        if (!cuda_ok(cudaMalloc(&device_output_, trt_output_elements_ * sizeof(float)), "cudaMalloc(output)")) return false;
        if (!cuda_ok(cudaHostAlloc(reinterpret_cast<void**>(&host_input_pinned_), trt_input_elements_ * sizeof(float), cudaHostAllocDefault),
                     "cudaHostAlloc(input staging)")) return false;
        if (!cuda_ok(cudaHostAlloc(reinterpret_cast<void**>(&host_output_pinned_), trt_output_elements_ * sizeof(float), cudaHostAllocDefault),
                     "cudaHostAlloc(output staging)")) return false;
        if (!context_->setTensorAddress(trt_input_name_.c_str(), device_input_) ||
            !context_->setTensorAddress(trt_output_name_.c_str(), device_output_)) {
            RCLCPP_WARN(get_logger(), "[TRT] setTensorAddress failed");
            return false;
        }

#if HAS_CUDA_PREPROCESS
        gpu_preprocess_active_ = use_gpu_preprocess_;
#else
        gpu_preprocess_active_ = false;
        if (use_gpu_preprocess_) {
            RCLCPP_WARN(get_logger(), "[TRT] use_gpu_preprocess=true but package was built without nvcc; using pinned CPU preprocess");
        }
#endif

        // One warm-up catches a bad context/tensor contract at startup instead of
        // on the first live camera frame.
        if (!cuda_ok(cudaMemsetAsync(device_input_, 0, trt_input_elements_ * sizeof(float), cuda_stream_), "cudaMemsetAsync(warmup)")) return false;
        if (!context_->enqueueV3(cuda_stream_)) {
            RCLCPP_WARN(get_logger(), "[TRT] warmup enqueueV3 failed");
            return false;
        }
        if (!cuda_ok(cudaStreamSynchronize(cuda_stream_), "cudaStreamSynchronize(warmup)")) return false;

        RCLCPP_INFO(get_logger(),
            "TensorRT READY: TRT=%d.%d.%d input=%s %dx%d output=%s channels=%d points=%d layout=%s GPU-preprocess=%s",
            NV_TENSORRT_MAJOR, NV_TENSORRT_MINOR, NV_TENSORRT_PATCH,
            trt_input_name_.c_str(), trt_input_w_, trt_input_h_, trt_output_name_.c_str(),
            trt_output_channels_, trt_output_points_,
            trt_output_channel_first_ ? "C,N" : "N,C",
            gpu_preprocess_active_ ? "ON" : "OFF");
        return true;
#else
        return false;
#endif
    }

    // ── TensorRT inference ─────────────────────────────────────────────────
    void infer_tensorrt(const cv::Mat& img_bgr, std::vector<YOLODet>& detections) {
#if HAS_TENSORRT
        using namespace std::chrono;
        const auto t0 = steady_clock::now();
        if (!context_ || !cuda_stream_ || !device_input_ || !device_output_ || !host_output_pinned_) {
            throw std::runtime_error("TensorRT runtime is not initialized");
        }

        if (gpu_preprocess_active_) {
#if HAS_CUDA_PREPROCESS
            if (!ensure_trt_image_buffers(img_bgr)) throw std::runtime_error("failed to allocate GPU image staging buffers");
            const size_t row_bytes = static_cast<size_t>(img_bgr.cols) * 3U;
            for (int y = 0; y < img_bgr.rows; ++y) {
                std::memcpy(host_image_pinned_ + static_cast<size_t>(y) * row_bytes,
                            img_bgr.ptr(y), row_bytes);
            }
            const size_t image_bytes = row_bytes * static_cast<size_t>(img_bgr.rows);
            if (!cuda_ok(cudaMemcpyAsync(device_image_, host_image_pinned_, image_bytes,
                                         cudaMemcpyHostToDevice, cuda_stream_), "camera H2D")) {
                throw std::runtime_error("camera H2D failed");
            }
            const cudaError_t prep = launch_bgr8_letterbox_to_nchw(
                device_image_, img_bgr.cols, img_bgr.rows, static_cast<int>(row_bytes),
                static_cast<float*>(device_input_), trt_input_w_, trt_input_h_, cuda_stream_);
            if (!cuda_ok(prep, "GPU letterbox/preprocess")) throw std::runtime_error("GPU preprocess failed");
#else
            throw std::runtime_error("GPU preprocess was not compiled");
#endif
        } else {
            // Fallback remains allocation-free: pinned float staging is persistent.
            cv::Mat letterboxed;
            apply_letterbox(img_bgr, letterboxed, trt_input_w_);
            const int plane = trt_input_w_ * trt_input_h_;
            for (int h = 0; h < trt_input_h_; ++h) {
                const cv::Vec3b* row = letterboxed.ptr<cv::Vec3b>(h);
                for (int w = 0; w < trt_input_w_; ++w) {
                    const cv::Vec3b& px = row[w];
                    const int idx = h * trt_input_w_ + w;
                    host_input_pinned_[idx] = px[2] / 255.0f;
                    host_input_pinned_[plane + idx] = px[1] / 255.0f;
                    host_input_pinned_[2 * plane + idx] = px[0] / 255.0f;
                }
            }
            if (!cuda_ok(cudaMemcpyAsync(device_input_, host_input_pinned_,
                                         trt_input_elements_ * sizeof(float),
                                         cudaMemcpyHostToDevice, cuda_stream_), "input H2D")) {
                throw std::runtime_error("input H2D failed");
            }
        }
        const auto t1 = steady_clock::now();

        if (!context_->enqueueV3(cuda_stream_)) throw std::runtime_error("TensorRT enqueueV3 failed");
        if (!cuda_ok(cudaMemcpyAsync(host_output_pinned_, device_output_,
                                     trt_output_elements_ * sizeof(float),
                                     cudaMemcpyDeviceToHost, cuda_stream_), "output D2H")) {
            throw std::runtime_error("output D2H failed");
        }
        if (!cuda_ok(cudaStreamSynchronize(cuda_stream_), "inference stream sync")) {
            throw std::runtime_error("TensorRT stream synchronization failed");
        }
        const auto t2 = steady_clock::now();

        auto at = [this](int channel, int point) -> float {
            const size_t idx = trt_output_channel_first_
                ? static_cast<size_t>(channel) * trt_output_points_ + point
                : static_cast<size_t>(point) * trt_output_channels_ + channel;
            return host_output_pinned_[idx];
        };
        std::vector<YOLODet> raw_dets;
        raw_dets.reserve(256);
        for (int i = 0; i < trt_output_points_; ++i) {
            float max_conf = 0.0f;
            int best_cls = 0;
            for (int c = 4; c < trt_output_channels_; ++c) {
                const float score = at(c, i);
                if (score > max_conf) { max_conf = score; best_cls = c - 4; }
            }
            if (!std::isfinite(max_conf) || max_conf < conf_thresh_) continue;
            YOLODet d{};
            d.bbox[0] = at(0, i);
            d.bbox[1] = at(1, i);
            d.bbox[2] = at(2, i);
            d.bbox[3] = at(3, i);
            d.cls_conf = max_conf;
            d.class_id = best_cls;
            if (std::isfinite(d.bbox[0]) && std::isfinite(d.bbox[1]) &&
                std::isfinite(d.bbox[2]) && std::isfinite(d.bbox[3])) raw_dets.push_back(d);
        }
        nms_yolo(raw_dets, iou_thresh_, detections);
        const auto t3 = steady_clock::now();

        last_preprocess_ms_ = duration_cast<microseconds>(t1 - t0).count() / 1000.0;
        last_inference_ms_ = duration_cast<microseconds>(t2 - t1).count() / 1000.0;
        last_postprocess_ms_ = duration_cast<microseconds>(t3 - t2).count() / 1000.0;
        last_total_ms_ = duration_cast<microseconds>(t3 - t0).count() / 1000.0;
#else
        (void)img_bgr; (void)detections;
#endif
    }

    // ── OpenCV DNN inference ──────────────────────────────────────────────
    // Supports both [1,C,N] and [1,N,C] Ultralytics detector exports.
    void infer_dnn(const cv::Mat& img, std::vector<cv::Rect>& boxes,
                   std::vector<float>& scores, std::vector<int>& ids) {
        using namespace std::chrono;
        const auto t0 = steady_clock::now();
        cv::Mat letterboxed;
        apply_letterbox(img, letterboxed, inference_size_);
        cv::Mat input_blob = cv::dnn::blobFromImage(
            letterboxed, 1.0 / 255.0,
            cv::Size(inference_size_, inference_size_), cv::Scalar(),
            true, false, CV_32F);
        const auto t1 = steady_clock::now();

        net_.setInput(input_blob);
        std::vector<cv::Mat> outs;
        net_.forward(outs, net_.getUnconnectedOutLayersNames());
        if (outs.empty()) return;
        cv::Mat out = outs[0];
        const auto t2 = steady_clock::now();

        cv::Mat matrix;
        if (out.dims == 3 && out.size[0] == 1) {
            if (out.size[1] == EXPECTED_OUTPUT_CHANNELS) {
                matrix = out.reshape(1, {out.size[1], out.size[2]});
            } else if (out.size[2] == EXPECTED_OUTPUT_CHANNELS) {
                cv::Mat rows = out.reshape(1, {out.size[1], out.size[2]});
                cv::transpose(rows, matrix);
            }
        } else if (out.dims == 2) {
            if (out.rows == EXPECTED_OUTPUT_CHANNELS) matrix = out;
            else if (out.cols == EXPECTED_OUTPUT_CHANNELS) cv::transpose(out, matrix);
        }
        if (matrix.empty() || matrix.rows != EXPECTED_OUTPUT_CHANNELS) {
            std::ostringstream shape;
            shape << "[";
            for (int i = 0; i < out.dims; ++i) { if (i) shape << ","; shape << out.size[i]; }
            shape << "]";
            throw std::runtime_error(
                "YOLO model output mismatch: expected [1," + std::to_string(EXPECTED_OUTPUT_CHANNELS) +
                ",N] or [1,N," + std::to_string(EXPECTED_OUTPUT_CHANNELS) + "], got " + shape.str());
        }

        LetterBox lb = compute_letterbox_params(img.cols, img.rows, inference_size_);
        for (int i = 0; i < matrix.cols; ++i) {
            int best_cls = 0;
            float best_score = 0.0f;
            for (int c = 4; c < matrix.rows; ++c) {
                const float sc = matrix.at<float>(c, i);
                if (sc > best_score) { best_score = sc; best_cls = c - 4; }
            }
            if (!std::isfinite(best_score) || best_score < conf_thresh_) continue;
            cv::Rect box = unletterbox_bbox(
                matrix.at<float>(0, i), matrix.at<float>(1, i),
                matrix.at<float>(2, i), matrix.at<float>(3, i),
                img.cols, img.rows, lb);
            if (box.area() <= 0) continue;
            boxes.push_back(box);
            scores.push_back(best_score);
            ids.push_back(best_cls);
        }

        // OpenCV's NMSBoxes is class-agnostic.  Run it per class so overlapping
        // application classes do not incorrectly erase each other.
        std::vector<cv::Rect> final_boxes;
        std::vector<float> final_scores;
        std::vector<int> final_ids;
        for (int cls = 0; cls < NUM_CLASSES; ++cls) {
            std::vector<cv::Rect> cls_boxes;
            std::vector<float> cls_scores;
            std::vector<int> original;
            for (size_t i = 0; i < boxes.size(); ++i) {
                if (ids[i] == cls) {
                    cls_boxes.push_back(boxes[i]);
                    cls_scores.push_back(scores[i]);
                    original.push_back(static_cast<int>(i));
                }
            }
            std::vector<int> keep;
            cv::dnn::NMSBoxes(cls_boxes, cls_scores, conf_thresh_, iou_thresh_, keep);
            for (int k : keep) {
                const int src = original.at(static_cast<size_t>(k));
                final_boxes.push_back(boxes.at(static_cast<size_t>(src)));
                final_scores.push_back(scores.at(static_cast<size_t>(src)));
                final_ids.push_back(ids.at(static_cast<size_t>(src)));
            }
        }
        boxes.swap(final_boxes); scores.swap(final_scores); ids.swap(final_ids);
        const auto t3 = steady_clock::now();
        last_preprocess_ms_ = duration_cast<microseconds>(t1 - t0).count() / 1000.0;
        last_inference_ms_ = duration_cast<microseconds>(t2 - t1).count() / 1000.0;
        last_postprocess_ms_ = duration_cast<microseconds>(t3 - t2).count() / 1000.0;
        last_total_ms_ = duration_cast<microseconds>(t3 - t0).count() / 1000.0;
    }

    // ── ROS callbacks ─────────────────────────────────────────────────────
    void image_callback(sensor_msgs::msg::Image::ConstSharedPtr msg) {
        if (!msg || msg->width == 0 || msg->height == 0 || msg->data.empty()) return;
        const std::size_t required = static_cast<std::size_t>(msg->step) * msg->height;
        if (msg->step == 0 || msg->data.size() < required) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "[YOLO-IMAGE] invalid Image buffer: encoding=%s %ux%u step=%u bytes=%zu",
                msg->encoding.c_str(), msg->width, msg->height, msg->step, msg->data.size());
            return;
        }

        // Keep only the newest ROS message. For the normal bgr8 camera path the
        // inference thread wraps msg->data directly as cv::Mat, eliminating the
        // previous 1280x720 clone in every subscriber callback (~83 MB/s at 30 Hz).
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            frames_received_.fetch_add(1, std::memory_order_relaxed);
            if (new_frame_ready_ && latest_msg_) {
                // Latest-frame-wins is intentional for bounded perception latency.
                frames_dropped_.fetch_add(1, std::memory_order_relaxed);
            }
            latest_msg_ = std::move(msg);
            new_frame_ready_ = true;
        }
        cond_var_.notify_one();
    }

    bool image_message_to_bgr(const sensor_msgs::msg::Image::ConstSharedPtr& msg,
                              cv::Mat& bgr) {
        if (!msg) return false;
        try {
            if (msg->encoding == "bgr8") {
                bgr = cv::Mat(msg->height, msg->width, CV_8UC3,
                              const_cast<uint8_t*>(msg->data.data()), msg->step);
            } else if (msg->encoding == "rgb8") {
                cv::Mat src(msg->height, msg->width, CV_8UC3,
                            const_cast<uint8_t*>(msg->data.data()), msg->step);
                cv::cvtColor(src, bgr, cv::COLOR_RGB2BGR);
            } else if (msg->encoding == "rgba8") {
                cv::Mat src(msg->height, msg->width, CV_8UC4,
                            const_cast<uint8_t*>(msg->data.data()), msg->step);
                cv::cvtColor(src, bgr, cv::COLOR_RGBA2BGR);
            } else if (msg->encoding == "bgra8") {
                cv::Mat src(msg->height, msg->width, CV_8UC4,
                            const_cast<uint8_t*>(msg->data.data()), msg->step);
                cv::cvtColor(src, bgr, cv::COLOR_BGRA2BGR);
            } else if (msg->encoding == "mono8") {
                cv::Mat src(msg->height, msg->width, CV_8UC1,
                            const_cast<uint8_t*>(msg->data.data()), msg->step);
                cv::cvtColor(src, bgr, cv::COLOR_GRAY2BGR);
            } else {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "[YOLO-IMAGE] unsupported encoding '%s' (expected rgb8/bgr8/rgba8/bgra8/mono8)",
                    msg->encoding.c_str());
                return false;
            }
        } catch (const cv::Exception& e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "[YOLO-IMAGE] conversion failed: %s", e.what());
            return false;
        }
        return !bgr.empty();
    }

    void inference_loop() {
        const auto min_interval = std::chrono::duration<double>(1.0 / max_inference_fps_);
        auto last_time = std::chrono::steady_clock::now() - min_interval;

        while (running_.load() && rclcpp::ok()) {
            sensor_msgs::msg::Image::ConstSharedPtr msg;

            {
                std::unique_lock<std::mutex> lock(frame_mutex_);
                cond_var_.wait_for(lock, std::chrono::milliseconds(500),
                                   [this]{ return new_frame_ready_; });
                if (!new_frame_ready_ || !latest_msg_) {
                    new_frame_ready_ = false;
                    continue;
                }
                msg = std::move(latest_msg_);
                latest_msg_.reset();
                new_frame_ready_ = false;
            }

            if (!msg) continue;

            const auto due = last_time + min_interval;
            const auto now = std::chrono::steady_clock::now();
            if (now < due) {
                std::this_thread::sleep_until(due);
                // A newer camera message may have arrived while waiting. Prefer
                // the freshest one so inference latency does not accumulate.
                std::lock_guard<std::mutex> lock(frame_mutex_);
                if (new_frame_ready_ && latest_msg_) {
                    msg = std::move(latest_msg_);
                    latest_msg_.reset();
                    new_frame_ready_ = false;
                }
            }
            last_time = std::chrono::steady_clock::now();

            cv::Mat img;
            if (!image_message_to_bgr(msg, img)) continue;
            process_frame(img, msg->header);
        }
    }

    void process_frame(const cv::Mat& img, const std_msgs::msg::Header& header) {
        // Detection/control work is independent from GUI visualization. The
        // annotated 1280x720 frame is built only when a subscriber exists and
        // at a bounded rate, so opening/closing GUI tabs cannot throttle YOLO.
        const auto viz_now = std::chrono::steady_clock::now();
        const auto viz_interval = std::chrono::duration<double>(1.0 / max_visualization_fps_);
        const bool viz_due = last_visualization_time_.time_since_epoch().count() == 0 ||
                             (viz_now - last_visualization_time_) >= viz_interval;
        render_visual_this_frame_ = annotated_pub_->get_subscription_count() > 0 && viz_due;
        if (render_visual_this_frame_) {
            img.copyTo(vis_buf_);
        }

        yolo_obstacle_detection_ros2::msg::ObstacleArray obstacles;
        obstacles.header = header;

        if (!model_ready_) {
            if (allow_passthrough_without_model_) {
                if (render_visual_this_frame_) {
                    draw_status_overlay(vis_buf_, 0);
                    publish_annotated(vis_buf_, header);
                    last_visualization_time_ = viz_now;
                }
                obstacle_pub_->publish(obstacles);
            }
            return;
        }

        LetterBox lb = compute_letterbox_params(img.cols, img.rows, inference_size_);

        try {
            if (use_tensorrt_) {
                std::vector<YOLODet> detections;
                infer_tensorrt(img, detections);
                for (const auto& d : detections) {
                    cv::Rect box = unletterbox_bbox(d.bbox[0], d.bbox[1], d.bbox[2], d.bbox[3],
                                                    img.cols, img.rows, lb);
                    if (box.area() <= 0) continue;
                    float cx = box.x + box.width * 0.5f;
                    float cy = box.y + box.height * 0.5f;
                    float bbox_img[4] = { cx, cy, static_cast<float>(box.width), static_cast<float>(box.height) };
                    process_detection(img, bbox_img, d.cls_conf, d.class_id, obstacles);
                }
            } else {
                std::vector<cv::Rect> boxes;
                std::vector<float> scores;
                std::vector<int> ids;
                infer_dnn(img, boxes, scores, ids);
                for (size_t i = 0; i < boxes.size(); ++i) {
                    float cx = boxes[i].x + boxes[i].width * 0.5f;
                    float cy = boxes[i].y + boxes[i].height * 0.5f;
                    float bbox[4] = { cx, cy,
                                      static_cast<float>(boxes[i].width),
                                      static_cast<float>(boxes[i].height) };
                    process_detection(img, bbox, scores[i], ids[i], obstacles);
                }
            }
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(get_logger(),
                "[YOLO-INFERENCE] CUDA/OpenCV inference failed: %s", e.what());
            if (allow_passthrough_without_model_ && !require_cuda_) {
                model_ready_ = false;
                if (render_visual_this_frame_) {
                    draw_status_overlay(vis_buf_, 0);
                    publish_annotated(vis_buf_, header);
                    last_visualization_time_ = viz_now;
                }
                obstacle_pub_->publish(obstacles);
                return;
            }
            return;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "[YOLO-INFERENCE] inference failed: %s", e.what());
            return;
        }

        if (render_visual_this_frame_) {
            draw_status_overlay(vis_buf_, static_cast<int>(obstacles.obstacles.size()));
            publish_annotated(vis_buf_, header);
            last_visualization_time_ = viz_now;
        }
        obstacle_pub_->publish(obstacles);
        publish_performance(header, static_cast<int>(obstacles.obstacles.size()));
    }

    void process_detection(const cv::Mat& img, const float bbox[4],
                          float conf, int class_id,
                          yolo_obstacle_detection_ros2::msg::ObstacleArray& obstacles) {
        float cx = bbox[0], cy = bbox[1];
        float bw = bbox[2], bh = bbox[3];
        int x1 = std::max(0, static_cast<int>(cx - bw * 0.5f));
        int y1 = std::max(0, static_cast<int>(cy - bh * 0.5f));
        int x2 = std::min(img.cols, static_cast<int>(cx + bw * 0.5f));
        int y2 = std::min(img.rows, static_cast<int>(cy + bh * 0.5f));
        cv::Rect box(x1, y1, x2 - x1, y2 - y1);
        if (box.area() <= 0) return;

#ifdef WAREHOUSE_COCO_PROFILE
        // Keep secondary semantics intentionally tiny: person is the only
        // additional dataset class used at runtime. All other geometry is
        // still detected by the existing pallet model and LiDAR corridor.
        if (class_id != 0) return;
#endif
        const std::string cname = class_name_for_id(class_id);
        const std::string cat = category(cname);
        const double dist = estimate_distance(box, img.rows);
        const bool on_path = (cx > img.cols * 0.28 && cx < img.cols * 0.72
                              && cy > img.rows * 0.45);
        const bool danger = dist < danger_zone_;
        const bool warn = dist < warning_distance_;

        yolo_obstacle_detection_ros2::msg::Obstacle obs;
        obs.class_id = class_id;
        obs.class_name = cname;
        obs.category = cat;
        obs.confidence = conf;
        obs.x1 = box.x; obs.y1 = box.y;
        obs.x2 = box.x + box.width; obs.y2 = box.y + box.height;
        obs.center_x = cx; obs.center_y = cy;
        obs.on_path = on_path;
        obs.in_danger_zone = danger;
        obstacles.obstacles.push_back(obs);
        if (cat == "dynamic") ++obstacles.dynamic_count;
        else if (cat == "static") ++obstacles.static_count;
        else if (cat == "pallet") ++obstacles.pallet_count;
        else ++obstacles.other_count;
        if (danger) obstacles.warning_active = true;
        if (on_path && (danger || warn)) obstacles.path_obstacles.push_back(cname);

        // Detection-only drawing: class box + confidence. No center target,
        // no path lane, no docking-safe badge and no alignment arrow.
        if (render_visual_this_frame_) {
            const cv::Scalar color(0, 255, 0);
            cv::rectangle(vis_buf_, box, color, 2);
            cv::putText(vis_buf_,
                        cv::format("%s [%.2f]", cname.c_str(), conf),
                        cv::Point(box.x, std::max(14, box.y - 3)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, color, 1);
        }
    }

    void draw_status_overlay(cv::Mat& vis, int object_count) {
        char buf[192];
        cv::Scalar txt_clr;
        if (!model_ready_) {
            snprintf(buf, sizeof(buf), "CAMERA OK | YOLO MODEL NOT LOADED | DETECTION ONLY");
            txt_clr = cv::Scalar(0, 200, 255);
        } else {
            std::string model_name = model_path_;
            const std::size_t slash = model_name.find_last_of("/\\");
            if (slash != std::string::npos) model_name = model_name.substr(slash + 1);
            snprintf(buf, sizeof(buf), "YOLOv8 DETECTION ONLY | objects=%d | %s",
                     object_count, model_name.c_str());
            txt_clr = cv::Scalar(0, 255, 0);
        }
        int baseline = 0;
        cv::Size sz = cv::getTextSize(buf, cv::FONT_HERSHEY_SIMPLEX, 0.55, 1.5, &baseline);
        cv::rectangle(vis, cv::Point(5, 5), cv::Point(sz.width + 12, sz.height + 16),
                      cv::Scalar(0, 0, 0), -1);
        cv::putText(vis, buf, cv::Point(10, sz.height + 12),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, txt_clr, 1.5);
    }

    std::string category(const std::string& c) const {
        // floor marking / block / front → static (warehouse infrastructure)
        static const std::set<std::string> sta{
            "floor marking","block","front","wall","column","rack","shelf"};
        // hole pallet / pallet → pallet
        static const std::set<std::string> plt{
            "hole pallet","pallet","euro pallet","wooden pallet"};
        static const std::set<std::string> dyn{
            "person"};
        if (dyn.count(c)) return "dynamic";
        if (sta.count(c)) return "static";
        if (plt.count(c)) return "pallet";
        return "other";
    }

    double estimate_distance(const cv::Rect& b, int img_h) const {
        if (b.height <= 0) return std::numeric_limits<double>::infinity();
        // Ponytail: assumes known object height; floor marking has ~20cm height,
        // front (fork truck front) ~50cm. Approximate average = 35cm.
        static constexpr float ASSUMED_OBJECT_HEIGHT = 0.35f; // meters
        return ASSUMED_OBJECT_HEIGHT * img_h / static_cast<float>(b.height);
    }

    void publish_performance(const std_msgs::msg::Header& header, int detection_count) {
        if (!performance_pub_) return;
        const auto now_steady = std::chrono::steady_clock::now();
        ++performance_window_frames_;
        frames_processed_.fetch_add(1, std::memory_order_relaxed);
        if (performance_window_start_.time_since_epoch().count() == 0) {
            performance_window_start_ = now_steady;
            return;
        }
        const double elapsed = std::chrono::duration<double>(now_steady - performance_window_start_).count();
        if (elapsed < 1.0) return;
        const double fps = performance_window_frames_ / std::max(1e-6, elapsed);
        double frame_age_ms = 0.0;
        if (header.stamp.sec != 0 || header.stamp.nanosec != 0) {
            const double age = (get_clock()->now() - rclcpp::Time(header.stamp)).seconds();
            if (std::isfinite(age) && age >= 0.0) frame_age_ms = age * 1000.0;
        }
        const char* backend = use_tensorrt_ ? "TensorRT_FP16" : (use_cuda_ ? "OpenCV_CUDA_FP16" : "OpenCV_CPU");
        std::ostringstream oss;
        oss.setf(std::ios::fixed); oss.precision(3);
        oss << "backend=" << backend
            << " fps=" << fps
            << " preprocess_ms=" << last_preprocess_ms_
            << " inference_ms=" << last_inference_ms_
            << " postprocess_ms=" << last_postprocess_ms_
            << " total_ms=" << last_total_ms_
            << " frame_age_ms=" << frame_age_ms
            << " detections=" << detection_count
            << " received=" << frames_received_.load(std::memory_order_relaxed)
            << " processed=" << frames_processed_.load(std::memory_order_relaxed)
            << " dropped=" << frames_dropped_.load(std::memory_order_relaxed)
            << " gpu_preprocess=" << (gpu_preprocess_active_ ? 1 : 0)
            << " input=" << inference_size_;
#if HAS_TENSORRT
        if (use_tensorrt_) oss << " output_points=" << trt_output_points_;
#endif
        std_msgs::msg::String msg; msg.data = oss.str(); performance_pub_->publish(msg);
        performance_window_frames_ = 0;
        performance_window_start_ = now_steady;
    }

    void publish_status(const std::string& text) {
        if (!status_pub_) return;
        std_msgs::msg::String msg;
        msg.data = text;
        status_pub_->publish(msg);
    }

    void publish_annotated(const cv::Mat& vis, const std_msgs::msg::Header& header) {
        out_msg_.header = header;
        out_msg_.height = vis.rows;
        out_msg_.width = vis.cols;
        out_msg_.encoding = "bgr8";
        out_msg_.is_bigendian = false;
        out_msg_.step = vis.cols * vis.elemSize();
        out_msg_.data.assign(vis.data, vis.data + out_msg_.step * out_msg_.height);
        annotated_pub_->publish(out_msg_);
    }

    // Members
    std::string model_path_, engine_path_;
    double conf_thresh_{0.5}, iou_thresh_{0.45};
    double danger_zone_{2.0}, warning_distance_{1.5};
    double max_inference_fps_{30.0};
    double max_visualization_fps_{15.0};
    int inference_size_{640};
    bool use_tensorrt_{false};
    bool use_cuda_{false};
    bool require_cuda_{true};
    bool use_gpu_preprocess_{true};
    bool gpu_preprocess_active_{false};
    bool allow_passthrough_without_model_{true};
    bool model_ready_{false};
    bool render_visual_this_frame_{false};
    std::chrono::steady_clock::time_point last_visualization_time_;
    std::atomic<bool> running_{true};
    std::atomic<uint64_t> frames_received_{0};
    std::atomic<uint64_t> frames_processed_{0};
    std::atomic<uint64_t> frames_dropped_{0};
    double last_preprocess_ms_{0.0};
    double last_inference_ms_{0.0};
    double last_postprocess_ms_{0.0};
    double last_total_ms_{0.0};
    uint64_t performance_window_frames_{0};
    std::chrono::steady_clock::time_point performance_window_start_;

    std::thread inference_thread_;
    std::mutex frame_mutex_;
    std::condition_variable cond_var_;
    sensor_msgs::msg::Image::ConstSharedPtr latest_msg_;
    bool new_frame_ready_{false};

    cv::dnn::Net net_;

#if HAS_TENSORRT
    nvinfer1::IRuntime* runtime_{nullptr};
    nvinfer1::ICudaEngine* engine_{nullptr};
    nvinfer1::IExecutionContext* context_{nullptr};
    std::string trt_input_name_;
    std::string trt_output_name_;
    int trt_input_h_{DEFAULT_INPUT_SIZE};
    int trt_input_w_{DEFAULT_INPUT_SIZE};
    int trt_output_channels_{EXPECTED_OUTPUT_CHANNELS};
    int trt_output_points_{0};
    bool trt_output_channel_first_{true};
    size_t trt_input_elements_{0};
    size_t trt_output_elements_{0};
    size_t image_buffer_capacity_{0};
    void* device_input_{nullptr};
    void* device_output_{nullptr};
    unsigned char* device_image_{nullptr};
    unsigned char* host_image_pinned_{nullptr};
    float* host_input_pinned_{nullptr};
    float* host_output_pinned_{nullptr};
    cudaStream_t cuda_stream_{nullptr};
#endif

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<yolo_obstacle_detection_ros2::msg::ObstacleArray>::SharedPtr obstacle_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr performance_pub_;
    cv::Mat vis_buf_;
    sensor_msgs::msg::Image out_msg_;
};

#ifdef YOLO_COMPONENT_BUILD
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ObstacleDetectorNode)
#else
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
#endif
