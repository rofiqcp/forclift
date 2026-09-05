/*
 * perception_node.cpp
 *
 * Node ROS 2 Humble untuk pipeline persepsi kendaraan:
 *
 *   Kamera Astra/V4L2
 *       -> konversi YUYV ke BGR di CUDA
 *       -> letterbox + normalisasi di CUDA
 *       -> inferensi YOLOP-v2 menggunakan TensorRT
 *       -> decoding deteksi + NMS di CUDA
 *       -> segmentasi drivable area dan lane di CUDA
 *       -> penggambaran mask serta bounding box di CUDA
 *       -> pinned host ring-buffer latest-frame-only
 *       -> sensor_msgs/Image
 *       -> RViz2
 *
 * Prinsip desain low-latency:
 * 1. Frame kamera lama sengaja dibuang ketika pipeline tertinggal.
 * 2. RViz/DDS tidak boleh menahan thread inferensi TensorRT.
 * 3. Hanya data yang memiliki subscriber yang disalin dari GPU ke RAM.
 * 4. Frame annotated memakai pinned memory agar transfer GPU->CPU lebih efisien.
 * 5. QoS image memakai Reliable + depth 1 agar kompatibel dengan RViz Humble; thread publisher tetap latest-frame-only.
 *
 * Catatan penting:
 * RViz2 standar tetap memerlukan sensor_msgs/Image di RAM. Karena itu masih ada
 * satu transfer Device-to-Host untuk frame final. Seluruh proses berat sebelum
 * transfer tersebut tetap dilakukan pada GPU.
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

#include "perception/perception_safety_core.hpp"

#include <NvInfer.h>
#include <cuda_runtime.h>

#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/opengl.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/poll.h>
#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace perception {

// ============================================================================
// UTILITAS UMUM DAN PEMERIKSAAN ERROR CUDA
// ============================================================================

#define CUDA_CHECK(call) do { \
  const cudaError_t _e = (call); \
  if (_e != cudaSuccess) { \
    std::ostringstream _oss; \
    _oss << "CUDA error " << cudaGetErrorName(_e) << ": " << cudaGetErrorString(_e) \
         << " at " << __FILE__ << ':' << __LINE__; \
    throw std::runtime_error(_oss.str()); \
  } \
} while (0)

// Menjalankan ioctl dan mengulanginya apabila terinterupsi sinyal EINTR.
// Fungsi kecil ini mencegah kegagalan kamera palsu akibat interupsi sementara.
static int xioctl(int fd, unsigned long request, void *arg) {
  int result;
  do {
    result = ::ioctl(fd, request, arg);
  } while (result == -1 && errno == EINTR);
  return result;
}

// Mengubah kode FOURCC V4L2, misalnya YUYV atau MJPG, menjadi teks yang mudah dibaca.
// Fungsi: Mengubah kode pixel-format FOURCC V4L2 menjadi teks untuk log diagnostik.
static std::string fourccToString(uint32_t value) {
  std::string result(4, ' ');
  result[0] = static_cast<char>(value & 0xFFU);
  result[1] = static_cast<char>((value >> 8U) & 0xFFU);
  result[2] = static_cast<char>((value >> 16U) & 0xFFU);
  result[3] = static_cast<char>((value >> 24U) & 0xFFU);
  return result;
}

namespace fs = std::filesystem;

// Fungsi: Menormalisasi path engine/model agar pencarian file tidak bergantung working directory.
static fs::path normalizedPath(const fs::path &path) {
  std::error_code error;
  fs::path result = fs::weakly_canonical(path, error);
  if (!error) return result;
  result = fs::absolute(path, error);
  return error ? path : result.lexically_normal();
}

// Fungsi: Memastikan kandidat engine benar-benar file reguler yang tidak kosong.
static bool nonEmptyRegularFile(const fs::path &path) {
  std::error_code error;
  return fs::is_regular_file(path, error) && !error && fs::file_size(path, error) > 0U && !error;
}

// Logger TensorRT. Hanya pesan warning dan error yang dicetak agar terminal tidak
// dipenuhi log internal TensorRT yang tidak diperlukan saat kendaraan berjalan.
class TrtLogger final : public nvinfer1::ILogger {
public:
  void log(Severity severity, const char *message) noexcept override {
    if (message == nullptr) {
      return;
    }
    const std::string text(message);
    if (text.find("Using an engine plan file across different models of devices") !=
      std::string::npos)
    {
      // This warning is emitted for some Jetson engine metadata combinations.
      // The loader below still rejects the engine if deserialization really fails.
      return;
    }
    if (severity <= Severity::kWARNING) {
      std::fprintf(stderr, "[TensorRT] %s\n", message);
    }
  }
};

template<typename T>
struct TrtDelete {
  // Fungsi: Deleter RAII untuk object TensorRT agar resource dilepas otomatis.
  void operator()(T *object) const noexcept { delete object; }
};

template<typename T>
using TrtUniquePtr = std::unique_ptr<T, TrtDelete<T>>;

// Menghitung jumlah elemen tensor statis. Dimensi dinamis sengaja ditolak karena
// paket ini dirancang untuk engine YOLOP-v2 dengan ukuran input tetap.
// Fungsi: Menghitung jumlah elemen tensor statis dan menolak dimensi dinamis/tidak valid.
static size_t tensorVolume(const nvinfer1::Dims &dims) {
  if (dims.nbDims <= 0) {
    throw std::runtime_error("TensorRT tensor has no dimensions");
  }
  size_t volume = 1;
  for (int i = 0; i < dims.nbDims; ++i) {
    const int64_t d = static_cast<int64_t>(dims.d[i]);
    if (d <= 0) {
      throw std::runtime_error("Dynamic/invalid TensorRT dimensions are not supported by this runtime package");
    }
    volume *= static_cast<size_t>(d);
  }
  return volume;
}

// Informasi satu tensor keluaran TensorRT beserta buffer device miliknya.
struct OutputTensor {
  std::string name;
  nvinfer1::Dims dims{};
  nvinfer1::DataType type{nvinfer1::DataType::kFLOAT};
  void *device{nullptr};
  size_t elements{0};
};

// ============================================================================
// PENGELOLA ENGINE TENSORRT
// ============================================================================
// Kelas ini membaca file .engine, mengenali input/output YOLOP-v2, mengalokasikan
// buffer keluaran di VRAM, dan menjalankan enqueueV3 pada CUDA stream utama.
class TensorRtEngine final {
public:
  TensorRtEngine() = default;
  // Fungsi: Destructor engine; melepaskan seluruh buffer TensorRT/CUDA melalui reset().
  ~TensorRtEngine() { reset(); }

  TensorRtEngine(const TensorRtEngine &) = delete;
  TensorRtEngine &operator=(const TensorRtEngine &) = delete;

  // Memuat engine TensorRT dari disk dan menghubungkan semua alamat tensor.
  // File .engine harus dibuat untuk versi TensorRT/GPU yang kompatibel.
  void load(const std::string &engine_path, float *device_input) {
    reset();

    std::ifstream input(engine_path, std::ios::binary | std::ios::ate);
    if (!input) {
      throw std::runtime_error("Cannot open TensorRT engine: " + engine_path);
    }
    const std::streamsize size = input.tellg();
    if (size <= 0) {
      throw std::runtime_error("TensorRT engine is empty: " + engine_path);
    }
    input.seekg(0, std::ios::beg);
    std::vector<char> bytes(static_cast<size_t>(size));
    if (!input.read(bytes.data(), size)) {
      throw std::runtime_error("Failed reading TensorRT engine: " + engine_path);
    }

    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    if (!runtime_) {
      throw std::runtime_error("createInferRuntime() failed");
    }
    engine_.reset(runtime_->deserializeCudaEngine(bytes.data(), bytes.size()));
    if (!engine_) {
      throw std::runtime_error(
        "deserializeCudaEngine() failed. TensorRT engine files are tied to the TensorRT/GPU platform; rebuild the .engine on this target if versions differ.");
    }
    context_.reset(engine_->createExecutionContext());
    if (!context_) {
      throw std::runtime_error("createExecutionContext() failed");
    }

    int input_count = 0;
    const int io_count = engine_->getNbIOTensors();
    for (int i = 0; i < io_count; ++i) {
      const char *name_c = engine_->getIOTensorName(i);
      if (!name_c) {
        throw std::runtime_error("TensorRT returned a null I/O tensor name");
      }
      const std::string name(name_c);
      const nvinfer1::Dims dims = engine_->getTensorShape(name_c);
      const auto mode = engine_->getTensorIOMode(name_c);
      const auto type = engine_->getTensorDataType(name_c);

      if (type != nvinfer1::DataType::kFLOAT) {
        throw std::runtime_error("Only FP32 TensorRT I/O tensors are supported. Tensor '" + name + "' has a different I/O type.");
      }

      if (mode == nvinfer1::TensorIOMode::kINPUT) {
        ++input_count;
        if (dims.nbDims != 4 || static_cast<int64_t>(dims.d[0]) != 1 || static_cast<int64_t>(dims.d[1]) != 3) {
          throw std::runtime_error("Expected one static NCHW input [1,3,H,W], got tensor: " + name);
        }
        input_name_ = name;
        input_dims_ = dims;
        input_elements_ = tensorVolume(dims);
        if (!context_->setTensorAddress(name.c_str(), device_input)) {
          throw std::runtime_error("setTensorAddress() failed for input: " + name);
        }
      } else {
        OutputTensor out;
        out.name = name;
        out.dims = dims;
        out.type = type;
        out.elements = tensorVolume(dims);
        CUDA_CHECK(cudaMalloc(&out.device, out.elements * sizeof(float)));
        if (!context_->setTensorAddress(name.c_str(), out.device)) {
          cudaFree(out.device);
          throw std::runtime_error("setTensorAddress() failed for output: " + name);
        }
        outputs_.push_back(out);
      }
    }

    if (input_count != 1) {
      throw std::runtime_error("Expected exactly one TensorRT input tensor");
    }
    identifyYolopOutputs();
  }

  // Fungsi: Menjalankan inference TensorRT asynchronous pada CUDA stream utama.
  bool enqueue(cudaStream_t stream) {
    return context_ && context_->enqueueV3(stream);
  }

  // Fungsi: Mengembalikan tinggi input TensorRT yang sudah divalidasi.
  int inputH() const { return static_cast<int>(input_dims_.d[2]); }
  // Fungsi: Mengembalikan lebar input TensorRT yang sudah divalidasi.
  int inputW() const { return static_cast<int>(input_dims_.d[3]); }
  // Fungsi: Mengembalikan jumlah elemen buffer input TensorRT.
  size_t inputElements() const { return input_elements_; }

  const OutputTensor &detHead(int index) const { return outputs_.at(static_cast<size_t>(det_indices_.at(index))); }
  bool hasAnchorGrids() const { return anchor_indices_.size() == 3U; }
  const OutputTensor &anchorGrid(int index) const {
    if (!hasAnchorGrids()) {
      throw std::runtime_error("YOLOPv2 anchor-grid outputs are unavailable");
    }
    return outputs_.at(static_cast<size_t>(anchor_indices_.at(static_cast<size_t>(index))));
  }
  const OutputTensor &drivableOutput() const { return outputs_.at(static_cast<size_t>(drivable_index_)); }
  const OutputTensor &laneOutput() const { return outputs_.at(static_cast<size_t>(lane_index_)); }

  // Fungsi: Menyusun ringkasan nama/dimensi tensor untuk verifikasi engine saat startup.
  std::string describe() const {
    std::ostringstream stream;
    stream << "input=" << input_name_ << " [";
    for (int i = 0; i < input_dims_.nbDims; ++i) {
      if (i) stream << ',';
      stream << static_cast<int64_t>(input_dims_.d[i]);
    }
    stream << "] outputs=";
    for (size_t i = 0; i < outputs_.size(); ++i) {
      if (i) stream << "; ";
      stream << outputs_[i].name << "[";
      for (int d = 0; d < outputs_[i].dims.nbDims; ++d) {
        if (d) stream << ',';
        stream << static_cast<int64_t>(outputs_[i].dims.d[d]);
      }
      stream << ']';
    }
    return stream.str();
  }

private:
  // Mengenali kontrak keluaran YOLOPv2 resmi:
  // - tiga raw detection head [1,255,H,W] untuk stride 8/16/32,
  // - tiga anchor-grid [1,3,1,1,2] yang dikembalikan model,
  // - drivable [1,2,384,640],
  // - lane [1,1,384,640].
  // V19 memprioritaskan nama output converter resmi agar head dan anchor tidak
  // pernah tertukar. Engine legacy tetap dapat dibaca memakai urutan I/O-nya.
  void identifyYolopOutputs() {
    det_indices_.clear();
    anchor_indices_.clear();
    drivable_index_ = -1;
    lane_index_ = -1;

    std::array<int, 3> named_det{{-1, -1, -1}};
    std::array<int, 3> named_anchor{{-1, -1, -1}};
    const std::array<std::string, 3> det_names{{"det_stride_8", "det_stride_16", "det_stride_32"}};
    const std::array<std::string, 3> anchor_names{{"anchor_stride_8", "anchor_stride_16", "anchor_stride_32"}};

    for (size_t i = 0; i < outputs_.size(); ++i) {
      const auto &out = outputs_[i];
      const auto &dims = out.dims;
      for (size_t slot = 0; slot < det_names.size(); ++slot) {
        if (out.name == det_names[slot]) named_det[slot] = static_cast<int>(i);
        if (out.name == anchor_names[slot]) named_anchor[slot] = static_cast<int>(i);
      }

      if (dims.nbDims == 5 && static_cast<int64_t>(dims.d[0]) == 1 &&
          static_cast<int64_t>(dims.d[1]) == 3 && static_cast<int64_t>(dims.d[2]) == 1 &&
          static_cast<int64_t>(dims.d[3]) == 1 && static_cast<int64_t>(dims.d[4]) == 2) {
        anchor_indices_.push_back(static_cast<int>(i));
        continue;
      }
      if (dims.nbDims != 4 || static_cast<int64_t>(dims.d[0]) != 1) continue;
      const int c = static_cast<int>(dims.d[1]);
      const int h = static_cast<int>(dims.d[2]);
      const int w = static_cast<int>(dims.d[3]);
      if (c == 255) {
        det_indices_.push_back(static_cast<int>(i));
      } else if (c == 2 && h == inputH() && w == inputW()) {
        drivable_index_ = static_cast<int>(i);
      } else if (c == 1 && h == inputH() && w == inputW()) {
        lane_index_ = static_cast<int>(i);
      }
    }

    const bool all_named_det = std::all_of(named_det.begin(), named_det.end(), [](int i) { return i >= 0; });
    if (all_named_det) {
      det_indices_.assign(named_det.begin(), named_det.end());
    } else {
      std::sort(det_indices_.begin(), det_indices_.end(), [this](int a, int b) {
        const auto &da = outputs_[static_cast<size_t>(a)].dims;
        const auto &db = outputs_[static_cast<size_t>(b)].dims;
        return static_cast<int64_t>(da.d[2]) * static_cast<int64_t>(da.d[3]) >
               static_cast<int64_t>(db.d[2]) * static_cast<int64_t>(db.d[3]);
      });
    }

    const bool all_named_anchor =
      std::all_of(named_anchor.begin(), named_anchor.end(), [](int i) { return i >= 0; });
    if (all_named_anchor) {
      anchor_indices_.assign(named_anchor.begin(), named_anchor.end());
    } else if (anchor_indices_.size() == 3U) {
      // TorchScript resmi mengekspor anchor_grid berurutan stride 8/16/32.
      // Pertahankan urutan I/O engine legacy; jangan menebak ulang berdasarkan label kelas.
      std::sort(anchor_indices_.begin(), anchor_indices_.end());
    }

    if (det_indices_.size() != 3U || anchor_indices_.size() != 3U ||
        drivable_index_ < 0 || lane_index_ < 0) {
      throw std::runtime_error(
        "Engine bukan kontrak resmi YOLOPv2 V19: wajib 3 detection heads + 3 anchor-grid + drivable + lane");
    }
  }

  // Fungsi: Melepaskan output GPU dan object TensorRT sehingga reload/shutdown aman.
  void reset() {
    for (auto &output : outputs_) {
      if (output.device) {
        cudaFree(output.device);
        output.device = nullptr;
      }
    }
    outputs_.clear();
    context_.reset();
    engine_.reset();
    runtime_.reset();
  }

  TrtLogger logger_;
  TrtUniquePtr<nvinfer1::IRuntime> runtime_;
  TrtUniquePtr<nvinfer1::ICudaEngine> engine_;
  TrtUniquePtr<nvinfer1::IExecutionContext> context_;
  std::string input_name_;
  nvinfer1::Dims input_dims_{};
  size_t input_elements_{0};
  std::vector<OutputTensor> outputs_;
  std::vector<int> det_indices_;
  std::vector<int> anchor_indices_;
  int drivable_index_{-1};
  int lane_index_{-1};
};

// Fungsi: Merapikan diagnostic multi-line menjadi satu baris terminal. Kondisi
// hardware tetap dilaporkan apa adanya, tetapi launch log tidak membengkak.
std::string oneLineDiagnostic(std::string text) {
  for (char &c : text) {
    if (c == '\n' || c == '\r' || c == '\t') c = ' ';
  }
  std::size_t pos = 0;
  while ((pos = text.find("  ", pos)) != std::string::npos) {
    text.replace(pos, 2, " ");
  }
  return text;
}

// ============================================================================
// CAPTURE KAMERA V4L2
// ============================================================================
// Struktur frame hanya menunjuk buffer milik V4L2. Buffer wajib dikembalikan
// dengan queue() setelah GPU selesai membaca frame tersebut.
struct V4L2Frame {
  uint32_t index{0};
  size_t bytes_used{0};
  void *host{nullptr};
  const uint8_t *device{nullptr};
};

// Capture kamera mendukung dua mode:
// 1. USERPTR menggunakan pinned mapped memory CUDA sebagai buffer kamera.
// 2. MMAP sebagai fallback apabila driver kamera tidak mendukung USERPTR.
class V4L2Capture final {
public:
  enum class MemoryMode { USERPTR, MMAP };

  struct Buffer {
    void *host{nullptr};
    size_t length{0};
    uint8_t *device{nullptr};
    bool cuda_allocated{false};
    bool cuda_registered{false};
  };

  V4L2Capture() = default;
  // Fungsi: Destructor capture; menghentikan streaming dan melepas buffer/device secara aman.
  ~V4L2Capture() { closeDevice(); }

  V4L2Capture(const V4L2Capture &) = delete;
  V4L2Capture &operator=(const V4L2Capture &) = delete;

  // Membuka perangkat kamera. Nilai "auto" memprioritaskan /dev/v4l/by-id lalu
  // memindai seluruh /dev/videoN yang ada. YUYV diprioritaskan untuk CUDA.
  void openDevice(
    const std::string &requested_device,
    int width,
    int height,
    int fps,
    const std::string &format_preference,
    bool allow_mjpeg,
    bool prefer_userptr,
    int buffer_count,
    bool allow_resolution_fallback_for_fps)
  {
    closeDevice();
    const int requested_width = width;
    const int requested_height = height;
    const int requested_fps = fps;
    width_ = requested_width;
    height_ = requested_height;
    fps_ = requested_fps;

    std::vector<std::string> candidates;
    if (requested_device.empty() || requested_device == "auto") {
      std::error_code error;
      const fs::path by_id("/dev/v4l/by-id");
      if (fs::is_directory(by_id, error)) {
        for (const auto &entry : fs::directory_iterator(by_id, error)) {
          if (error) break;
          const std::string name = entry.path().filename().string();
          if (name.find("index0") != std::string::npos || name.find("video") != std::string::npos) {
            candidates.push_back(entry.path().string());
          }
        }
      }
      std::sort(candidates.begin(), candidates.end());
      for (int i = 0; i < 64; ++i) {
        const std::string device = "/dev/video" + std::to_string(i);
        if (::access(device.c_str(), F_OK) == 0) candidates.push_back(device);
      }
      std::vector<std::string> unique;
      for (const auto &candidate : candidates) {
        if (std::find(unique.begin(), unique.end(), candidate) == unique.end()) unique.push_back(candidate);
      }
      candidates = std::move(unique);
    } else {
      candidates.push_back(requested_device);
    }

    std::vector<uint32_t> formats;
    const std::string pref = upper(format_preference);
    if (pref == "MJPG" || pref == "MJPEG") {
      if (allow_mjpeg) formats.push_back(V4L2_PIX_FMT_MJPEG);
      formats.push_back(V4L2_PIX_FMT_YUYV);
    } else {
      formats.push_back(V4L2_PIX_FMT_YUYV);
      if (allow_mjpeg) formats.push_back(V4L2_PIX_FMT_MJPEG);
    }

    std::string errors;
    bool found_existing_device = false;
    for (const auto &candidate : candidates) {
      if (::access(candidate.c_str(), F_OK) != 0) {
        continue;
      }
      found_existing_device = true;
      for (const uint32_t pixel_format : formats) {
        try {
          width_ = requested_width;
          height_ = requested_height;
          fps_ = requested_fps;
          mode_adjusted_for_fps_ = false;
          selected_mode_max_fps_ = 0.0;
          tryOpenOne(
            candidate, pixel_format, prefer_userptr, buffer_count,
            allow_resolution_fallback_for_fps);

          // Untuk fidelity input, prioritaskan YUYV lossless bila benar-benar mampu
          // 1280x720@30. Jika driver menerima YUYV tetapi diam-diam menurunkan
          // FPS, coba MJPEG berikutnya agar geometri + target frame rate resmi
          // YOLOPv2 tetap dipertahankan. Jangan turun resolusi secara implisit.
          if (pixel_format == V4L2_PIX_FMT_YUYV && allow_mjpeg &&
              std::abs(actual_fps_ - static_cast<double>(requested_fps)) > 0.5) {
            errors += candidate + " YUYV: actual FPS " + std::to_string(actual_fps_) +
                      " tidak memenuhi requested " + std::to_string(requested_fps) +
                      "; mencoba MJPEG 1280x720 pada FPS yang sama\n";
            closeDevice();
            continue;
          }
          return;
        } catch (const std::exception &e) {
          errors += candidate + " " + fourccToString(pixel_format) + ": " + e.what() + "\n";
          closeDevice();
        }
      }
    }
    if (!found_existing_device) {
      throw std::runtime_error("No V4L2 video device present (/dev/v4l/by-id or /dev/videoN)");
    }
    throw std::runtime_error("No usable V4L2 RGB camera found. Attempts:\n" + errors);
  }

  // Mengambil satu frame yang sudah selesai diisi kamera. Fungsi tidak melakukan
  // penyalinan gambar; hasilnya hanya referensi ke buffer V4L2.
  bool dequeue(V4L2Frame &frame, int timeout_ms) {
    if (fd_ < 0 || !streaming_) return false;
    pollfd pfd{};
    pfd.fd = fd_;
    pfd.events = POLLIN;
    const int poll_result = ::poll(&pfd, 1, timeout_ms);
    if (poll_result == 0) return false;
    if (poll_result < 0) {
      if (errno == EINTR) return false;
      throw std::runtime_error("poll(V4L2) failed: " + std::string(std::strerror(errno)));
    }

    v4l2_buffer buffer{};
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = memory_mode_ == MemoryMode::USERPTR ? V4L2_MEMORY_USERPTR : V4L2_MEMORY_MMAP;
    if (xioctl(fd_, VIDIOC_DQBUF, &buffer) < 0) {
      if (errno == EAGAIN) return false;
      throw std::runtime_error("VIDIOC_DQBUF failed: " + std::string(std::strerror(errno)));
    }
    uint32_t resolved_index = buffer.index;
    if (memory_mode_ == MemoryMode::USERPTR) {
      const void *returned_ptr = reinterpret_cast<const void *>(buffer.m.userptr);
      if (resolved_index >= buffers_.size() || buffers_[resolved_index].host != returned_ptr) {
        const auto found = std::find_if(buffers_.begin(), buffers_.end(),
          [returned_ptr](const Buffer &candidate) { return candidate.host == returned_ptr; });
        if (found == buffers_.end()) {
          throw std::runtime_error("V4L2 returned an unknown USERPTR buffer");
        }
        resolved_index = static_cast<uint32_t>(std::distance(buffers_.begin(), found));
      }
    }
    if (resolved_index >= buffers_.size()) {
      throw std::runtime_error("V4L2 returned an invalid buffer index");
    }

    frame.index = resolved_index;
    frame.bytes_used = buffer.bytesused;
    frame.host = buffers_[resolved_index].host;
    frame.device = buffers_[resolved_index].device;
    return true;
  }

  // Mengembalikan buffer ke driver agar dapat dipakai lagi oleh kamera.
  void queue(uint32_t index) {
    if (fd_ < 0 || index >= buffers_.size()) return;
    v4l2_buffer buffer{};
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.index = index;
    if (memory_mode_ == MemoryMode::USERPTR) {
      buffer.memory = V4L2_MEMORY_USERPTR;
      buffer.m.userptr = reinterpret_cast<unsigned long>(buffers_[index].host);
      buffer.length = static_cast<uint32_t>(buffers_[index].length);
    } else {
      buffer.memory = V4L2_MEMORY_MMAP;
    }
    if (xioctl(fd_, VIDIOC_QBUF, &buffer) < 0) {
      throw std::runtime_error("VIDIOC_QBUF failed: " + std::string(std::strerror(errno)));
    }
  }

  // Fungsi: Menghentikan VIDIOC_STREAMOFF hanya bila stream sedang aktif.
  void stopStreaming() {
    if (fd_ >= 0 && streaming_) {
      v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      xioctl(fd_, VIDIOC_STREAMOFF, &type);
      streaming_ = false;
    }
  }

  uint32_t pixelFormat() const { return pixel_format_; }
  int width() const { return width_; }
  int height() const { return height_; }
  int fps() const { return fps_; }
  double actualFps() const { return actual_fps_; }
  bool modeAdjustedForFps() const { return mode_adjusted_for_fps_; }
  double selectedModeMaxFps() const { return selected_mode_max_fps_; }
  size_t sizeImage() const { return size_image_; }
  size_t bytesPerLine() const { return bytes_per_line_; }
  MemoryMode memoryMode() const { return memory_mode_; }
  const std::string &devicePath() const { return device_path_; }
  // Fungsi: Memeriksa apakah semua buffer kamera memiliki alamat device CUDA tanpa staging copy.
  bool directCudaMapped() const {
    return !buffers_.empty() && std::all_of(buffers_.begin(), buffers_.end(), [](const Buffer &b) { return b.device != nullptr; });
  }

private:
  // Fungsi: Menormalkan nama format pixel menjadi huruf kapital sebelum dibandingkan.
  static std::string upper(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return value;
  }

  struct CameraMode {
    int width{0};
    int height{0};
    double max_fps{0.0};
    bool valid{false};
  };

  // Fungsi: Mengubah interval frame V4L2 menjadi FPS dan menangani denominator/numerator nol.
  static double frameIntervalToFps(const v4l2_fract &interval) {
    if (interval.numerator == 0U || interval.denominator == 0U) return 0.0;
    return static_cast<double>(interval.denominator) /
           static_cast<double>(interval.numerator);
  }

  // Enumerasi mode dilakukan melalui VIDIOC_ENUM_FRAMESIZES dan
  // VIDIOC_ENUM_FRAMEINTERVALS. Bila resolusi yang diminta tidak mampu mencapai
  // FPS target, pilih resolusi terbesar dengan aspect ratio paling dekat yang
  // benar-benar mengiklankan FPS target. Ini lebih benar daripada meminta 30 FPS
  // pada mode 1280x720 yang oleh driver diam-diam diturunkan menjadi 10 FPS.
  CameraMode chooseModeForRequestedFps(
    uint32_t pixel_format,
    int requested_width,
    int requested_height,
    int requested_fps) const
  {
    std::vector<CameraMode> modes;
    for (uint32_t size_index = 0;; ++size_index) {
      v4l2_frmsizeenum size_enum{};
      size_enum.index = size_index;
      size_enum.pixel_format = pixel_format;
      if (xioctl(fd_, VIDIOC_ENUM_FRAMESIZES, &size_enum) < 0) break;

      int mode_width = requested_width;
      int mode_height = requested_height;
      if (size_enum.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
        mode_width = static_cast<int>(size_enum.discrete.width);
        mode_height = static_cast<int>(size_enum.discrete.height);
      } else if (
        size_enum.type == V4L2_FRMSIZE_TYPE_STEPWISE ||
        size_enum.type == V4L2_FRMSIZE_TYPE_CONTINUOUS)
      {
        const auto &step = size_enum.stepwise;
        mode_width = std::clamp(
          requested_width,
          static_cast<int>(step.min_width),
          static_cast<int>(step.max_width));
        mode_height = std::clamp(
          requested_height,
          static_cast<int>(step.min_height),
          static_cast<int>(step.max_height));
      } else {
        continue;
      }

      double max_fps = 0.0;
      for (uint32_t interval_index = 0;; ++interval_index) {
        v4l2_frmivalenum interval{};
        interval.index = interval_index;
        interval.pixel_format = pixel_format;
        interval.width = static_cast<uint32_t>(mode_width);
        interval.height = static_cast<uint32_t>(mode_height);
        if (xioctl(fd_, VIDIOC_ENUM_FRAMEINTERVALS, &interval) < 0) break;
        if (interval.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
          max_fps = std::max(max_fps, frameIntervalToFps(interval.discrete));
        } else if (
          interval.type == V4L2_FRMIVAL_TYPE_STEPWISE ||
          interval.type == V4L2_FRMIVAL_TYPE_CONTINUOUS)
        {
          // stepwise.min adalah interval waktu terkecil, sehingga FPS terbesar.
          max_fps = std::max(max_fps, frameIntervalToFps(interval.stepwise.min));
          break;
        }
      }
      if (max_fps > 0.0) {
        modes.push_back(CameraMode{mode_width, mode_height, max_fps, true});
      }
      if (size_enum.type != V4L2_FRMSIZE_TYPE_DISCRETE) break;
    }

    const auto supports_target = [requested_fps](const CameraMode &mode) {
      return mode.valid && mode.max_fps + 0.25 >= static_cast<double>(requested_fps);
    };

    for (const auto &mode : modes) {
      if (mode.width == requested_width && mode.height == requested_height &&
          supports_target(mode)) {
        return mode;
      }
    }

    CameraMode best;
    double best_score = std::numeric_limits<double>::infinity();
    const double requested_aspect = static_cast<double>(requested_width) /
                                    static_cast<double>(requested_height);
    const double requested_area = static_cast<double>(requested_width) * requested_height;
    for (const auto &mode : modes) {
      if (!supports_target(mode)) continue;
      const double area = static_cast<double>(mode.width) * mode.height;
      const double aspect = static_cast<double>(mode.width) / mode.height;
      const double aspect_error = std::abs(aspect - requested_aspect);
      const bool fits_requested = mode.width <= requested_width && mode.height <= requested_height;
      const bool same_aspect = aspect_error <= 0.03;

      // Prioritas: tidak melebihi resolusi target, aspect ratio sama, lalu area
      // terbesar. Penalti dibuat berjenjang agar 640x360 dipilih sebelum 640x480
      // untuk target 16:9, bila keduanya mendukung 30 FPS.
      double score = 0.0;
      if (!fits_requested) score += 1.0e15;
      if (!same_aspect) score += 1.0e14;
      score += aspect_error * 1.0e12;
      score += fits_requested ? (requested_area - area) : std::abs(area - requested_area);
      if (score < best_score) {
        best_score = score;
        best = mode;
      }
    }
    return best;
  }

  // Mencoba satu kombinasi perangkat dan format pixel. Jika USERPTR gagal,
  // kode otomatis beralih ke MMAP tanpa menghentikan keseluruhan pencarian.
  void tryOpenOne(
    const std::string &path,
    uint32_t requested_format,
    bool prefer_userptr,
    int buffer_count,
    bool allow_resolution_fallback_for_fps)
  {
    fd_ = ::open(path.c_str(), O_RDWR | O_NONBLOCK, 0);
    if (fd_ < 0) {
      throw std::runtime_error(std::strerror(errno));
    }

    v4l2_capability capability{};
    if (xioctl(fd_, VIDIOC_QUERYCAP, &capability) < 0) {
      throw std::runtime_error("VIDIOC_QUERYCAP failed");
    }
    if (!(capability.capabilities & V4L2_CAP_VIDEO_CAPTURE) || !(capability.capabilities & V4L2_CAP_STREAMING)) {
      throw std::runtime_error("device does not support streaming video capture");
    }

    if (allow_resolution_fallback_for_fps) {
      const CameraMode selected = chooseModeForRequestedFps(
        requested_format, width_, height_, fps_);
      if (selected.valid) {
        selected_mode_max_fps_ = selected.max_fps;
        if (selected.width != width_ || selected.height != height_) {
          width_ = selected.width;
          height_ = selected.height;
          mode_adjusted_for_fps_ = true;
        }
      }
    }

    v4l2_format format{};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = static_cast<uint32_t>(width_);
    format.fmt.pix.height = static_cast<uint32_t>(height_);
    format.fmt.pix.pixelformat = requested_format;
    format.fmt.pix.field = V4L2_FIELD_ANY;
    if (xioctl(fd_, VIDIOC_S_FMT, &format) < 0) {
      throw std::runtime_error("VIDIOC_S_FMT failed: " + std::string(std::strerror(errno)));
    }
    if (format.fmt.pix.pixelformat != requested_format) {
      throw std::runtime_error("driver substituted pixel format " + fourccToString(format.fmt.pix.pixelformat));
    }
    if (static_cast<int>(format.fmt.pix.width) != width_ || static_cast<int>(format.fmt.pix.height) != height_) {
      throw std::runtime_error("driver substituted resolution " + std::to_string(format.fmt.pix.width) + "x" + std::to_string(format.fmt.pix.height));
    }
    pixel_format_ = requested_format;
    bytes_per_line_ = static_cast<size_t>(format.fmt.pix.bytesperline);
    if (pixel_format_ == V4L2_PIX_FMT_YUYV) {
      const size_t minimum_stride = static_cast<size_t>(width_) * 2U;
      if (bytes_per_line_ == 0U) bytes_per_line_ = minimum_stride;
      if (bytes_per_line_ < minimum_stride) {
        throw std::runtime_error(
          "invalid V4L2 YUYV bytesperline=" + std::to_string(bytes_per_line_) +
          " for width=" + std::to_string(width_));
      }
    } else {
      bytes_per_line_ = 0U;
    }
    size_image_ = static_cast<size_t>(format.fmt.pix.sizeimage);
    const size_t minimum_capture_bytes = pixel_format_ == V4L2_PIX_FMT_YUYV
      ? bytes_per_line_ * static_cast<size_t>(height_)
      : static_cast<size_t>(width_) * static_cast<size_t>(height_) * 3U;
    if (size_image_ == 0U) size_image_ = minimum_capture_bytes;
    if (pixel_format_ == V4L2_PIX_FMT_YUYV && size_image_ < minimum_capture_bytes) {
      size_image_ = minimum_capture_bytes;
    }

    v4l2_streamparm streamparm{};
    streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    streamparm.parm.capture.timeperframe.numerator = 1;
    streamparm.parm.capture.timeperframe.denominator = static_cast<uint32_t>(fps_);

    // VIDIOC_S_PARM mengembalikan interval aktual yang dipilih driver. Kamera
    // UVC dapat menurunkan FPS apabila kombinasi resolusi dan
    // format tersebut tidak tersedia. Simpan nilai aktual agar log tidak
    // menyesatkan.
    if (xioctl(fd_, VIDIOC_S_PARM, &streamparm) < 0) {
      std::memset(&streamparm, 0, sizeof(streamparm));
      streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      (void)xioctl(fd_, VIDIOC_G_PARM, &streamparm);
    }
    const auto numerator = streamparm.parm.capture.timeperframe.numerator;
    const auto denominator = streamparm.parm.capture.timeperframe.denominator;
    actual_fps_ = (numerator > 0U && denominator > 0U)
      ? static_cast<double>(denominator) / static_cast<double>(numerator)
      : static_cast<double>(fps_);

    bool started = false;
    if (prefer_userptr) {
      try {
        setupUserPtr(buffer_count);
        queueAllAndStart();
        started = true;
      } catch (...) {
        stopStreaming();
        releaseBuffers();
        releaseKernelBuffers(V4L2_MEMORY_USERPTR);
      }
    }
    if (!started) {
      setupMmap(buffer_count);
      queueAllAndStart();
    }
    device_path_ = path;
  }


  // Fungsi: Mengantre seluruh buffer V4L2 lalu memulai streaming kamera.
  void queueAllAndStart() {
    for (uint32_t i = 0; i < buffers_.size(); ++i) {
      queue(i);
    }
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
      throw std::runtime_error("VIDIOC_STREAMON failed: " + std::string(std::strerror(errno)));
    }
    streaming_ = true;
  }

  // Fungsi: Meminta kernel melepaskan buffer V4L2 untuk mode memory yang sedang digunakan.
  void releaseKernelBuffers(v4l2_memory memory) {
    if (fd_ < 0) return;
    v4l2_requestbuffers request{};
    request.count = 0;
    request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request.memory = memory;
    xioctl(fd_, VIDIOC_REQBUFS, &request);
  }

  // USERPTR adalah jalur paling ringan: kamera menulis ke pinned host memory yang
  // sekaligus memiliki alamat device CUDA, sehingga tidak perlu memcpy H2D biasa.
  void setupUserPtr(int requested_count) {
    v4l2_requestbuffers request{};
    request.count = static_cast<uint32_t>(std::max(2, requested_count));
    request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request.memory = V4L2_MEMORY_USERPTR;
    if (xioctl(fd_, VIDIOC_REQBUFS, &request) < 0 || request.count < 2) {
      throw std::runtime_error("V4L2 USERPTR buffers are not supported");
    }

    memory_mode_ = MemoryMode::USERPTR;
    buffers_.resize(request.count);
    for (auto &buffer : buffers_) {
      void *host = nullptr;
      const unsigned int flags = cudaHostAllocMapped | cudaHostAllocPortable | cudaHostAllocWriteCombined;
      CUDA_CHECK(cudaHostAlloc(&host, size_image_, flags));
      void *device = nullptr;
      CUDA_CHECK(cudaHostGetDevicePointer(&device, host, 0));
      buffer.host = host;
      buffer.length = size_image_;
      buffer.device = static_cast<uint8_t *>(device);
      buffer.cuda_allocated = true;
    }
  }

  // MMAP adalah fallback kompatibel. Buffer MMAP dicoba diregistrasikan ke CUDA;
  // jika registrasi gagal, frame akan disalin ke staging buffer di VRAM.
  void setupMmap(int requested_count) {
    v4l2_requestbuffers request{};
    request.count = static_cast<uint32_t>(std::max(2, requested_count));
    request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd_, VIDIOC_REQBUFS, &request) < 0 || request.count < 2) {
      throw std::runtime_error("V4L2 MMAP buffers are not supported");
    }

    memory_mode_ = MemoryMode::MMAP;
    buffers_.resize(request.count);
    for (uint32_t i = 0; i < request.count; ++i) {
      v4l2_buffer query{};
      query.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      query.memory = V4L2_MEMORY_MMAP;
      query.index = i;
      if (xioctl(fd_, VIDIOC_QUERYBUF, &query) < 0) {
        throw std::runtime_error("VIDIOC_QUERYBUF failed");
      }
      void *host = ::mmap(nullptr, query.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, query.m.offset);
      if (host == MAP_FAILED) {
        throw std::runtime_error("mmap(V4L2) failed");
      }
      buffers_[i].host = host;
      buffers_[i].length = query.length;

      const cudaError_t register_result = cudaHostRegister(host, query.length, cudaHostRegisterMapped | cudaHostRegisterPortable);
      if (register_result == cudaSuccess) {
        void *device = nullptr;
        if (cudaHostGetDevicePointer(&device, host, 0) == cudaSuccess) {
          buffers_[i].device = static_cast<uint8_t *>(device);
          buffers_[i].cuda_registered = true;
        } else {
          cudaHostUnregister(host);
        }
      } else {
        cudaGetLastError();
      }
    }
  }

  // Membebaskan seluruh VRAM dan pinned host memory secara aman.
  // Fungsi: Melepas seluruh buffer CUDA/pinned host milik node pada shutdown atau kegagalan startup.
  void releaseBuffers() {
    for (auto &buffer : buffers_) {
      if (buffer.cuda_allocated && buffer.host) {
        cudaFreeHost(buffer.host);
      } else {
        if (buffer.cuda_registered && buffer.host) {
          cudaHostUnregister(buffer.host);
        }
        if (buffer.host && buffer.host != MAP_FAILED && buffer.length) {
          ::munmap(buffer.host, buffer.length);
        }
      }
      buffer = Buffer{};
    }
    buffers_.clear();
  }

  // Fungsi: Menutup device V4L2 dan mereset metadata mode agar reconnect berikutnya bersih.
  void closeDevice() {
    stopStreaming();
    releaseBuffers();
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
    device_path_.clear();
    actual_fps_ = 0.0;
    selected_mode_max_fps_ = 0.0;
    bytes_per_line_ = 0U;
    mode_adjusted_for_fps_ = false;
  }

  int fd_{-1};
  int width_{0};
  int height_{0};
  int fps_{0};
  double actual_fps_{0.0};
  double selected_mode_max_fps_{0.0};
  bool mode_adjusted_for_fps_{false};
  uint32_t pixel_format_{0};
  size_t size_image_{0};
  size_t bytes_per_line_{0};
  MemoryMode memory_mode_{MemoryMode::MMAP};
  std::vector<Buffer> buffers_;
  bool streaming_{false};
  std::string device_path_;
};

// ============================================================================
// STRUKTUR DATA DAN KERNEL CUDA
// ============================================================================
// Bounding box disimpan langsung di VRAM selama decoding, NMS, scaling, dan
// penggambaran. Data baru disalin ke RAM bila topic detections/label CPU aktif.
enum DetectionClipFlags : int {
  DETECTION_CLIP_LEFT = 1 << 0,
  DETECTION_CLIP_TOP = 1 << 1,
  DETECTION_CLIP_RIGHT = 1 << 2,
  DETECTION_CLIP_BOTTOM = 1 << 3,
};

struct DetectionGpu {
  float x1;
  float y1;
  float x2;
  float y2;
  float confidence;
  int class_id;
  int keep;
  // Menyimpan informasi bbox sebelum clamp ke batas frame. Bit BOTTOM dipakai
  // near-field fail-safe saat object terlalu dekat sampai terpotong frame.
  int clip_flags;
};

// Data metrik siap pakai untuk node bringup. Semua proyeksi pixel -> ground plane
// dilakukan di GPU; bringup hanya melakukan tracking, safety gating, dan publish
// PointCloud/Marker tanpa OpenCV, NumPy, atau akses frame kamera.
struct MetricDetectionGpu {
  float forward_m;
  float left_m;
  float width_m;
  float confidence;
  float center_x_px;
  float center_y_px;
  float bottom_y_px;
  float width_px;
  float height_px;
  float area_px;
  int class_id;
  int valid;
};

// Fungsi GPU: Menentukan class YOLO yang diperlakukan sebagai obstacle kendaraan/orang.
__device__ __forceinline__ bool obstacleClassGpu(int class_id) {
  return class_id == 0 || class_id == 1 || class_id == 2 || class_id == 3 ||
         class_id == 5 || class_id == 6 || class_id == 7 || class_id == 8;
}

__device__ __forceinline__ bool projectPixelToGroundGpu(
  float x, float y, const float *h, float origin_x, float origin_y,
  float scale_x, float scale_y, int canvas_width, int canvas_height,
  float &forward, float &left)
{
  const float w = h[6] * x + h[7] * y + h[8];
  if (!isfinite(w) || fabsf(w) < 1.0e-6F) return false;
  const float bx = (h[0] * x + h[1] * y + h[2]) / w;
  const float by = (h[3] * x + h[4] * y + h[5]) / w;
  if (!isfinite(bx) || !isfinite(by)) return false;
  // Same validity boundary as the previous cv2.perspectiveTransform/
  // warpPerspective path: a point outside the calibrated BEV canvas is not a
  // valid metric ground observation.
  if (bx < 0.0F || by < 0.0F || bx >= static_cast<float>(canvas_width) ||
      by >= static_cast<float>(canvas_height)) return false;
  forward = (origin_y - by) * scale_y;
  left = (origin_x - bx) * scale_x;
  return isfinite(forward) && isfinite(left);
}

__global__ void projectDetectionsMetricKernel(
  const DetectionGpu *__restrict__ detections,
  const int *__restrict__ count,
  int max_detections,
  const uint8_t *__restrict__ drivable_mask,
  int image_width, int image_height,
  int require_drivable_contact,
  int drivable_contact_radius_px,
  int drivable_contact_vertical_tolerance_px,
  int drivable_contact_min_samples,
  float drivable_contact_min_fraction,
  const float *__restrict__ homography,
  float origin_x, float origin_y, float scale_x, float scale_y,
  int canvas_width, int canvas_height,
  float forward_offset, float lateral_offset,
  float min_forward, float max_forward, float max_abs_left,
  MetricDetectionGpu *__restrict__ metric)
{
  const int i = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  if (i >= max_detections) return;
  metric[i] = MetricDetectionGpu{
    0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    -1, 0};
  const int valid_count = min(*count, max_detections);
  if (i >= valid_count) return;
  const DetectionGpu d = detections[i];
  if (d.class_id < 0 || d.confidence <= 0.0F) return;

  const float cx = 0.5F * (d.x1 + d.x2);
  const float bottom = d.y2;

  // Kandidat obstacle normal wajib mempunyai foot-region yang benar-benar
  // bersinggungan dengan drivable mask. Jangan hanya mencicipi center bbox:
  // pada tikungan/parked-car, bbox besar dapat menutupi jalan di citra walaupun
  // footprint fisiknya berada di luar jalur. Sampling 5x3 di strip kaki bbox
  // membuat keputusan lebih representatif namun tetap sangat ringan di CUDA.
  if (require_drivable_contact != 0) {
    if (!drivable_mask || image_width <= 0 || image_height <= 0) return;
    const int bottom_y = max(0, min(image_height - 1, static_cast<int>(lrintf(bottom))));
    const int tolerance = max(0, drivable_contact_vertical_tolerance_px);
    const float inset = static_cast<float>(max(0, drivable_contact_radius_px));
    const float x_min = fminf(d.x2, d.x1 + inset);
    const float x_max = fmaxf(d.x1, d.x2 - inset);
    // Contact band melintasi tepi bawah bbox dengan toleransi kecil 2 px ke
    // bawah. Ini penting karena pixel badan obstacle biasanya bukan drivable;
    // yang kita cari adalah road support tepat di kaki object, bukan road yang
    // jauh di bawah bbox.
    const int sample_y[3] = {
      max(0, bottom_y - max(2, tolerance)),
      max(0, bottom_y - 2),
      min(image_height - 1, bottom_y + 2)};
    int road_samples = 0;
    int total_samples = 0;
    for (int yi = 0; yi < 3; ++yi) {
      for (int xi = 0; xi < 5; ++xi) {
        const float alpha = static_cast<float>(xi) * 0.25F;
        const int sx = max(0, min(
          image_width - 1, static_cast<int>(lrintf(x_min + alpha * (x_max - x_min)))));
        ++total_samples;
        road_samples += drivable_mask[sample_y[yi] * image_width + sx] > 0U ? 1 : 0;
      }
    }
    const float road_fraction = total_samples > 0 ?
      static_cast<float>(road_samples) / static_cast<float>(total_samples) : 0.0F;
    if (road_samples < max(1, drivable_contact_min_samples) ||
        road_fraction < drivable_contact_min_fraction) return;
  }

  float forward = 0.0F, left = 0.0F;
  if (!projectPixelToGroundGpu(
      cx, bottom, homography, origin_x, origin_y, scale_x, scale_y,
      canvas_width, canvas_height, forward, left)) return;
  forward += forward_offset;
  left += lateral_offset;
  if (forward < min_forward || forward > max_forward || fabsf(left) > max_abs_left) return;

  float lf = 0.0F, ll = 0.0F, rf = 0.0F, rl = 0.0F;
  float width = 0.35F;
  if (projectPixelToGroundGpu(
        d.x1, bottom, homography, origin_x, origin_y, scale_x, scale_y,
        canvas_width, canvas_height, lf, ll) &&
      projectPixelToGroundGpu(
        d.x2, bottom, homography, origin_x, origin_y, scale_x, scale_y,
        canvas_width, canvas_height, rf, rl)) {
    width = fabsf(ll - rl);
  }
  width = fminf(2.5F, fmaxf(0.20F, width));
  const float box_width = fmaxf(0.0F, d.x2 - d.x1);
  const float box_height = fmaxf(0.0F, d.y2 - d.y1);
  metric[i] = MetricDetectionGpu{
    forward, left, width, d.confidence,
    cx, 0.5F * (d.y1 + d.y2), bottom,
    box_width, box_height, box_width * box_height,
    d.class_id, 1};
}

// Mengubah lane mask yang sudah berada di VRAM menjadi pasangan batas kiri/kanan
// untuk sejumlah lookahead bins. Tidak ada warpPerspective/morphology di Python.
// Pixel lane diproyeksikan langsung dengan homography dan atomik memilih batas
// terdekat terhadap center robot pada setiap bin.
__global__ void laneMetricBinsKernel(
  const uint8_t *__restrict__ lane_mask, int width, int height,
  const float *__restrict__ homography,
  float origin_x, float origin_y, float scale_x, float scale_y,
  int canvas_width, int canvas_height,
  float near_m, float far_m, float min_center_offset_m, int sample_rows,
  int *__restrict__ left_mm, int *__restrict__ right_mm)
{
  const int x = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  const int y = static_cast<int>(blockIdx.y * blockDim.y + threadIdx.y);
  if (x >= width || y >= height) return;
  const size_t pixel = static_cast<size_t>(y) * width + x;
  if (lane_mask[pixel] == 0U) return;

  float forward = 0.0F, left = 0.0F;
  if (!projectPixelToGroundGpu(
      static_cast<float>(x), static_cast<float>(y), homography,
      origin_x, origin_y, scale_x, scale_y, canvas_width, canvas_height,
      forward, left)) return;
  if (forward < near_m || forward > far_m || fabsf(left) < min_center_offset_m) return;

  const float t = (forward - near_m) / fmaxf(1.0e-6F, far_m - near_m);
  const int row = max(0, min(sample_rows - 1, static_cast<int>(lrintf(t * (sample_rows - 1)))));
  const int millimeters = max(0, static_cast<int>(lrintf(fabsf(left) * 1000.0F)));
  // Fill the neighbouring metric rows as a tiny GPU-domain continuity filter.
  // It replaces the old CPU morphology/vertical-search path without allocating
  // another full-resolution mask.
  for (int delta = -1; delta <= 1; ++delta) {
    const int target = row + delta;
    if (target < 0 || target >= sample_rows) continue;
    if (left > 0.0F) atomicMin(&left_mm[target], millimeters);
    else atomicMin(&right_mm[target], millimeters);
  }
}


// Mengubah drivable mask menjadi envelope ruang bebas metrik per lookahead bin.
// Berbeda dengan lane mask, boundary ini berasal langsung dari output drivable
// YOLOPv2 sehingga MPPI mengetahui sisi kiri/kanan yang benar-benar boleh dipakai.
__global__ void drivableMetricBinsKernel(
  const uint8_t *__restrict__ drivable_mask, int width, int height,
  const float *__restrict__ homography,
  float origin_x, float origin_y, float scale_x, float scale_y,
  int canvas_width, int canvas_height,
  float near_m, float far_m, float max_abs_left_m, int sample_rows, int pixel_stride,
  int *__restrict__ left_extent_mm, int *__restrict__ right_extent_mm,
  int *__restrict__ sample_count)
{
  const int sample_x = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  const int sample_y = static_cast<int>(blockIdx.y * blockDim.y + threadIdx.y);
  const int x = sample_x * pixel_stride;
  const int y = sample_y * pixel_stride;
  if (x >= width || y >= height) return;
  const size_t pixel = static_cast<size_t>(y) * width + x;
  if (drivable_mask[pixel] == 0U) return;

  float forward = 0.0F, left = 0.0F;
  if (!projectPixelToGroundGpu(
      static_cast<float>(x), static_cast<float>(y), homography,
      origin_x, origin_y, scale_x, scale_y, canvas_width, canvas_height,
      forward, left)) return;
  if (forward < near_m || forward > far_m || fabsf(left) > max_abs_left_m) return;

  const float t = (forward - near_m) / fmaxf(1.0e-6F, far_m - near_m);
  const int row = max(0, min(sample_rows - 1, static_cast<int>(lrintf(t * (sample_rows - 1)))));
  const int millimeters = max(0, static_cast<int>(lrintf(fabsf(left) * 1000.0F)));
  if (left >= 0.0F) atomicMax(&left_extent_mm[row], millimeters);
  else atomicMax(&right_extent_mm[row], millimeters);
  atomicAdd(&sample_count[row], 1);
}

// Fungsi GPU: Menjepit nilai float hasil konversi warna ke rentang byte 0..255.
__device__ __forceinline__ uint8_t clampU8(float value) {
  value = fminf(255.0F, fmaxf(0.0F, value));
  return static_cast<uint8_t>(value);
}

// Fungsi GPU: Mengubah satu pixel YUV menjadi BGR langsung di device memory.
__device__ __forceinline__ void yuvToBgr(uint8_t y, uint8_t u, uint8_t v, uint8_t &b, uint8_t &g, uint8_t &r) {
  const float yf = fmaxf(0.0F, static_cast<float>(y) - 16.0F) * 1.164383F;
  const float uf = static_cast<float>(u) - 128.0F;
  const float vf = static_cast<float>(v) - 128.0F;
  r = clampU8(yf + 1.596027F * vf);
  g = clampU8(yf - 0.391762F * uf - 0.812968F * vf);
  b = clampU8(yf + 2.017232F * uf);
}

// Konversi YUYV 4:2:2 menjadi BGR8 sekaligus melakukan horizontal flip.
__global__ void yuyvToBgrKernel(
  const uint8_t *__restrict__ yuyv,
  uint8_t *__restrict__ bgr,
  int width,
  int height,
  size_t input_stride_bytes,
  bool flip_horizontal)
{
  const int x = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  const int y = static_cast<int>(blockIdx.y * blockDim.y + threadIdx.y);
  if (x >= width || y >= height) return;

  const int source_x = flip_horizontal ? (width - 1 - x) : x;
  const int pair_x = source_x & ~1;
  const size_t input_offset = static_cast<size_t>(y) * input_stride_bytes +
                              static_cast<size_t>(pair_x) * 2U;
  const uint8_t yy = yuyv[input_offset + ((source_x & 1) ? 2U : 0U)];
  const uint8_t u = yuyv[input_offset + 1U];
  const uint8_t v = yuyv[input_offset + 3U];
  uint8_t b, g, r;
  yuvToBgr(yy, u, v, b, g, r);
  const size_t output_offset = (static_cast<size_t>(y) * width + x) * 3U;
  bgr[output_offset + 0U] = b;
  bgr[output_offset + 1U] = g;
  bgr[output_offset + 2U] = r;
}

// Menyalin BGR ke buffer kerja GPU dan melakukan flip bila diminta.
__global__ void copyOrFlipBgrKernel(
  const uint8_t *__restrict__ input,
  uint8_t *__restrict__ output,
  int width,
  int height,
  bool flip_horizontal)
{
  const int x = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  const int y = static_cast<int>(blockIdx.y * blockDim.y + threadIdx.y);
  if (x >= width || y >= height) return;
  const int source_x = flip_horizontal ? (width - 1 - x) : x;
  const size_t source = (static_cast<size_t>(y) * width + source_x) * 3U;
  const size_t target = (static_cast<size_t>(y) * width + x) * 3U;
  output[target + 0U] = input[source + 0U];
  output[target + 1U] = input[source + 1U];
  output[target + 2U] = input[source + 2U];
}

// Statistik citra yang sangat ringan untuk fail-safe visibilitas kamera. Hanya
// beberapa ribu sample/frame yang diakumulasi di GPU; host menerima array statistik kecil.
enum CameraHealthStatIndex : int {
  CAMERA_HEALTH_COUNT = 0,
  CAMERA_HEALTH_SUM = 1,
  CAMERA_HEALTH_SUM_SQ = 2,
  CAMERA_HEALTH_DARK = 3,
  CAMERA_HEALTH_BRIGHT = 4,
  CAMERA_HEALTH_GRADIENT = 5,
  NEAR_FIELD_SAMPLE_COUNT = 6,
  NEAR_FIELD_DRIVABLE_COUNT = 7,
  CAMERA_HEALTH_STAT_COUNT = 8,
};

__global__ void cameraHealthStatsKernel(
  const uint8_t *__restrict__ bgr,
  int width,
  int height,
  int sample_stride,
  int dark_threshold,
  int bright_threshold,
  unsigned long long *__restrict__ stats)
{
  const int sx = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  const int sy = static_cast<int>(blockIdx.y * blockDim.y + threadIdx.y);
  const int x = sx * sample_stride;
  const int y = sy * sample_stride;
  if (x >= width || y >= height) return;

  const size_t offset = (static_cast<size_t>(y) * width + x) * 3U;
  const unsigned int b = bgr[offset + 0U];
  const unsigned int g = bgr[offset + 1U];
  const unsigned int r = bgr[offset + 2U];
  const unsigned int luma = (b + 2U * g + r) >> 2U;

  unsigned int gradient = 0U;
  const int nx = min(x + sample_stride, width - 1);
  if (nx != x) {
    const size_t no = (static_cast<size_t>(y) * width + nx) * 3U;
    const unsigned int nb = bgr[no + 0U];
    const unsigned int ng = bgr[no + 1U];
    const unsigned int nr = bgr[no + 2U];
    const unsigned int nluma = (nb + 2U * ng + nr) >> 2U;
    gradient = luma >= nluma ? (luma - nluma) : (nluma - luma);
  }

  atomicAdd(&stats[CAMERA_HEALTH_COUNT], 1ULL);
  atomicAdd(&stats[CAMERA_HEALTH_SUM], static_cast<unsigned long long>(luma));
  atomicAdd(&stats[CAMERA_HEALTH_SUM_SQ],
    static_cast<unsigned long long>(luma) * static_cast<unsigned long long>(luma));
  if (static_cast<int>(luma) <= dark_threshold) atomicAdd(&stats[CAMERA_HEALTH_DARK], 1ULL);
  if (static_cast<int>(luma) >= bright_threshold) atomicAdd(&stats[CAMERA_HEALTH_BRIGHT], 1ULL);
  atomicAdd(&stats[CAMERA_HEALTH_GRADIENT], static_cast<unsigned long long>(gradient));
}


__global__ void nearFieldDrivableStatsKernel(
  const uint8_t *__restrict__ drivable_mask,
  int width,
  int height,
  int sample_stride,
  float center_corridor_fraction,
  float bottom_roi_fraction,
  unsigned long long *__restrict__ stats)
{
  const int sx = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  const int sy = static_cast<int>(blockIdx.y * blockDim.y + threadIdx.y);
  const int x = sx * sample_stride;
  const int y = sy * sample_stride;
  if (!drivable_mask || x >= width || y >= height) return;

  const float corridor = fminf(1.0F, fmaxf(0.05F, center_corridor_fraction));
  const float bottom_fraction = fminf(0.80F, fmaxf(0.05F, bottom_roi_fraction));
  const float half_width = 0.5F * static_cast<float>(width) * corridor;
  const float center = 0.5F * static_cast<float>(width - 1);
  const int roi_top = static_cast<int>((1.0F - bottom_fraction) * static_cast<float>(height));
  if (fabsf(static_cast<float>(x) - center) > half_width || y < roi_top) return;

  atomicAdd(&stats[NEAR_FIELD_SAMPLE_COUNT], 1ULL);
  if (drivable_mask[y * width + x] > 0U) {
    atomicAdd(&stats[NEAR_FIELD_DRIVABLE_COUNT], 1ULL);
  }
}

__device__ __forceinline__ float bilinearChannel(
  const uint8_t *image,
  int width,
  int height,
  float x,
  float y,
  int channel)
{
  x = fminf(static_cast<float>(width - 1), fmaxf(0.0F, x));
  y = fminf(static_cast<float>(height - 1), fmaxf(0.0F, y));
  const int x0 = static_cast<int>(floorf(x));
  const int y0 = static_cast<int>(floorf(y));
  const int x1 = min(x0 + 1, width - 1);
  const int y1 = min(y0 + 1, height - 1);
  const float dx = x - x0;
  const float dy = y - y0;
  const float p00 = image[(static_cast<size_t>(y0) * width + x0) * 3U + channel];
  const float p01 = image[(static_cast<size_t>(y0) * width + x1) * 3U + channel];
  const float p10 = image[(static_cast<size_t>(y1) * width + x0) * 3U + channel];
  const float p11 = image[(static_cast<size_t>(y1) * width + x1) * 3U + channel];
  const float top = p00 + (p01 - p00) * dx;
  const float bottom = p10 + (p11 - p10) * dx;
  return top + (bottom - top) * dy;
}

// Resize letterbox bilinear, BGR->RGB, normalisasi 0..1, dan HWC->NCHW.
__global__ void preprocessBgrKernel(
  const uint8_t *__restrict__ input,
  int input_width,
  int input_height,
  float *__restrict__ output,
  int output_width,
  int output_height,
  float gain,
  float pad_x,
  float pad_y)
{
  const int x = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  const int y = static_cast<int>(blockIdx.y * blockDim.y + threadIdx.y);
  if (x >= output_width || y >= output_height) return;

  const size_t plane = static_cast<size_t>(output_width) * output_height;
  const size_t index = static_cast<size_t>(y) * output_width + x;
  const float source_x = (static_cast<float>(x) + 0.5F - pad_x) / gain - 0.5F;
  const float source_y = (static_cast<float>(y) + 0.5F - pad_y) / gain - 0.5F;
  if (source_x < -0.5F || source_x > input_width - 0.5F || source_y < -0.5F || source_y > input_height - 0.5F) {
    constexpr float pad = 114.0F / 255.0F;
    output[index] = pad;
    output[plane + index] = pad;
    output[2U * plane + index] = pad;
    return;
  }
  const float b = bilinearChannel(input, input_width, input_height, source_x, source_y, 0) / 255.0F;
  const float g = bilinearChannel(input, input_width, input_height, source_x, source_y, 1) / 255.0F;
  const float r = bilinearChannel(input, input_width, input_height, source_x, source_y, 2) / 255.0F;
  output[index] = r;
  output[plane + index] = g;
  output[2U * plane + index] = b;
}

// Fungsi GPU: Menghitung sigmoid untuk decode confidence/class YOLOP.
__device__ __forceinline__ float sigmoidGpu(float value) {
  return 1.0F / (1.0F + expf(-value));
}


// Decode satu head YOLO menjadi kandidat bounding box di VRAM.
__global__ void decodeHeadKernel(
  const float *__restrict__ head,
  const float *__restrict__ anchor_grid,
  int grid_width,
  int grid_height,
  int stride,
  int class_count,
  float confidence_threshold,
  DetectionGpu *__restrict__ candidates,
  int *__restrict__ candidate_count,
  int max_candidates)
{
  const int gx = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  const int gy = static_cast<int>(blockIdx.y * blockDim.y + threadIdx.y);
  const int anchor = static_cast<int>(blockIdx.z);
  if (gx >= grid_width || gy >= grid_height || anchor >= 3) return;

  const int area = grid_width * grid_height;
  const int attrs = 5 + class_count;
  const int cell = gy * grid_width + gx;
  const int base = anchor * attrs * area;
  const float objectness = sigmoidGpu(head[base + 4 * area + cell]);
  if (objectness < confidence_threshold) return;

  float best_score = 0.0F;
  int best_class = -1;
  for (int cls = 0; cls < class_count; ++cls) {
    const float score = objectness * sigmoidGpu(head[base + (5 + cls) * area + cell]);
    if (score > best_score) {
      best_score = score;
      best_class = cls;
    }
  }
  if (best_score < confidence_threshold) return;

  const float tx = sigmoidGpu(head[base + 0 * area + cell]);
  const float ty = sigmoidGpu(head[base + 1 * area + cell]);
  const float tw = sigmoidGpu(head[base + 2 * area + cell]);
  const float th = sigmoidGpu(head[base + 3 * area + cell]);
  const float cx = (tx * 2.0F - 0.5F + gx) * stride;
  const float cy = (ty * 2.0F - 0.5F + gy) * stride;
  const float anchor_w = anchor_grid[anchor * 2];
  const float anchor_h = anchor_grid[anchor * 2 + 1];
  const float width = powf(tw * 2.0F, 2.0F) * anchor_w;
  const float height = powf(th * 2.0F, 2.0F) * anchor_h;

  const int out = atomicAdd(candidate_count, 1);
  if (out < max_candidates) {
    candidates[out] = DetectionGpu{
      cx - width * 0.5F,
      cy - height * 0.5F,
      cx + width * 0.5F,
      cy + height * 0.5F,
      best_score,
      best_class,
      1,
      0};
  }
}

// Fungsi GPU: Menghitung IoU dua bbox untuk deterministic non-maximum suppression.
__device__ __forceinline__ float intersectionOverUnion(const DetectionGpu &a, const DetectionGpu &b) {
  const float left = fmaxf(a.x1, b.x1);
  const float top = fmaxf(a.y1, b.y1);
  const float right = fminf(a.x2, b.x2);
  const float bottom = fminf(a.y2, b.y2);
  const float intersection = fmaxf(0.0F, right - left) * fmaxf(0.0F, bottom - top);
  const float area_a = fmaxf(0.0F, a.x2 - a.x1) * fmaxf(0.0F, a.y2 - a.y1);
  const float area_b = fmaxf(0.0F, b.x2 - b.x1) * fmaxf(0.0F, b.y2 - b.y1);
  return intersection / (area_a + area_b - intersection + 1.0e-6F);
}

// Greedy NMS official-equivalent: torchvision.ops.nms memproses score tertinggi,
// mempertahankan kandidat terbaik, lalu menekan kandidat sekelas dengan IoU di
// atas threshold. Kernel satu blok ini melakukan pemilihan maksimum + suppression
// secara iteratif di GPU tanpa round-trip seluruh kandidat ke CPU.
// keep: 0=suppressed, 1=eligible, 2=selected/output.
__global__ void officialGreedyNmsKernel(
  DetectionGpu *__restrict__ candidates,
  const int *__restrict__ candidate_count,
  int max_candidates,
  int max_detections,
  float iou_threshold)
{
  constexpr int kThreads = 256;
  __shared__ float best_scores[kThreads];
  __shared__ int best_indices[kThreads];
  __shared__ int selected_index;
  __shared__ int selected_count;
  __shared__ int done;

  if (threadIdx.x == 0) {
    selected_count = 0;
    selected_index = -1;
    done = 0;
  }
  __syncthreads();

  const int count = min(*candidate_count, max_candidates);
  for (int iteration = 0; iteration < max_detections; ++iteration) {
    float local_score = -1.0F;
    int local_index = -1;
    for (int i = static_cast<int>(threadIdx.x); i < count; i += static_cast<int>(blockDim.x)) {
      const DetectionGpu candidate = candidates[i];
      if (candidate.keep != 1) continue;
      const bool better = candidate.confidence > local_score;
      const bool tie = candidate.confidence == local_score && (local_index < 0 || i < local_index);
      if (better || tie) {
        local_score = candidate.confidence;
        local_index = i;
      }
    }

    best_scores[threadIdx.x] = local_score;
    best_indices[threadIdx.x] = local_index;
    __syncthreads();

    for (int offset = kThreads / 2; offset > 0; offset >>= 1) {
      if (static_cast<int>(threadIdx.x) < offset) {
        const float other_score = best_scores[threadIdx.x + offset];
        const int other_index = best_indices[threadIdx.x + offset];
        const float mine_score = best_scores[threadIdx.x];
        const int mine_index = best_indices[threadIdx.x];
        const bool better = other_score > mine_score;
        const bool tie = other_score == mine_score && other_index >= 0 &&
          (mine_index < 0 || other_index < mine_index);
        if (better || tie) {
          best_scores[threadIdx.x] = other_score;
          best_indices[threadIdx.x] = other_index;
        }
      }
      __syncthreads();
    }

    if (threadIdx.x == 0) {
      selected_index = best_indices[0];
      if (selected_index < 0) {
        done = 1;
      } else {
        candidates[selected_index].keep = 2;
        ++selected_count;
        done = selected_count >= max_detections ? 1 : 0;
      }
    }
    __syncthreads();
    if (selected_index < 0) break;

    // Setelah output ke-max_det terpilih, ranking berikutnya tidak diperlukan.
    if (done != 0) break;

    const DetectionGpu selected = candidates[selected_index];
    for (int i = static_cast<int>(threadIdx.x); i < count; i += static_cast<int>(blockDim.x)) {
      if (i == selected_index) continue;
      const DetectionGpu candidate = candidates[i];
      if (candidate.keep != 1 || candidate.class_id != selected.class_id) continue;
      if (intersectionOverUnion(selected, candidate) > iou_threshold) {
        candidates[i].keep = 0;
      }
    }
    __syncthreads();
  }
}

// Memadatkan hasil NMS dan memetakan koordinat dari ruang model ke citra asli.
__global__ void compactAndScaleKernel(
  const DetectionGpu *__restrict__ candidates,
  const int *__restrict__ candidate_count,
  int max_candidates,
  DetectionGpu *__restrict__ final_detections,
  int *__restrict__ final_count,
  int max_detections,
  float gain,
  float pad_x,
  float pad_y,
  int image_width,
  int image_height)
{
  const int index = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  const int count = min(*candidate_count, max_candidates);
  if (index >= count || candidates[index].keep != 2) return;

  DetectionGpu detection = candidates[index];
  const float raw_x1 = (detection.x1 - pad_x) / gain;
  const float raw_y1 = (detection.y1 - pad_y) / gain;
  const float raw_x2 = (detection.x2 - pad_x) / gain;
  const float raw_y2 = (detection.y2 - pad_y) / gain;
  detection.clip_flags = 0;
  if (raw_x1 <= 0.0F) detection.clip_flags |= DETECTION_CLIP_LEFT;
  if (raw_y1 <= 0.0F) detection.clip_flags |= DETECTION_CLIP_TOP;
  if (raw_x2 >= static_cast<float>(image_width - 1)) detection.clip_flags |= DETECTION_CLIP_RIGHT;
  if (raw_y2 >= static_cast<float>(image_height - 1)) detection.clip_flags |= DETECTION_CLIP_BOTTOM;
  detection.x1 = fminf(static_cast<float>(image_width - 1), fmaxf(0.0F, raw_x1));
  detection.y1 = fminf(static_cast<float>(image_height - 1), fmaxf(0.0F, raw_y1));
  detection.x2 = fminf(static_cast<float>(image_width - 1), fmaxf(0.0F, raw_x2));
  detection.y2 = fminf(static_cast<float>(image_height - 1), fmaxf(0.0F, raw_y2));
  if (detection.x2 <= detection.x1 || detection.y2 <= detection.y1) return;

  const int out = atomicAdd(final_count, 1);
  if (out < max_detections) {
    final_detections[out] = detection;
  }
}

__device__ __forceinline__ float bilinearFloatPlane(
  const float *plane,
  int width,
  int height,
  float x,
  float y)
{
  x = fminf(static_cast<float>(width - 1), fmaxf(0.0F, x));
  y = fminf(static_cast<float>(height - 1), fmaxf(0.0F, y));
  const int x0 = static_cast<int>(floorf(x));
  const int y0 = static_cast<int>(floorf(y));
  const int x1 = min(x0 + 1, width - 1);
  const int y1 = min(y0 + 1, height - 1);
  const float dx = x - x0;
  const float dy = y - y0;
  const float p00 = plane[y0 * width + x0];
  const float p01 = plane[y0 * width + x1];
  const float p10 = plane[y1 * width + x0];
  const float p11 = plane[y1 * width + x1];
  return (p00 + (p01 - p00) * dx) + ((p10 + (p11 - p10) * dx) - (p00 + (p01 - p00) * dx)) * dy;
}

// Membentuk tiga mask resolusi kamera: drivable, lane, dan class mask.
__global__ void segmentationKernel(
  const float *__restrict__ drivable,
  const float *__restrict__ lane,
  int model_width,
  int model_height,
  uint8_t *__restrict__ drivable_mask,
  uint8_t *__restrict__ lane_mask,
  uint8_t *__restrict__ class_mask,
  int image_width,
  int image_height,
  float gain,
  float pad_x,
  float pad_y,
  float lane_threshold)
{
  const int x = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  const int y = static_cast<int>(blockIdx.y * blockDim.y + threadIdx.y);
  if (x >= image_width || y >= image_height) return;

  const float model_x = (static_cast<float>(x) + 0.5F) * gain + pad_x - 0.5F;
  const float model_y = (static_cast<float>(y) + 0.5F) * gain + pad_y - 0.5F;
  const size_t output_index = static_cast<size_t>(y) * image_width + x;
  if (model_x < 0.0F || model_x >= model_width || model_y < 0.0F || model_y >= model_height) {
    drivable_mask[output_index] = 0;
    lane_mask[output_index] = 0;
    if (class_mask) class_mask[output_index] = 0;
    return;
  }

  const size_t plane = static_cast<size_t>(model_width) * model_height;
  const float background = bilinearFloatPlane(drivable, model_width, model_height, model_x, model_y);
  const float road = bilinearFloatPlane(drivable + plane, model_width, model_height, model_x, model_y);
  const float lane_value = bilinearFloatPlane(lane, model_width, model_height, model_x, model_y);
  const bool is_road = road > background;
  const bool is_lane = lane_value > lane_threshold;
  drivable_mask[output_index] = is_road ? 255U : 0U;
  lane_mask[output_index] = is_lane ? 255U : 0U;
  if (class_mask) class_mask[output_index] = is_lane ? 2U : (is_road ? 1U : 0U);
}

// Menggabungkan drivable area (hijau) dan lane (merah) ke citra BGR.
__global__ void overlayMasksKernel(
  uint8_t *__restrict__ image,
  const uint8_t *__restrict__ drivable_mask,
  const uint8_t *__restrict__ lane_mask,
  int width,
  int height,
  float alpha)
{
  const int x = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  const int y = static_cast<int>(blockIdx.y * blockDim.y + threadIdx.y);
  if (x >= width || y >= height) return;
  const size_t pixel = static_cast<size_t>(y) * width + x;
  const size_t base = pixel * 3U;
  if (lane_mask[pixel]) {
    image[base + 0U] = clampU8(image[base + 0U] * (1.0F - alpha));
    image[base + 1U] = clampU8(image[base + 1U] * (1.0F - alpha));
    image[base + 2U] = clampU8(image[base + 2U] * (1.0F - alpha) + 255.0F * alpha);
  } else if (drivable_mask[pixel]) {
    image[base + 0U] = clampU8(image[base + 0U] * (1.0F - alpha));
    image[base + 1U] = clampU8(image[base + 1U] * (1.0F - alpha) + 255.0F * alpha);
    image[base + 2U] = clampU8(image[base + 2U] * (1.0F - alpha));
  }
}

// Fungsi GPU: Menulis pixel BGR dengan pemeriksaan batas untuk overlay/box annotation.
__device__ __forceinline__ void setPixelBgr(uint8_t *image, int width, int height, int x, int y, uint8_t b, uint8_t g, uint8_t r) {
  if (x < 0 || x >= width || y < 0 || y >= height) return;
  const size_t index = (static_cast<size_t>(y) * width + x) * 3U;
  image[index + 0U] = b;
  image[index + 1U] = g;
  image[index + 2U] = r;
}

// Menggambar bounding box kuning langsung pada buffer citra di GPU.
__global__ void drawBoxesKernel(
  uint8_t *__restrict__ image,
  int width,
  int height,
  const DetectionGpu *__restrict__ detections,
  const int *__restrict__ count,
  int max_detections,
  int thickness)
{
  const int detection_index = static_cast<int>(blockIdx.x);
  const int valid_count = min(*count, max_detections);
  if (detection_index >= valid_count) return;
  const DetectionGpu detection = detections[detection_index];
  const int x1 = max(0, min(width - 1, static_cast<int>(roundf(detection.x1))));
  const int y1 = max(0, min(height - 1, static_cast<int>(roundf(detection.y1))));
  const int x2 = max(0, min(width - 1, static_cast<int>(roundf(detection.x2))));
  const int y2 = max(0, min(height - 1, static_cast<int>(roundf(detection.y2))));
  const int tid = static_cast<int>(threadIdx.x);
  const int horizontal = max(0, x2 - x1 + 1);
  const int vertical = max(0, y2 - y1 + 1);
  for (int offset = tid; offset < horizontal; offset += static_cast<int>(blockDim.x)) {
    for (int t = 0; t < thickness; ++t) {
      setPixelBgr(image, width, height, x1 + offset, y1 + t, 0U, 255U, 255U);
      setPixelBgr(image, width, height, x1 + offset, y2 - t, 0U, 255U, 255U);
    }
  }
  for (int offset = tid; offset < vertical; offset += static_cast<int>(blockDim.x)) {
    for (int t = 0; t < thickness; ++t) {
      setPixelBgr(image, width, height, x1 + t, y1 + offset, 0U, 255U, 255U);
      setPixelBgr(image, width, height, x2 - t, y1 + offset, 0U, 255U, 255U);
    }
  }
}

// ============================================================================
// RING-BUFFER PINNED MEMORY UNTUK RVIZ
// ============================================================================
// State slot mencegah thread inferensi menimpa buffer yang sedang diserialisasi
// oleh thread publisher ROS. Kebijakan utama: hanya frame terbaru yang dipertahankan.
enum class RvizFrameState : uint8_t {
  FREE = 0,
  FILLING,
  READY,
  PUBLISHING,
};

struct RvizFrameSlot {
  uint8_t *host{nullptr};
  RvizFrameState state{RvizFrameState::FREE};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  uint64_t sequence{0};
};

// Geometri lane siap-safety. Struktur ini hanya berisi beberapa puluh sample
// metrik; pixel dan homography tetap selesai di CUDA sebelum data ini dibuat.
struct LaneGeometryRuntime {
  bool valid{false};
  float left_clearance_m{0.0F};
  float right_clearance_m{0.0F};
  float center_error_m{0.0F};
  float heading_error_rad{0.0F};
  float road_width_m{0.0F};
  float confidence{0.0F};
  bool single_side_estimated{false};
  std::string lane_source{"NONE"};
  std::vector<std::array<float, 2>> left_samples;
  std::vector<std::array<float, 2>> right_samples;
  std::vector<std::array<float, 2>> center_samples;
};

// Obstacle hasil proyeksi GPU yang sudah diberi track-id oleh state kecil CPU.
// BBox pixel dipertahankan agar alat kalibrasi statik C++ benar-benar berguna.
struct MetricDetectionRuntime {
  int track_id{-1};
  int class_id{-1};
  float score{0.0F};
  // Jarak homography mentah dipertahankan terpisah dari nilai tracking/EMA.
  // Logger kalibrasi harus membandingkan model bbox terhadap pengukuran frame asli.
  float forward_homography_m{0.0F};
  float forward_m{0.0F};
  float left_m{0.0F};
  float width_m{0.20F};
  float center_x_px{0.0F};
  float center_y_px{0.0F};
  float bottom_y_px{0.0F};
  float width_px{0.0F};
  float height_px{0.0F};
  float area_px{0.0F};
  int missed_frames{0};
  bool confirmed{false};
};

// State tracker obstacle antar-frame. Tracking ini sengaja di CPU karena jumlah
// bbox kecil; peluncuran kernel CUDA tambahan justru lebih mahal daripada O(N^2)
// yang hanya bekerja pada puluhan deteksi setelah NMS.
struct ObstacleTrackRuntime {
  int track_id{-1};
  int class_id{-1};
  float score{0.0F};
  float forward_m{0.0F};
  float left_m{0.0F};
  float width_m{0.20F};
  int hits{1};
  int missed{0};
  bool confirmed{false};
};

// ============================================================================
// NODE UTAMA ROS 2
// ============================================================================
class AstraYolopGpuNode final : public rclcpp::Node {
public:
  // Urutan inisialisasi dibuat eksplisit agar error engine, CUDA, atau kamera
  // muncul sebelum thread pemrosesan mulai berjalan.
  // Fungsi: Menginisialisasi seluruh pipeline kamera, CUDA, TensorRT, publisher, dan runtime safety terintegrasi.
  AstraYolopGpuNode() : Node("perception") {
    declareParameters();
    readParameters();
    camera_connected_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/perception/camera_connected",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    publishCameraConnected(false);
    // Untuk CUDA/OpenGL interoperability, context OpenGL dibuat sebelum
    // pemanggilan CUDA pertama.
    initializeOpenCvWindow();
    initializeCuda();
    // Kamera dibuka sebelum alokasi frame GPU agar mode 30 FPS hasil negosiasi
    // menentukan ukuran buffer sebenarnya dan staging VRAM tidak dialokasikan
    // bila USERPTR sudah CUDA-mapped.
    initializeCamera();
    if (!rclcpp::ok()) {
      return;
    }
    initializeGroundProjection();
    allocateBuffersAndLoadEngine();
    createPublishers();
    createIntegratedPerceptionRuntime();
    warmupTensorRt();

    // SIGINT dapat datang ketika TensorRT warmup masih berjalan. Jangan membuat
    // worker atau guard condition baru setelah context ROS ditutup.
    if (!rclcpp::ok()) {
      return;
    }

    running_.store(true);
    if (async_rviz_publish_ && annotated_pub_) {
      rviz_publisher_worker_ = std::thread(&AstraYolopGpuNode::rvizPublisherLoop, this);
    }
    worker_ = std::thread(&AstraYolopGpuNode::processingLoop, this);
  }

  // Destructor menghentikan thread lebih dahulu, lalu kamera, buffer, dan stream.
  // Urutan ini mencegah thread mengakses VRAM yang sudah dibebaskan.
  ~AstraYolopGpuNode() override {
    publishCameraConnected(false);
    running_.store(false);
    rviz_frame_cv_.notify_all();
    if (worker_.joinable()) worker_.join();
    if (rviz_publisher_worker_.joinable()) rviz_publisher_worker_.join();
    camera_.stopStreaming();
    if (opencv_window_initialized_) cv::destroyAllWindows();
    releaseBuffers();
    if (stream_) cudaStreamDestroy(stream_);
  }

private:
  // Runtime autonomous uses exactly one external TensorRT engine. The model is
  // intentionally NOT copied into src/install so the ROS source package stays small
  // and there is no silent model-selection fallback.
  std::string resolveEnginePath(const std::string & requested) {
    static constexpr const char * kRequiredEnginePath =
      "/home/sirobo/ros/models/yolopv2.engine";
    const fs::path required = normalizedPath(fs::path(kRequiredEnginePath));

    if (!requested.empty() && normalizedPath(fs::path(requested)) != required) {
      throw std::runtime_error(
        std::string("engine_path harus persis: ") + required.string() +
        " ; path lain sengaja ditolak agar model runtime tidak ambigu. Diterima: " + requested);
    }
    if (!nonEmptyRegularFile(required)) {
      throw std::runtime_error(
        std::string("TensorRT engine wajib tidak ditemukan/kosong: ") + required.string());
    }
    RCLCPP_INFO(get_logger(), "TensorRT engine external: %s", required.string().c_str());
    return required.string();
  }

  // Mendeklarasikan seluruh parameter agar dapat dibaca dari YAML/launch.
  // Fungsi: Mendeklarasikan seluruh parameter kamera, GPU, projection, publisher, lane safety, obstacle, dan mixer.
  void declareParameters() {
    declare_parameter<std::string>("engine_path", "/home/sirobo/ros/models/yolopv2.engine");
    declare_parameter<int>("gpu_device", 0);
    declare_parameter<std::string>("rgb_device", "auto");
    declare_parameter<int>("rgb_width", 1280);
    declare_parameter<int>("rgb_height", 720);
    declare_parameter<int>("fps", 30);
    declare_parameter<std::string>("v4l2_pixel_format", "YUYV");
    declare_parameter<bool>("allow_mjpeg_cpu_fallback", true);
    declare_parameter<bool>("use_v4l2_userptr_zero_copy", true);
    declare_parameter<int>("v4l2_buffer_count", 3);
    declare_parameter<bool>("allow_resolution_fallback_for_fps", false);
    declare_parameter<bool>("strict_camera_mode", true);
    // Pada autonomous launch, kamera USB dapat terlambat siap atau masih
    // direlease oleh proses sebelumnya. Retry internal mencegah crash/respawn loop.
    declare_parameter<bool>("camera_hotplug_retry", true);
    declare_parameter<double>("camera_retry_interval_sec", 2.0);
    declare_parameter<double>("camera_retry_log_interval_sec", 30.0);
    declare_parameter<bool>("flip_horizontal", true);
    declare_parameter<double>("confidence_threshold", 0.30);
    declare_parameter<double>("iou_threshold", 0.45);
    declare_parameter<double>("lane_threshold", 0.50);
    declare_parameter<int>("max_candidates", 16384);
    declare_parameter<int>("max_detections", 300);
    declare_parameter<int>("warmup_iterations", 5);
    declare_parameter<double>("overlay_alpha", 0.45);
    declare_parameter<int>("box_thickness", 2);
    declare_parameter<bool>("publish_annotated", true);
    declare_parameter<bool>("publish_raw_rgb", false);
    // publish_masks dipertahankan sebagai alias legacy. Runtime autonomous baru
    // memakai flag terpisah agar drivable mask tidak perlu keluar dari GPU.
    declare_parameter<bool>("publish_masks", false);
    declare_parameter<bool>("publish_drivable_mask", false);
    declare_parameter<bool>("publish_lane_mask", true);
    declare_parameter<bool>("publish_class_mask", false);
    declare_parameter<bool>("publish_detections", false);
    declare_parameter<bool>("publish_lane_metrics", true);
    declare_parameter<bool>("publish_obstacle_metrics", true);
    declare_parameter<bool>("publish_only_when_subscribed", true);
    declare_parameter<bool>("show_opencv_cuda_window", false);
    declare_parameter<bool>("opencv_use_cuda_opengl", true);
    declare_parameter<int>("opencv_window_width", 640);
    declare_parameter<int>("opencv_window_height", 480);
    declare_parameter<bool>("opencv_window_fullscreen", false);
    declare_parameter<bool>("draw_text_labels_cpu", false);
    declare_parameter<bool>("async_rviz_publish", true);
    declare_parameter<std::string>("publisher_reliability", "reliable");
    declare_parameter<int>("rviz_publish_buffer_count", 2);
    declare_parameter<double>("rviz_max_publish_rate_hz", 10.0);
    declare_parameter<std::string>("frame_id", "camera_color_optical_frame");
    declare_parameter<std::string>("annotated_topic", "/camera/yolop/image_annotated");
    declare_parameter<std::string>("raw_topic", "/camera/color/image_raw");
    declare_parameter<std::string>("camera_info_topic", "/camera/color/camera_info");
    declare_parameter<std::string>("detections_topic", "/yolop/detections");
    declare_parameter<std::string>("drivable_mask_topic", "/yolop/drivable_mask");
    declare_parameter<std::string>("lane_mask_topic", "/yolop/lane_mask");
    declare_parameter<std::string>("lane_metrics_topic", "/yolop/lane_metrics");
    declare_parameter<std::string>("drivable_space_topic", "/perception/drivable_space");
    declare_parameter<std::string>("drivable_boundary_points_topic", "/perception/drivable_boundary_points");
    declare_parameter<std::string>("obstacle_metrics_topic", "/yolop/obstacle_metrics_raw");
    // BAB IV 4.6: statistik pipeline yang benar-benar diukur oleh source.
    // Ini adalah total processFrame(), bukan klaim pure TensorRT inference time.
    declare_parameter<std::string>("performance_topic", "/perception/performance");
    declare_parameter<std::string>("metric_frame_id", "base_footprint");
    declare_parameter<std::string>("class_mask_topic", "/yolop/class_mask");

    // Ground-plane calibration is owned by this GPU node. Coordinates are
    // specified in their calibration image resolution and scaled once to the
    // actual V4L2 mode after the camera opens.
    declare_parameter<bool>("camera_metric_calibration_validated", false);
    declare_parameter<int>("ground_calibration_width", 1280);
    declare_parameter<int>("ground_calibration_height", 720);
    declare_parameter<std::vector<double>>("ground_src_points",
      std::vector<double>{40.0,680.0, 1240.0,680.0, 760.0,350.0, 520.0,350.0});
    declare_parameter<std::vector<double>>("ground_dst_points",
      std::vector<double>{0.0,513.5714111328125, 1279.0,503.29998779296875,
                          1279.0,10.271428108215332, 0.0,0.0});
    declare_parameter<int>("ground_canvas_width", 1280);
    declare_parameter<int>("ground_canvas_height", 720);
    declare_parameter<double>("ground_origin_x_px", 596.8666666666666);
    declare_parameter<double>("ground_origin_y_px", 719.0);
    declare_parameter<double>("ground_meters_per_pixel_x", 0.003909304143862393);
    declare_parameter<double>("ground_meters_per_pixel_y", 0.009735744089012517);

    declare_parameter<double>("metric_forward_offset_m", 0.0);
    declare_parameter<double>("metric_lateral_offset_m", 0.0);
    declare_parameter<double>("metric_minimum_forward_m", 0.20);
    declare_parameter<double>("metric_maximum_forward_m", 4.0);
    declare_parameter<double>("metric_maximum_abs_left_m", 2.5);

    declare_parameter<double>("lane_vehicle_width_m", 0.55);
    declare_parameter<double>("lane_minimum_road_width_m", 3.5);
    declare_parameter<double>("lane_maximum_road_width_m", 6.5);
    declare_parameter<double>("lane_lookahead_near_m", 1.0);
    declare_parameter<double>("lane_lookahead_far_m", 3.0);
    declare_parameter<double>("lane_control_lookahead_m", 2.0);
    declare_parameter<int>("lane_sample_rows", 72);
    declare_parameter<int>("lane_minimum_valid_rows", 18);
    declare_parameter<double>("lane_minimum_center_offset_m", 0.08);
    declare_parameter<double>("lane_ema_alpha", 0.35);
    // Free-space envelope langsung dari drivable head YOLOPv2. Hanya array kecil
    // per lookahead yang keluar GPU; full mask tetap tidak perlu dipublish.
    declare_parameter<double>("drivable_space_near_m", 0.35);
    declare_parameter<double>("drivable_space_far_m", 4.0);
    declare_parameter<double>("drivable_space_max_abs_left_m", 2.8);
    declare_parameter<int>("drivable_space_sample_rows", 64);
    declare_parameter<int>("drivable_space_min_pixels_per_row", 8);
    declare_parameter<int>("drivable_space_pixel_stride_px", 4);
    declare_parameter<double>("drivable_boundary_guard_offset_m", 0.0);
    declare_parameter<double>("drivable_boundary_z_m", 0.20);
    declare_parameter<double>("drivable_lane_constraint_min_confidence", 0.25);
    declare_parameter<double>("camera_fx", 455.0);
    declare_parameter<double>("camera_fy", 606.6667);
    declare_parameter<double>("camera_cx", 320.0);
    declare_parameter<double>("camera_cy", 240.0);
    declare_parameter<std::vector<double>>("camera_distortion", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0});

    // Runtime safety C++ menggantikan lane_safety_perception.py,
    // perception_control_mixer.py, perception_safety_core.py, dan
    // yolop_obstacle_projector.py. Semua input geometri berasal langsung dari
    // buffer metrik CUDA pada proses yang sama, bukan melalui JSON antar-node.
    declare_parameter<bool>("enable_integrated_perception_runtime", true);
    declare_parameter<bool>("lane_safety_enabled", false);
    declare_parameter<std::string>("lane_state_topic", "/perception/lane_safety_state");
    declare_parameter<std::string>("lane_centerline_topic", "/perception/lane_centerline");
    declare_parameter<std::string>("lane_safety_markers_topic", "/perception/lane_safety_markers");
    declare_parameter<double>("nominal_road_width_m", 5.0);
    declare_parameter<double>("edge_warning_clearance_m", 1.00);
    declare_parameter<double>("edge_critical_clearance_m", 0.40);
    declare_parameter<double>("edge_release_clearance_m", 1.20);
    declare_parameter<double>("center_deadband_m", 0.25);
    declare_parameter<double>("minimum_metric_confidence", 0.10);
    declare_parameter<int>("state_confirm_frames", 3);
    declare_parameter<int>("release_confirm_frames", 5);
    declare_parameter<int>("lost_confirm_frames", 2);

    declare_parameter<std::string>("object_points_topic", "/perception/object_points");
    declare_parameter<std::string>("object_clearing_points_topic", "/perception/object_clearing_points");
    declare_parameter<std::string>("obstacle_markers_topic", "/perception/obstacle_markers");
    declare_parameter<std::string>("perception_obstacle_metrics_topic", "/perception/obstacle_metrics");
    // V17 calibration feed: hasil YOLO post-NMS apa adanya, sebelum filter
    // drivable/range/safety. x/y adalah proyeksi homography mentah dan TIDAK
    // dipakai sebagai obstacle safety sampai lolos pipeline metric terpisah.
    declare_parameter<std::string>("raw_detection_summary_topic", "/perception/raw_detections");
    declare_parameter<std::string>("raw_detection_markers_topic", "/perception/raw_detection_markers");
    declare_parameter<double>("raw_detection_log_period_sec", 0.50);
    declare_parameter<double>("minimum_obstacle_confidence", 0.30);
    declare_parameter<bool>("accept_all_detected_classes_as_obstacles", false);
    // ID 0/2/3 adalah target mapping person/car/motorcycle yang diminta. Mapping
    // semantik engine wajib divalidasi dari /perception/raw_detections; jangan
    // menganggap nama class hanya dari angka logit TensorRT.
    declare_parameter<std::vector<int64_t>>(
      "safety_obstacle_class_ids", std::vector<int64_t>{0, 2, 3});
    declare_parameter<bool>("require_drivable_contact", true);
    declare_parameter<int>("drivable_contact_radius_px", 12);
    declare_parameter<int>("drivable_contact_vertical_tolerance_px", 8);
    declare_parameter<int>("drivable_contact_min_samples", 3);
    declare_parameter<double>("drivable_contact_min_fraction", 0.20);
    declare_parameter<int>("points_per_box", 5);
    declare_parameter<double>("obstacle_cloud_max_publish_rate_hz", 12.0);
    declare_parameter<double>("clearing_fov_deg", 100.0);
    declare_parameter<double>("clearing_range_m", 4.5);
    declare_parameter<int>("clearing_ray_count", 41);
    declare_parameter<double>("track_match_distance_m", 0.90);
    declare_parameter<double>("track_ema_alpha", 0.45);
    declare_parameter<int>("track_confirm_hits", 3);
    declare_parameter<int>("track_max_missed_frames", 8);
    declare_parameter<double>("clearing_obstacle_margin_m", 0.20);

    declare_parameter<std::string>("control_mode", "active");
    declare_parameter<std::string>("nav_cmd_topic", "/cmd_vel_nav_smoothed");
    declare_parameter<std::string>("safe_cmd_topic", "/cmd_vel/perception_advisory");
    declare_parameter<std::string>("control_state_topic", "/perception/lane_control_state");
    declare_parameter<std::string>("control_markers_topic", "/perception/lane_control_markers");
    declare_parameter<double>("control_rate_hz", 30.0);
    declare_parameter<double>("cmd_timeout_sec", 0.50);
    declare_parameter<double>("lane_state_timeout_sec", 0.50);
    declare_parameter<double>("obstacle_state_timeout_sec", 0.70);
    declare_parameter<double>("wheelbase_m", 0.70);
    declare_parameter<double>("maximum_steering_angle_rad", 0.34);
    declare_parameter<double>("recenter_speed_mps", 0.20);
    declare_parameter<double>("critical_recenter_speed_mps", 0.10);
    declare_parameter<double>("center_gain", 0.55);
    declare_parameter<double>("heading_gain", 0.70);
    declare_parameter<double>("lane_blend_gain", 1.0);
    declare_parameter<double>("minimum_speed_for_yaw_limit_mps", 0.10);
    declare_parameter<double>("obstacle_forward_min_m", 0.20);
    declare_parameter<double>("obstacle_forward_max_m", 3.0);
    declare_parameter<double>("obstacle_lateral_margin_m", 0.30);
    declare_parameter<double>("minimum_obstacle_width_m", 0.20);
    declare_parameter<int>("blocked_confirm_frames", 3);
    declare_parameter<int>("clear_confirm_frames", 5);

    // Fail-safe kamera bukan sumber steering. Kedua output berikut hanya menjadi
    // veto pada trajectory_safety_supervisor bila visibilitas hilang atau object
    // near-field terpotong oleh tepi bawah frame.
    declare_parameter<bool>("camera_health_enabled", true);
    declare_parameter<std::string>("camera_health_topic", "/perception/camera_healthy");
    declare_parameter<std::string>("camera_health_state_topic", "/perception/camera_health_state");
    declare_parameter<int>("camera_health_sample_stride_px", 16);
    declare_parameter<int>("camera_health_dark_luma", 18);
    declare_parameter<int>("camera_health_bright_luma", 245);
    declare_parameter<double>("camera_health_extreme_fraction", 0.97);
    declare_parameter<double>("camera_health_min_stddev", 5.0);
    declare_parameter<double>("camera_health_min_gradient", 2.0);
    declare_parameter<int>("camera_health_bad_confirm_frames", 5);
    declare_parameter<int>("camera_health_good_confirm_frames", 5);

    declare_parameter<bool>("near_field_emergency_enabled", true);
    declare_parameter<std::string>("emergency_stop_topic", "/perception/emergency_stop");
    declare_parameter<std::string>("near_field_state_topic", "/perception/near_field_state");
    declare_parameter<double>("near_field_min_confidence", 0.25);
    declare_parameter<double>("near_field_center_corridor_fraction", 0.65);
    declare_parameter<double>("near_field_min_bbox_height_fraction", 0.20);
    // Guard generik untuk benda sangat dekat yang tidak lagi menghasilkan bbox
    // utuh: lower-center ROI harus tetap mempunyai cukup drivable pixels.
    declare_parameter<bool>("near_field_drivable_guard_enabled", true);
    declare_parameter<double>("near_field_bottom_roi_fraction", 0.22);
    declare_parameter<double>("near_field_min_drivable_fraction", 0.12);
    declare_parameter<int>("near_field_confirm_frames", 2);
    declare_parameter<int>("near_field_release_frames", 5);
  }

  // Membaca parameter, membatasi rentang aman, dan memvalidasi konfigurasi.
  // Fungsi: Membaca, membatasi, dan memvalidasi parameter sebelum resource GPU dialokasikan.
  void readParameters() {
    engine_path_ = get_parameter("engine_path").as_string();
    gpu_device_ = get_parameter("gpu_device").as_int();
    rgb_device_ = get_parameter("rgb_device").as_string();
    width_ = get_parameter("rgb_width").as_int();
    height_ = get_parameter("rgb_height").as_int();
    fps_ = get_parameter("fps").as_int();
    pixel_format_ = get_parameter("v4l2_pixel_format").as_string();
    allow_mjpeg_ = get_parameter("allow_mjpeg_cpu_fallback").as_bool();
    use_userptr_ = get_parameter("use_v4l2_userptr_zero_copy").as_bool();
    v4l2_buffer_count_ = get_parameter("v4l2_buffer_count").as_int();
    allow_resolution_fallback_for_fps_ =
      get_parameter("allow_resolution_fallback_for_fps").as_bool();
    strict_camera_mode_ = get_parameter("strict_camera_mode").as_bool();
    camera_hotplug_retry_ = get_parameter("camera_hotplug_retry").as_bool();
    camera_retry_interval_sec_ = std::max(
      0.25, get_parameter("camera_retry_interval_sec").as_double());
    camera_retry_log_interval_sec_ = std::max(
      1.0, get_parameter("camera_retry_log_interval_sec").as_double());
    flip_horizontal_ = get_parameter("flip_horizontal").as_bool();
    if (strict_camera_mode_ && allow_resolution_fallback_for_fps_) {
      RCLCPP_WARN(
        get_logger(),
        "strict_camera_mode=true: menonaktifkan fallback resolusi agar mode kamera tetap eksak.");
      allow_resolution_fallback_for_fps_ = false;
    }
    confidence_threshold_ = static_cast<float>(get_parameter("confidence_threshold").as_double());
    iou_threshold_ = static_cast<float>(get_parameter("iou_threshold").as_double());
    lane_threshold_ = static_cast<float>(get_parameter("lane_threshold").as_double());
    max_candidates_ = get_parameter("max_candidates").as_int();
    max_detections_ = get_parameter("max_detections").as_int();
    warmup_iterations_ = get_parameter("warmup_iterations").as_int();
    overlay_alpha_ = static_cast<float>(get_parameter("overlay_alpha").as_double());
    box_thickness_ = get_parameter("box_thickness").as_int();
    publish_annotated_ = get_parameter("publish_annotated").as_bool();
    publish_raw_ = get_parameter("publish_raw_rgb").as_bool();
    const bool legacy_publish_masks = get_parameter("publish_masks").as_bool();
    publish_drivable_mask_ = get_parameter("publish_drivable_mask").as_bool() || legacy_publish_masks;
    publish_lane_mask_ = get_parameter("publish_lane_mask").as_bool() || legacy_publish_masks;
    publish_class_mask_ = get_parameter("publish_class_mask").as_bool();
    publish_detections_ = get_parameter("publish_detections").as_bool();
    publish_lane_metrics_ = get_parameter("publish_lane_metrics").as_bool();
    publish_obstacle_metrics_ = get_parameter("publish_obstacle_metrics").as_bool();
    publish_only_when_subscribed_ = get_parameter("publish_only_when_subscribed").as_bool();
    show_opencv_cuda_window_ = get_parameter("show_opencv_cuda_window").as_bool();
    opencv_use_cuda_opengl_ = get_parameter("opencv_use_cuda_opengl").as_bool();
    opencv_window_width_ = get_parameter("opencv_window_width").as_int();
    opencv_window_height_ = get_parameter("opencv_window_height").as_int();
    opencv_window_fullscreen_ = get_parameter("opencv_window_fullscreen").as_bool();
    draw_text_labels_cpu_ = get_parameter("draw_text_labels_cpu").as_bool();
    async_rviz_publish_ = get_parameter("async_rviz_publish").as_bool();
    publisher_reliability_ = get_parameter("publisher_reliability").as_string();
    rviz_publish_buffer_count_ = get_parameter("rviz_publish_buffer_count").as_int();
    rviz_max_publish_rate_hz_ = get_parameter("rviz_max_publish_rate_hz").as_double();
    frame_id_ = get_parameter("frame_id").as_string();
    annotated_topic_ = get_parameter("annotated_topic").as_string();
    raw_topic_ = get_parameter("raw_topic").as_string();
    camera_info_topic_ = get_parameter("camera_info_topic").as_string();
    detections_topic_ = get_parameter("detections_topic").as_string();
    drivable_mask_topic_ = get_parameter("drivable_mask_topic").as_string();
    lane_mask_topic_ = get_parameter("lane_mask_topic").as_string();
    lane_metrics_topic_ = get_parameter("lane_metrics_topic").as_string();
    drivable_space_topic_ = get_parameter("drivable_space_topic").as_string();
    drivable_boundary_points_topic_ = get_parameter("drivable_boundary_points_topic").as_string();
    obstacle_metrics_topic_ = get_parameter("obstacle_metrics_topic").as_string();
    performance_topic_ = get_parameter("performance_topic").as_string();
    metric_frame_id_ = get_parameter("metric_frame_id").as_string();
    class_mask_topic_ = get_parameter("class_mask_topic").as_string();

    camera_metric_calibration_validated_ =
      get_parameter("camera_metric_calibration_validated").as_bool();
    ground_calibration_width_ = get_parameter("ground_calibration_width").as_int();
    ground_calibration_height_ = get_parameter("ground_calibration_height").as_int();
    ground_src_points_ = get_parameter("ground_src_points").as_double_array();
    ground_dst_points_ = get_parameter("ground_dst_points").as_double_array();
    ground_canvas_width_ = get_parameter("ground_canvas_width").as_int();
    ground_canvas_height_ = get_parameter("ground_canvas_height").as_int();
    ground_origin_x_ = static_cast<float>(get_parameter("ground_origin_x_px").as_double());
    ground_origin_y_ = static_cast<float>(get_parameter("ground_origin_y_px").as_double());
    ground_scale_x_ = static_cast<float>(get_parameter("ground_meters_per_pixel_x").as_double());
    ground_scale_y_ = static_cast<float>(get_parameter("ground_meters_per_pixel_y").as_double());
    metric_forward_offset_ = static_cast<float>(get_parameter("metric_forward_offset_m").as_double());
    metric_lateral_offset_ = static_cast<float>(get_parameter("metric_lateral_offset_m").as_double());
    metric_min_forward_ = static_cast<float>(get_parameter("metric_minimum_forward_m").as_double());
    metric_max_forward_ = static_cast<float>(get_parameter("metric_maximum_forward_m").as_double());
    metric_max_abs_left_ = static_cast<float>(get_parameter("metric_maximum_abs_left_m").as_double());
    lane_vehicle_width_ = static_cast<float>(get_parameter("lane_vehicle_width_m").as_double());
    lane_min_width_ = static_cast<float>(get_parameter("lane_minimum_road_width_m").as_double());
    lane_max_width_ = static_cast<float>(get_parameter("lane_maximum_road_width_m").as_double());
    lane_near_ = static_cast<float>(get_parameter("lane_lookahead_near_m").as_double());
    lane_far_ = static_cast<float>(get_parameter("lane_lookahead_far_m").as_double());
    lane_control_lookahead_ = static_cast<float>(get_parameter("lane_control_lookahead_m").as_double());
    lane_sample_rows_ = get_parameter("lane_sample_rows").as_int();
    lane_minimum_rows_ = get_parameter("lane_minimum_valid_rows").as_int();
    lane_min_center_offset_ = static_cast<float>(get_parameter("lane_minimum_center_offset_m").as_double());
    lane_ema_alpha_ = static_cast<float>(get_parameter("lane_ema_alpha").as_double());
    drivable_space_near_ = static_cast<float>(get_parameter("drivable_space_near_m").as_double());
    drivable_space_far_ = static_cast<float>(get_parameter("drivable_space_far_m").as_double());
    drivable_space_max_abs_left_ = static_cast<float>(get_parameter("drivable_space_max_abs_left_m").as_double());
    drivable_space_sample_rows_ = get_parameter("drivable_space_sample_rows").as_int();
    drivable_space_min_pixels_per_row_ = get_parameter("drivable_space_min_pixels_per_row").as_int();
    drivable_space_pixel_stride_px_ = get_parameter("drivable_space_pixel_stride_px").as_int();
    drivable_boundary_guard_offset_m_ = get_parameter("drivable_boundary_guard_offset_m").as_double();
    drivable_boundary_z_m_ = get_parameter("drivable_boundary_z_m").as_double();
    drivable_lane_constraint_min_confidence_ =
      get_parameter("drivable_lane_constraint_min_confidence").as_double();
    camera_fx_ = get_parameter("camera_fx").as_double();
    camera_fy_ = get_parameter("camera_fy").as_double();
    camera_cx_ = get_parameter("camera_cx").as_double();
    camera_cy_ = get_parameter("camera_cy").as_double();
    camera_distortion_ = get_parameter("camera_distortion").as_double_array();

    integrated_runtime_enabled_ = get_parameter("enable_integrated_perception_runtime").as_bool();
    lane_safety_enabled_ = get_parameter("lane_safety_enabled").as_bool();
    lane_state_topic_ = get_parameter("lane_state_topic").as_string();
    lane_centerline_topic_ = get_parameter("lane_centerline_topic").as_string();
    lane_safety_markers_topic_ = get_parameter("lane_safety_markers_topic").as_string();
    nominal_road_width_m_ = get_parameter("nominal_road_width_m").as_double();
    lane_thresholds_.warning_clearance_m = get_parameter("edge_warning_clearance_m").as_double();
    lane_thresholds_.critical_clearance_m = get_parameter("edge_critical_clearance_m").as_double();
    lane_thresholds_.release_clearance_m = get_parameter("edge_release_clearance_m").as_double();
    lane_thresholds_.center_deadband_m = get_parameter("center_deadband_m").as_double();
    minimum_metric_confidence_ = get_parameter("minimum_metric_confidence").as_double();
    state_confirm_frames_ = get_parameter("state_confirm_frames").as_int();
    release_confirm_frames_ = get_parameter("release_confirm_frames").as_int();
    lost_confirm_frames_ = get_parameter("lost_confirm_frames").as_int();

    object_points_topic_ = get_parameter("object_points_topic").as_string();
    object_clearing_points_topic_ = get_parameter("object_clearing_points_topic").as_string();
    obstacle_markers_topic_ = get_parameter("obstacle_markers_topic").as_string();
    perception_obstacle_metrics_topic_ = get_parameter("perception_obstacle_metrics_topic").as_string();
    raw_detection_summary_topic_ = get_parameter("raw_detection_summary_topic").as_string();
    raw_detection_markers_topic_ = get_parameter("raw_detection_markers_topic").as_string();
    raw_detection_log_period_sec_ = std::clamp(
      get_parameter("raw_detection_log_period_sec").as_double(), 0.10, 10.0);
    minimum_obstacle_confidence_ = get_parameter("minimum_obstacle_confidence").as_double();
    accept_all_detected_classes_as_obstacles_ =
      get_parameter("accept_all_detected_classes_as_obstacles").as_bool();
    safety_obstacle_class_ids_ = get_parameter("safety_obstacle_class_ids").as_integer_array();
    require_drivable_contact_ = get_parameter("require_drivable_contact").as_bool();
    drivable_contact_radius_px_ = get_parameter("drivable_contact_radius_px").as_int();
    drivable_contact_vertical_tolerance_px_ =
      get_parameter("drivable_contact_vertical_tolerance_px").as_int();
    drivable_contact_min_samples_ = get_parameter("drivable_contact_min_samples").as_int();
    drivable_contact_min_fraction_ =
      static_cast<float>(get_parameter("drivable_contact_min_fraction").as_double());
    points_per_box_ = get_parameter("points_per_box").as_int();
    obstacle_cloud_max_publish_rate_hz_ = get_parameter("obstacle_cloud_max_publish_rate_hz").as_double();
    clearing_fov_rad_ = get_parameter("clearing_fov_deg").as_double() * M_PI / 180.0;
    clearing_range_m_ = get_parameter("clearing_range_m").as_double();
    clearing_ray_count_ = get_parameter("clearing_ray_count").as_int();
    track_match_distance_m_ = get_parameter("track_match_distance_m").as_double();
    track_ema_alpha_ = get_parameter("track_ema_alpha").as_double();
    track_confirm_hits_ = get_parameter("track_confirm_hits").as_int();
    track_max_missed_frames_ = get_parameter("track_max_missed_frames").as_int();
    clearing_obstacle_margin_m_ = get_parameter("clearing_obstacle_margin_m").as_double();

    control_mode_ = get_parameter("control_mode").as_string();
    nav_cmd_topic_ = get_parameter("nav_cmd_topic").as_string();
    safe_cmd_topic_ = get_parameter("safe_cmd_topic").as_string();
    control_state_topic_ = get_parameter("control_state_topic").as_string();
    control_markers_topic_ = get_parameter("control_markers_topic").as_string();
    control_rate_hz_ = get_parameter("control_rate_hz").as_double();
    cmd_timeout_sec_ = get_parameter("cmd_timeout_sec").as_double();
    lane_state_timeout_sec_ = get_parameter("lane_state_timeout_sec").as_double();
    obstacle_state_timeout_sec_ = get_parameter("obstacle_state_timeout_sec").as_double();
    mixer_config_.wheelbase_m = get_parameter("wheelbase_m").as_double();
    mixer_config_.maximum_steering_angle_rad = get_parameter("maximum_steering_angle_rad").as_double();
    mixer_config_.recenter_speed_mps = get_parameter("recenter_speed_mps").as_double();
    mixer_config_.critical_recenter_speed_mps = get_parameter("critical_recenter_speed_mps").as_double();
    mixer_config_.center_gain = get_parameter("center_gain").as_double();
    mixer_config_.heading_gain = get_parameter("heading_gain").as_double();
    mixer_config_.lane_blend_gain = get_parameter("lane_blend_gain").as_double();
    mixer_config_.minimum_speed_for_yaw_limit_mps = get_parameter("minimum_speed_for_yaw_limit_mps").as_double();
    obstacle_gate_config_.vehicle_width_m = lane_vehicle_width_;
    obstacle_gate_config_.forward_min_m = get_parameter("obstacle_forward_min_m").as_double();
    obstacle_gate_config_.forward_max_m = get_parameter("obstacle_forward_max_m").as_double();
    obstacle_gate_config_.lateral_margin_m = get_parameter("obstacle_lateral_margin_m").as_double();
    obstacle_gate_config_.minimum_obstacle_width_m = get_parameter("minimum_obstacle_width_m").as_double();
    blocked_confirm_frames_ = get_parameter("blocked_confirm_frames").as_int();
    clear_confirm_frames_ = get_parameter("clear_confirm_frames").as_int();

    camera_health_enabled_ = get_parameter("camera_health_enabled").as_bool();
    camera_health_topic_ = get_parameter("camera_health_topic").as_string();
    camera_health_state_topic_ = get_parameter("camera_health_state_topic").as_string();
    camera_health_sample_stride_px_ = get_parameter("camera_health_sample_stride_px").as_int();
    camera_health_dark_luma_ = get_parameter("camera_health_dark_luma").as_int();
    camera_health_bright_luma_ = get_parameter("camera_health_bright_luma").as_int();
    camera_health_extreme_fraction_ = get_parameter("camera_health_extreme_fraction").as_double();
    camera_health_min_stddev_ = get_parameter("camera_health_min_stddev").as_double();
    camera_health_min_gradient_ = get_parameter("camera_health_min_gradient").as_double();
    camera_health_bad_confirm_frames_ = get_parameter("camera_health_bad_confirm_frames").as_int();
    camera_health_good_confirm_frames_ = get_parameter("camera_health_good_confirm_frames").as_int();

    near_field_emergency_enabled_ = get_parameter("near_field_emergency_enabled").as_bool();
    emergency_stop_topic_ = get_parameter("emergency_stop_topic").as_string();
    near_field_state_topic_ = get_parameter("near_field_state_topic").as_string();
    near_field_min_confidence_ = get_parameter("near_field_min_confidence").as_double();
    near_field_center_corridor_fraction_ = get_parameter("near_field_center_corridor_fraction").as_double();
    near_field_min_bbox_height_fraction_ = get_parameter("near_field_min_bbox_height_fraction").as_double();
    near_field_drivable_guard_enabled_ = get_parameter("near_field_drivable_guard_enabled").as_bool();
    near_field_bottom_roi_fraction_ = get_parameter("near_field_bottom_roi_fraction").as_double();
    near_field_min_drivable_fraction_ = get_parameter("near_field_min_drivable_fraction").as_double();
    near_field_confirm_frames_ = get_parameter("near_field_confirm_frames").as_int();
    near_field_release_frames_ = get_parameter("near_field_release_frames").as_int();

    engine_path_ = resolveEnginePath(engine_path_);
    if (width_ <= 0 || height_ <= 0 || fps_ <= 0) throw std::runtime_error("Invalid RGB size/fps parameters");
    if (opencv_window_width_ <= 0) opencv_window_width_ = width_;
    if (opencv_window_height_ <= 0) opencv_window_height_ = height_;
    if (max_candidates_ < 1 || max_detections_ < 1) throw std::runtime_error("max_candidates/max_detections must be positive");
    confidence_threshold_ = std::clamp(confidence_threshold_, 0.01F, 0.99F);
    iou_threshold_ = std::clamp(iou_threshold_, 0.01F, 0.99F);
    lane_threshold_ = std::clamp(lane_threshold_, 0.01F, 0.99F);
    overlay_alpha_ = std::clamp(overlay_alpha_, 0.0F, 1.0F);
    box_thickness_ = std::max(1, box_thickness_);
    rviz_publish_buffer_count_ = std::clamp(rviz_publish_buffer_count_, 2, 6);
    rviz_max_publish_rate_hz_ = std::max(0.0, rviz_max_publish_rate_hz_);
    if (publisher_reliability_ != "reliable" && publisher_reliability_ != "best_effort") {
      throw std::runtime_error("publisher_reliability must be reliable or best_effort");
    }
    if (publish_lane_metrics_ || publish_obstacle_metrics_ || integrated_runtime_enabled_) {
      if (ground_calibration_width_ <= 1 || ground_calibration_height_ <= 1 ||
          ground_canvas_width_ <= 1 || ground_canvas_height_ <= 1 ||
          ground_src_points_.size() != 8U || ground_dst_points_.size() != 8U) {
        throw std::runtime_error("Ground-plane calibration must contain four valid source/destination points");
      }
      if (!(ground_scale_x_ > 0.0F) || !(ground_scale_y_ > 0.0F) ||
          metric_min_forward_ < 0.0F || metric_max_forward_ <= metric_min_forward_ ||
          metric_max_abs_left_ <= 0.0F) {
        throw std::runtime_error("Invalid metric ground-plane scale/range parameters");
      }
    }
    if (publish_lane_metrics_ || integrated_runtime_enabled_) {
      if (lane_vehicle_width_ <= 0.0F || lane_min_width_ <= lane_vehicle_width_ ||
          lane_max_width_ <= lane_min_width_ || lane_near_ < 0.0F || lane_far_ <= lane_near_ ||
          lane_control_lookahead_ < lane_near_ || lane_control_lookahead_ > lane_far_ ||
          lane_sample_rows_ < 4 || lane_minimum_rows_ < 2 || lane_minimum_rows_ > lane_sample_rows_) {
        throw std::runtime_error("Invalid lane metric geometry parameters");
      }
      lane_ema_alpha_ = std::clamp(lane_ema_alpha_, 0.01F, 1.0F);
      lane_min_center_offset_ = std::max(0.0F, lane_min_center_offset_);
    }

    if (integrated_runtime_enabled_) {
      lane_thresholds_.validate();
      mixer_config_.validate();
      obstacle_gate_config_.validate();
      minimum_metric_confidence_ = std::clamp(minimum_metric_confidence_, 0.0, 1.0);
      minimum_obstacle_confidence_ = std::clamp(minimum_obstacle_confidence_, 0.0, 1.0);
      drivable_contact_radius_px_ = std::clamp(drivable_contact_radius_px_, 0, 128);
      drivable_contact_vertical_tolerance_px_ =
        std::clamp(drivable_contact_vertical_tolerance_px_, 0, 128);
      drivable_contact_min_samples_ = std::clamp(drivable_contact_min_samples_, 1, 15);
      drivable_contact_min_fraction_ = std::clamp(drivable_contact_min_fraction_, 0.0F, 1.0F);
      points_per_box_ = std::max(1, points_per_box_);
      obstacle_cloud_max_publish_rate_hz_ = std::clamp(obstacle_cloud_max_publish_rate_hz_, 1.0, 30.0);
      clearing_ray_count_ = std::max(3, clearing_ray_count_);
      clearing_range_m_ = std::max(0.1, clearing_range_m_);
      track_match_distance_m_ = std::max(0.05, track_match_distance_m_);
      track_ema_alpha_ = std::clamp(track_ema_alpha_, 0.01, 1.0);
      track_confirm_hits_ = std::clamp(track_confirm_hits_, 1, 30);
      track_max_missed_frames_ = std::max(0, track_max_missed_frames_);
      drivable_space_sample_rows_ = std::clamp(drivable_space_sample_rows_, 8, 160);
      drivable_space_min_pixels_per_row_ = std::clamp(drivable_space_min_pixels_per_row_, 1, 10000);
      drivable_space_pixel_stride_px_ = std::clamp(drivable_space_pixel_stride_px_, 1, 16);
      drivable_space_near_ = std::max(0.05F, drivable_space_near_);
      drivable_space_far_ = std::max(drivable_space_near_ + 0.25F, drivable_space_far_);
      drivable_space_max_abs_left_ = std::max(0.50F, drivable_space_max_abs_left_);
      drivable_boundary_guard_offset_m_ = std::clamp(drivable_boundary_guard_offset_m_, 0.0, 0.50);
      drivable_boundary_z_m_ = std::clamp(drivable_boundary_z_m_, 0.0, 1.5);
      drivable_lane_constraint_min_confidence_ =
        std::clamp(drivable_lane_constraint_min_confidence_, 0.0, 1.0);
      clearing_obstacle_margin_m_ = std::max(0.0, clearing_obstacle_margin_m_);
      state_confirm_frames_ = std::max(1, state_confirm_frames_);
      release_confirm_frames_ = std::max(1, release_confirm_frames_);
      lost_confirm_frames_ = std::max(1, lost_confirm_frames_);
      blocked_confirm_frames_ = std::max(1, blocked_confirm_frames_);
      clear_confirm_frames_ = std::max(1, clear_confirm_frames_);
      camera_health_sample_stride_px_ = std::clamp(camera_health_sample_stride_px_, 4, 64);
      camera_health_dark_luma_ = std::clamp(camera_health_dark_luma_, 0, 254);
      camera_health_bright_luma_ = std::clamp(camera_health_bright_luma_, 1, 255);
      if (camera_health_bright_luma_ <= camera_health_dark_luma_) {
        throw std::runtime_error("camera_health_bright_luma harus > camera_health_dark_luma");
      }
      camera_health_extreme_fraction_ = std::clamp(camera_health_extreme_fraction_, 0.50, 1.0);
      camera_health_min_stddev_ = std::max(0.0, camera_health_min_stddev_);
      camera_health_min_gradient_ = std::max(0.0, camera_health_min_gradient_);
      camera_health_bad_confirm_frames_ = std::max(1, camera_health_bad_confirm_frames_);
      camera_health_good_confirm_frames_ = std::max(1, camera_health_good_confirm_frames_);
      near_field_min_confidence_ = std::clamp(near_field_min_confidence_, 0.01, 1.0);
      near_field_center_corridor_fraction_ = std::clamp(near_field_center_corridor_fraction_, 0.10, 1.0);
      near_field_min_bbox_height_fraction_ = std::clamp(near_field_min_bbox_height_fraction_, 0.05, 1.0);
      near_field_bottom_roi_fraction_ = std::clamp(near_field_bottom_roi_fraction_, 0.05, 0.80);
      near_field_min_drivable_fraction_ = std::clamp(near_field_min_drivable_fraction_, 0.0, 1.0);
      near_field_confirm_frames_ = std::max(1, near_field_confirm_frames_);
      near_field_release_frames_ = std::max(1, near_field_release_frames_);
      control_rate_hz_ = std::max(1.0, control_rate_hz_);
      cmd_timeout_sec_ = std::max(0.05, cmd_timeout_sec_);
      lane_state_timeout_sec_ = std::max(0.05, lane_state_timeout_sec_);
      obstacle_state_timeout_sec_ = std::max(0.05, obstacle_state_timeout_sec_);
      if (control_mode_ != "active" && control_mode_ != "monitor") {
        throw std::runtime_error("control_mode harus 'active' atau 'monitor'");
      }
      if (nominal_road_width_m_ <= lane_vehicle_width_) {
        throw std::runtime_error("nominal_road_width_m harus lebih besar dari vehicle width");
      }
    }

    // Semua publisher yang aktif wajib memiliki nama topic. Validasi ini membuat
    // kesalahan konfigurasi terdeteksi di terminal, bukan muncul sebagai topic
    // kosong di RViz.
    const auto wajib_terisi = [](const std::string &nama, const char *parameter) {
      if (nama.empty()) {
        throw std::runtime_error(std::string("Parameter topic tidak boleh kosong: ") + parameter);
      }
    };
    wajib_terisi(frame_id_, "frame_id");
    if (publish_annotated_) wajib_terisi(annotated_topic_, "annotated_topic");
    if (publish_raw_) wajib_terisi(raw_topic_, "raw_topic");
    wajib_terisi(camera_info_topic_, "camera_info_topic");
    if (publish_detections_) wajib_terisi(detections_topic_, "detections_topic");
    if (publish_drivable_mask_) wajib_terisi(drivable_mask_topic_, "drivable_mask_topic");
    if (publish_lane_mask_) wajib_terisi(lane_mask_topic_, "lane_mask_topic");
    if (publish_lane_metrics_) wajib_terisi(lane_metrics_topic_, "lane_metrics_topic");
    if (publish_obstacle_metrics_) wajib_terisi(obstacle_metrics_topic_, "obstacle_metrics_topic");
    wajib_terisi(performance_topic_, "performance_topic");
    if (integrated_runtime_enabled_) {
      wajib_terisi(raw_detection_summary_topic_, "raw_detection_summary_topic");
      wajib_terisi(raw_detection_markers_topic_, "raw_detection_markers_topic");
      if (camera_health_enabled_) {
        wajib_terisi(camera_health_topic_, "camera_health_topic");
        wajib_terisi(camera_health_state_topic_, "camera_health_state_topic");
      }
      if (near_field_emergency_enabled_) {
        wajib_terisi(emergency_stop_topic_, "emergency_stop_topic");
        wajib_terisi(near_field_state_topic_, "near_field_state_topic");
      }
    }
    wajib_terisi(metric_frame_id_, "metric_frame_id");
    if (publish_class_mask_) wajib_terisi(class_mask_topic_, "class_mask_topic");

    if (ground_calibration_width_ <= 0 || ground_calibration_height_ <= 0 ||
        ground_canvas_width_ <= 0 || ground_canvas_height_ <= 0 ||
        ground_src_points_.size() != 8 || ground_dst_points_.size() != 8 ||
        ground_scale_x_ <= 0.0F || ground_scale_y_ <= 0.0F) {
      throw std::runtime_error("Ground-plane calibration parameters are invalid");
    }
    if (!(0.0F < lane_near_ && lane_near_ < lane_far_) ||
        !(0.0F < lane_vehicle_width_ && lane_vehicle_width_ < lane_min_width_ && lane_min_width_ < lane_max_width_) ||
        lane_sample_rows_ < 8 || lane_minimum_rows_ < 4 || lane_minimum_rows_ > lane_sample_rows_) {
      throw std::runtime_error("Lane metric parameters are invalid");
    }
    lane_ema_alpha_ = std::clamp(lane_ema_alpha_, 0.01F, 1.0F);

    // Tidak perlu membuat worker/pinned ring-buffer RViz apabila output annotated
    // dimatikan. Ini menghindari alokasi RAM yang tidak digunakan.
    async_rviz_publish_ = async_rviz_publish_ && publish_annotated_;

    // Jendela OpenCV dan tulisan label berbasis CPU memerlukan frame pada thread
    // pemrosesan utama. Karena itu publisher asinkron khusus RViz hanya digunakan
    // ketika overlay final benar-benar dibuat sepenuhnya oleh GPU.
    if (show_opencv_cuda_window_ || draw_text_labels_cpu_) {
      async_rviz_publish_ = false;
    }
  }

  // Memilih GPU dan membuat CUDA stream non-blocking berprioritas tinggi.
  // Fungsi: Memilih GPU CUDA, membuat stream prioritas, dan memverifikasi kemampuan device.
  void initializeCuda() {
    CUDA_CHECK(cudaSetDevice(gpu_device_));
    int least_priority = 0;
    int greatest_priority = 0;
    CUDA_CHECK(cudaDeviceGetStreamPriorityRange(&least_priority, &greatest_priority));
    CUDA_CHECK(cudaStreamCreateWithPriority(&stream_, cudaStreamNonBlocking, greatest_priority));
    cudaDeviceProp properties{};
    CUDA_CHECK(cudaGetDeviceProperties(&properties, gpu_device_));
    RCLCPP_INFO(
      get_logger(),
      "GPU CUDA %d: %s, compute capability %d.%d, prioritas stream %d..%d",
      gpu_device_, properties.name, properties.major, properties.minor,
      greatest_priority, least_priority);
  }

  // Ground-plane transform is prepared once after the negotiated V4L2 size is
  // known. V19 menjalankan autonomous pada 1280x720 sesuai preprocessing resmi
  // YOLOPv2. Scaling tetap tersedia hanya untuk launch/debug non-autonomous;
  // akurasi metrik final tetap membutuhkan kalibrasi ground-plane fisik pada
  // mode kamera 1280x720 yang benar-benar dipakai. The 3x3 transform is copied to constant-size
  // VRAM and used by CUDA kernels for lane and obstacle metric projection.
  // Fungsi: Menyiapkan homography pixel-ke-ground di VRAM untuk obstacle dan lane metric.
  void initializeGroundProjection() {
    if (!publish_lane_metrics_ && !publish_obstacle_metrics_ && !integrated_runtime_enabled_) return;
    const float sx = static_cast<float>(width_) / static_cast<float>(ground_calibration_width_);
    const float sy = static_cast<float>(height_) / static_cast<float>(ground_calibration_height_);
    std::vector<cv::Point2f> src(4);
    std::vector<cv::Point2f> dst(4);
    for (int i = 0; i < 4; ++i) {
      src[static_cast<size_t>(i)] = cv::Point2f(
        static_cast<float>(ground_src_points_[static_cast<size_t>(2 * i)]) * sx,
        static_cast<float>(ground_src_points_[static_cast<size_t>(2 * i + 1)]) * sy);
      dst[static_cast<size_t>(i)] = cv::Point2f(
        static_cast<float>(ground_dst_points_[static_cast<size_t>(2 * i)]),
        static_cast<float>(ground_dst_points_[static_cast<size_t>(2 * i + 1)]));
    }
    cv::Mat homography = cv::getPerspectiveTransform(src, dst);
    if (homography.empty()) throw std::runtime_error("Failed to build ground-plane homography");
    cv::Mat h32;
    homography.convertTo(h32, CV_32F);
    ground_homography_host_.resize(9U);
    std::memcpy(ground_homography_host_.data(), h32.ptr<float>(), 9U * sizeof(float));
    for (float value : ground_homography_host_) {
      if (!std::isfinite(value)) throw std::runtime_error("Non-finite ground-plane homography");
    }
    RCLCPP_INFO(
      get_logger(),
      "GPU ground projection: calibration=%dx%d runtime=%dx%d scale=(%.3f,%.3f), frame=%s",
      ground_calibration_width_, ground_calibration_height_, width_, height_, sx, sy,
      metric_frame_id_.c_str());
  }

  // Mengalokasikan VRAM, pinned host memory, lalu memuat engine TensorRT.
  // Fungsi: Mengalokasikan VRAM/pinned memory yang dibutuhkan pipeline lalu memuat TensorRT engine.
  void allocateBuffersAndLoadEngine() {
    const size_t image_bytes = static_cast<size_t>(width_) * height_ * 3U;
    const size_t mask_bytes = static_cast<size_t>(width_) * height_;

    CUDA_CHECK(cudaMalloc(&d_bgr_, image_bytes));
    if (!camera_.directCudaMapped() || camera_.pixelFormat() == V4L2_PIX_FMT_MJPEG) {
      // V4L2 may pad rows. Allocate from the driver's sizeimage instead of a
      // tightly-packed width*height assumption.
      CUDA_CHECK(cudaMalloc(&d_capture_staging_, std::max(image_bytes, camera_.sizeImage())));
    }

    // Engine yang disertakan umumnya memakai input statis [1,3,384,640]. Buffer
    // awal dialokasikan sebesar itu, lalu ukuran sebenarnya divalidasi setelah
    // deserialisasi. Jika engine lebih besar, buffer dibuat ulang secara aman.
    CUDA_CHECK(cudaMalloc(&d_input_, 3U * 384U * 640U * sizeof(float)));
    engine_.load(engine_path_, d_input_);
    if (engine_.inputElements() > 3U * 384U * 640U) {
      cudaFree(d_input_);
      d_input_ = nullptr;
      CUDA_CHECK(cudaMalloc(&d_input_, engine_.inputElements() * sizeof(float)));
      engine_.load(engine_path_, d_input_);
    }

    model_width_ = engine_.inputW();
    model_height_ = engine_.inputH();
    gain_ = std::min(static_cast<float>(model_width_) / width_, static_cast<float>(model_height_) / height_);
    pad_x_ = (static_cast<float>(model_width_) - width_ * gain_) * 0.5F;
    pad_y_ = (static_cast<float>(model_height_) - height_ * gain_) * 0.5F;

    // Kontrak preprocessing resmi CAIC-AD/YOLOPv2:
    // source 1280x720 -> letterbox(img_size=640, stride=32) -> tensor 640x384,
    // gain 0.5 dan padding 12 px di atas/bawah.
    if (width_ != 1280 || height_ != 720 || model_width_ != 640 || model_height_ != 384 ||
        std::fabs(gain_ - 0.5F) > 1.0e-4F || std::fabs(pad_x_) > 1.0e-4F ||
        std::fabs(pad_y_ - 12.0F) > 1.0e-3F) {
      std::ostringstream geometry_error;
      geometry_error << "YOLOPv2 official preprocessing geometry mismatch: camera="
                     << width_ << "x" << height_ << " engine=" << model_width_ << "x"
                     << model_height_ << " gain=" << gain_ << " pad=(" << pad_x_ << ","
                     << pad_y_ << "). Wajib camera 1280x720 -> input 640x384, gain=0.5, pad=(0,12).";
      throw std::runtime_error(geometry_error.str());
    }
    RCLCPP_INFO(
      get_logger(),
      "YOLOPv2 official preprocessing VALID: camera=1280x720 -> RGB/255 -> input=640x384 gain=0.500 pad=(0.0,12.0), anchor_source=%s",
      "engine-output");

    const bool segmentation_enabled =
      publish_annotated_ || show_opencv_cuda_window_ || publish_drivable_mask_ ||
      publish_lane_mask_ || publish_class_mask_ || publish_lane_metrics_ || integrated_runtime_enabled_;
    if (segmentation_enabled) {
      CUDA_CHECK(cudaMalloc(&d_drivable_mask_, mask_bytes));
      CUDA_CHECK(cudaMalloc(&d_lane_mask_, mask_bytes));
    }
    if (publish_class_mask_) {
      CUDA_CHECK(cudaMalloc(&d_class_mask_, mask_bytes));
    }
    CUDA_CHECK(cudaMalloc(&d_candidates_, static_cast<size_t>(max_candidates_) * sizeof(DetectionGpu)));
    CUDA_CHECK(cudaMalloc(&d_candidate_count_, sizeof(int)));
    CUDA_CHECK(cudaMalloc(&d_final_detections_, static_cast<size_t>(max_detections_) * sizeof(DetectionGpu)));
    CUDA_CHECK(cudaMalloc(&d_final_count_, sizeof(int)));
    if (integrated_runtime_enabled_ &&
        (camera_health_enabled_ || (near_field_emergency_enabled_ && near_field_drivable_guard_enabled_))) {
      CUDA_CHECK(cudaMalloc(
        &d_camera_health_stats_,
        static_cast<size_t>(CAMERA_HEALTH_STAT_COUNT) * sizeof(unsigned long long)));
      CUDA_CHECK(cudaHostAlloc(
        reinterpret_cast<void **>(&h_camera_health_stats_),
        static_cast<size_t>(CAMERA_HEALTH_STAT_COUNT) * sizeof(unsigned long long),
        cudaHostAllocPortable));
      std::memset(
        h_camera_health_stats_, 0,
        static_cast<size_t>(CAMERA_HEALTH_STAT_COUNT) * sizeof(unsigned long long));
    }

    if ((publish_lane_metrics_ || publish_obstacle_metrics_ || integrated_runtime_enabled_) &&
        !ground_homography_host_.empty()) {
      CUDA_CHECK(cudaMalloc(&d_ground_homography_, 9U * sizeof(float)));
      CUDA_CHECK(cudaMemcpyAsync(
        d_ground_homography_, ground_homography_host_.data(), 9U * sizeof(float),
        cudaMemcpyHostToDevice, stream_));
    }
    if (publish_obstacle_metrics_ || integrated_runtime_enabled_) {
      CUDA_CHECK(cudaMalloc(
        &d_metric_detections_, static_cast<size_t>(max_detections_) * sizeof(MetricDetectionGpu)));
      h_metric_detections_.resize(static_cast<size_t>(max_detections_));
    }
    if (publish_lane_metrics_ || integrated_runtime_enabled_) {
      CUDA_CHECK(cudaMalloc(&d_lane_left_mm_, static_cast<size_t>(lane_sample_rows_) * sizeof(int)));
      CUDA_CHECK(cudaMalloc(&d_lane_right_mm_, static_cast<size_t>(lane_sample_rows_) * sizeof(int)));
      h_lane_left_mm_.resize(static_cast<size_t>(lane_sample_rows_));
      h_lane_right_mm_.resize(static_cast<size_t>(lane_sample_rows_));
    }
    if (integrated_runtime_enabled_) {
      CUDA_CHECK(cudaMalloc(&d_drivable_left_mm_, static_cast<size_t>(drivable_space_sample_rows_) * sizeof(int)));
      CUDA_CHECK(cudaMalloc(&d_drivable_right_mm_, static_cast<size_t>(drivable_space_sample_rows_) * sizeof(int)));
      CUDA_CHECK(cudaMalloc(&d_drivable_count_, static_cast<size_t>(drivable_space_sample_rows_) * sizeof(int)));
      h_drivable_left_mm_.resize(static_cast<size_t>(drivable_space_sample_rows_));
      h_drivable_right_mm_.resize(static_cast<size_t>(drivable_space_sample_rows_));
      h_drivable_count_.resize(static_cast<size_t>(drivable_space_sample_rows_));
    }

    if (async_rviz_publish_) {
      rviz_frame_slots_.resize(static_cast<size_t>(rviz_publish_buffer_count_));
      for (auto &slot : rviz_frame_slots_) {
        CUDA_CHECK(cudaHostAlloc(
          reinterpret_cast<void **>(&slot.host), image_bytes,
          cudaHostAllocPortable));
      }
    } else if (publish_annotated_ || show_opencv_cuda_window_ || draw_text_labels_cpu_) {
      // Buffer ini dipakai sebagai fallback apabila CUDA -> OpenGL langsung
      // tidak tersedia, serta untuk publisher sinkron/label CPU.
      CUDA_CHECK(cudaHostAlloc(
        reinterpret_cast<void **>(&h_annotated_), image_bytes,
        cudaHostAllocPortable));
    }
    if (publish_raw_) CUDA_CHECK(cudaHostAlloc(reinterpret_cast<void **>(&h_raw_), image_bytes, cudaHostAllocPortable));
    // Host mask/detection buffers dialokasikan secara lazy ketika subscriber
    // pertama muncul. Ini menghemat pinned RAM pada bringup yang hanya memakai
    // annotated image.

    RCLCPP_INFO(get_logger(), "TensorRT: %s", engine_.describe().c_str());
    RCLCPP_INFO(
      get_logger(), "RViz output: %s, pinned buffers=%d, QoS=%s/depth1, policy=latest-frame-only",
      async_rviz_publish_ ? "asynchronous" : "synchronous", rviz_publish_buffer_count_,
      publisher_reliability_.c_str());
    logCudaMemory("setelah alokasi pipeline");
  }

  // Membuat publisher hanya untuk keluaran yang diaktifkan. Dengan konfigurasi
  // launch RViz, topic gambar yang diiklankan adalah annotated, drivable mask,
  // dan lane mask. Topic lain tidak memenuhi ROS graph secara tidak perlu.
  // Fungsi: Membuat publisher ROS dengan QoS yang sesuai data sensor/RViz dan opsi publish yang aktif.
  void createPublishers() {
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.durability_volatile();
    if (publisher_reliability_ == "reliable") {
      qos.reliable();
    } else {
      qos.best_effort();
    }

    if (publish_annotated_) {
      annotated_pub_ = create_publisher<sensor_msgs::msg::Image>(annotated_topic_, qos);
    }
    if (publish_raw_) {
      raw_pub_ = create_publisher<sensor_msgs::msg::Image>(raw_topic_, qos);
    }

    // CameraInfo tetap dibuat, tetapi hanya dipublikasikan ketika ada subscriber.
    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, qos);

    if (publish_detections_) {
      detections_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(detections_topic_, qos);
    }
    if (publish_drivable_mask_) {
      drivable_pub_ = create_publisher<sensor_msgs::msg::Image>(drivable_mask_topic_, qos);
    }
    if (publish_lane_mask_) {
      lane_pub_ = create_publisher<sensor_msgs::msg::Image>(lane_mask_topic_, qos);
    }
    if (publish_lane_metrics_) {
      lane_metrics_pub_ = create_publisher<std_msgs::msg::String>(lane_metrics_topic_, qos);
    }
    if (publish_obstacle_metrics_) {
      obstacle_metrics_pub_ = create_publisher<std_msgs::msg::String>(obstacle_metrics_topic_, qos);
    }
    performance_pub_ = create_publisher<std_msgs::msg::String>(
      performance_topic_, rclcpp::QoS(rclcpp::KeepLast(5)).reliable());
    if (publish_class_mask_) {
      class_mask_pub_ = create_publisher<sensor_msgs::msg::Image>(class_mask_topic_, qos);
    }

    if (publish_annotated_) RCLCPP_INFO(get_logger(), "Topic annotated       : %s", annotated_topic_.c_str());
    if (publish_drivable_mask_) RCLCPP_INFO(get_logger(), "Topic drivable mask   : %s", drivable_mask_topic_.c_str());
    if (publish_lane_mask_) RCLCPP_INFO(get_logger(), "Topic lane mask       : %s", lane_mask_topic_.c_str());
    if (publish_detections_) RCLCPP_INFO(get_logger(), "Topic raw detections  : %s", detections_topic_.c_str());
    if (publish_lane_metrics_) RCLCPP_INFO(get_logger(), "Topic lane metrics    : %s", lane_metrics_topic_.c_str());
    if (publish_obstacle_metrics_) RCLCPP_INFO(get_logger(), "Topic obstacle metrics: %s", obstacle_metrics_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Topic performance     : %s", performance_topic_.c_str());
  }

  // Membuat seluruh antarmuka safety/control C++ yang dahulu tersebar pada
  // empat node Python. Jalur ini tidak subscribe kembali ke JSON YOLOP; data
  // lane/obstacle diteruskan langsung dari buffer hasil CUDA di proses ini.
  // Fungsi: Membuat publisher/subscriber/timer C++ pengganti lane safety, obstacle projector, dan control mixer Python.
  void createIntegratedPerceptionRuntime() {
    if (!integrated_runtime_enabled_) return;

    lane_state_filter_ = std::make_unique<safety::ConfirmedState>(
      safety::LANE_LOST, state_confirm_frames_, release_confirm_frames_, lost_confirm_frames_);

    lane_state_pub_ = create_publisher<std_msgs::msg::String>(lane_state_topic_, 10);
    lane_centerline_pub_ = create_publisher<nav_msgs::msg::Path>(lane_centerline_topic_, 10);
    lane_safety_markers_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(lane_safety_markers_topic_, 10);

    // Costmap/collision monitor hanya membutuhkan sample obstacle terbaru.
    // Depth=1 mencegah backlog PointCloud2 lama saat lifecycle costmap baru aktif;
    // ini mengurangi latency, RAM DDS, dan stale TF message-filter drop startup.
    // V17: RELIABLE depth=1 tetap low-latency di localhost dan kompatibel
    // dengan subscriber BEST_EFFORT, sekaligus memenuhi RViz RELIABLE sehingga
    // tidak ada lagi incompatible RELIABILITY_QOS_POLICY.
    auto obstacle_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    object_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      object_points_topic_, obstacle_qos);
    object_clearing_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      object_clearing_points_topic_, obstacle_qos);

    // Jangan kirim PointCloud obstacle sebelum local costmap benar-benar aktif
    // dan sudah mem-publish grid pertamanya. Ini mencegah TF MessageFilter
    // menyimpan/membuang cloud startup dengan timestamp lebih tua dari cache TF.
    auto local_costmap_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    local_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/local_costmap/costmap", local_costmap_qos,
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr) {
        if (!local_costmap_ready_.exchange(true, std::memory_order_acq_rel)) {
          RCLCPP_INFO(
            get_logger(),
            "Local costmap ready; obstacle PointCloud gate dibuka.");
        }
      });

    obstacle_markers_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(obstacle_markers_topic_, 10);
    perception_obstacle_metrics_pub_ =
      create_publisher<std_msgs::msg::String>(perception_obstacle_metrics_topic_, 10);
    raw_detection_summary_pub_ = create_publisher<std_msgs::msg::String>(
      raw_detection_summary_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    raw_detection_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      raw_detection_markers_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

    const auto safety_state_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    if (camera_health_enabled_) {
      camera_health_pub_ = create_publisher<std_msgs::msg::Bool>(
        camera_health_topic_, safety_state_qos);
      camera_health_state_pub_ = create_publisher<std_msgs::msg::String>(
        camera_health_state_topic_, safety_state_qos);
      publishCameraHealthState(false, "STARTUP", 0.0, 0.0, 0.0, 0.0);
    }
    if (near_field_emergency_enabled_) {
      emergency_stop_pub_ = create_publisher<std_msgs::msg::Bool>(
        emergency_stop_topic_, safety_state_qos);
      near_field_state_pub_ = create_publisher<std_msgs::msg::String>(
        near_field_state_topic_, safety_state_qos);
      publishNearFieldState(false, "STARTUP", -1, 0.0, 1.0);
    }

    drivable_space_pub_ = create_publisher<std_msgs::msg::String>(
      drivable_space_topic_, safety_state_qos);
    drivable_boundary_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      drivable_boundary_points_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

    safe_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(safe_cmd_topic_, 10);
    control_state_pub_ = create_publisher<std_msgs::msg::String>(control_state_topic_, 10);
    control_markers_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(control_markers_topic_, 10);
    nav_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      nav_cmd_topic_, 10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(perception_state_mutex_);
        last_nav_cmd_ = *message;
        last_nav_cmd_time_ = std::chrono::steady_clock::now();
        have_nav_cmd_ = true;
      });

    const auto timer_period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&AstraYolopGpuNode::controlTick, this));

    RCLCPP_INFO(
      get_logger(),
      "Integrated perception C++ aktif: GPU metrics -> lane safety + obstacle tracking + mixer; mode=%s, output=%s",
      control_mode_.c_str(), safe_cmd_topic_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "YOLOPv2 V19 official pipeline: raw_cls dipertahankan apa adanya; preprocessing 1280x720 resmi dan anchor-grid engine dipakai untuk decode");
    if (!accept_all_detected_classes_as_obstacles_) {
      std::ostringstream ids;
      for (size_t i = 0; i < safety_obstacle_class_ids_.size(); ++i) {
        if (i) ids << ',';
        ids << safety_obstacle_class_ids_[i];
      }
      RCLCPP_WARN(
        get_logger(),
        "Safety class allow-list aktif raw_cls=[%s]. Validasi person/car/motorcycle pada /perception/raw_detections sebelum autonomous penuh.",
        ids.str().c_str());
    }
  }

  // Membuat PointCloud2 XYZI dengan layout 4 float32 (x,y,z,intensity).
  // Format ini sama dengan adapter Python lama dan langsung diterima Collision Monitor.
  sensor_msgs::msg::PointCloud2 makePointCloud(
    const rclcpp::Time &stamp,
    const std::vector<std::array<float, 4>> &points) const
  {
    sensor_msgs::msg::PointCloud2 message;
    message.header.stamp = stamp;
    message.header.frame_id = metric_frame_id_;
    message.height = 1U;
    message.width = static_cast<uint32_t>(points.size());
    message.is_bigendian = false;
    message.is_dense = true;
    message.point_step = 16U;
    message.row_step = message.point_step * message.width;
    message.fields.resize(4U);
    const char *names[] = {"x", "y", "z", "intensity"};
    for (size_t i = 0; i < 4U; ++i) {
      message.fields[i].name = names[i];
      message.fields[i].offset = static_cast<uint32_t>(i * sizeof(float));
      message.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
      message.fields[i].count = 1U;
    }
    message.data.resize(points.size() * 4U * sizeof(float));
    if (!points.empty()) {
      std::memcpy(message.data.data(), points.data(), message.data.size());
    }
    return message;
  }


  sensor_msgs::msg::PointCloud2 makeTrackedPointCloud(
    const rclcpp::Time &stamp,
    const std::vector<std::array<float, 5>> &points) const
  {
    sensor_msgs::msg::PointCloud2 message;
    message.header.stamp = stamp;
    message.header.frame_id = metric_frame_id_;
    message.height = 1U;
    message.width = static_cast<uint32_t>(points.size());
    message.is_bigendian = false;
    message.is_dense = true;
    message.point_step = 20U;
    message.row_step = message.point_step * message.width;
    message.fields.resize(5U);
    const char *names[] = {"x", "y", "z", "intensity", "track_id"};
    for (size_t i = 0; i < 5U; ++i) {
      message.fields[i].name = names[i];
      message.fields[i].offset = static_cast<uint32_t>(i * sizeof(float));
      message.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
      message.fields[i].count = 1U;
    }
    message.data.resize(points.size() * 5U * sizeof(float));
    if (!points.empty()) {
      std::memcpy(message.data.data(), points.data(), message.data.size());
    }
    return message;
  }

  // Menerbitkan fan clearing tanpa menembus obstacle track aktif. DDS tidak
  // menjamin urutan antar-topic marking/clearing, jadi ray yang memotong track
  // harus dibuang agar dropout singkat tidak menghapus obstacle yang masih valid.
  void publishClearingFan(const rclcpp::Time &stamp) {
    if (!object_clearing_points_pub_ || object_clearing_points_pub_->get_subscription_count() == 0U) return;
    std::vector<std::array<float, 4>> points;
    points.reserve(static_cast<size_t>(clearing_ray_count_));
    for (int i = 0; i < clearing_ray_count_; ++i) {
      const double ratio = clearing_ray_count_ <= 1 ? 0.5 :
        static_cast<double>(i) / static_cast<double>(clearing_ray_count_ - 1);
      const double angle = -0.5 * clearing_fov_rad_ + ratio * clearing_fov_rad_;
      bool intersects_track = false;
      for (const auto &track : obstacle_tracks_) {
        if (track.forward_m <= 0.0F) continue;
        const double track_angle = std::atan2(track.left_m, track.forward_m);
        const double distance = std::hypot(track.forward_m, track.left_m);
        const double protected_half_width =
          0.5 * std::max(0.20, static_cast<double>(track.width_m)) +
          clearing_obstacle_margin_m_;
        const double protected_half_angle =
          std::atan2(protected_half_width, std::max(0.05, distance));
        const double angle_error = std::atan2(
          std::sin(angle - track_angle), std::cos(angle - track_angle));
        if (std::abs(angle_error) <= protected_half_angle) {
          intersects_track = true;
          break;
        }
      }
      if (intersects_track) continue;
      points.push_back({
        static_cast<float>(clearing_range_m_ * std::cos(angle)),
        static_cast<float>(clearing_range_m_ * std::sin(angle)), 0.20F, 0.0F});
    }
    object_clearing_points_pub_->publish(makePointCloud(stamp, points));
  }

  // Memberi ID stabil pada obstacle dengan nearest-neighbour per class dan EMA.
  // Data bbox pixel tetap data frame terbaru; hanya posisi/width metrik yang dihaluskan.
  void updateObstacleTracks(std::vector<MetricDetectionRuntime> &detections) {
    std::sort(detections.begin(), detections.end(), [](const auto &a, const auto &b) {
      return a.forward_m < b.forward_m;
    });
    std::vector<bool> used(obstacle_tracks_.size(), false);
    for (auto &detection : detections) {
      int best = -1;
      double best_distance = std::numeric_limits<double>::infinity();
      for (size_t i = 0; i < obstacle_tracks_.size(); ++i) {
        if (used[i] || obstacle_tracks_[i].class_id != detection.class_id) continue;
        const double dx = obstacle_tracks_[i].forward_m - detection.forward_m;
        const double dy = obstacle_tracks_[i].left_m - detection.left_m;
        const double distance = std::hypot(dx, dy);
        if (distance < track_match_distance_m_ && distance < best_distance) {
          best = static_cast<int>(i);
          best_distance = distance;
        }
      }

      if (best < 0) {
        obstacle_tracks_.push_back(ObstacleTrackRuntime{
          next_track_id_++, detection.class_id, detection.score, detection.forward_m,
          detection.left_m, detection.width_m, 1, 0, track_confirm_hits_ <= 1});
        used.push_back(true);
        best = static_cast<int>(obstacle_tracks_.size() - 1U);
      } else {
        auto &track = obstacle_tracks_[static_cast<size_t>(best)];
        const float alpha = static_cast<float>(track_ema_alpha_);
        track.forward_m = alpha * detection.forward_m + (1.0F - alpha) * track.forward_m;
        track.left_m = alpha * detection.left_m + (1.0F - alpha) * track.left_m;
        track.width_m = alpha * detection.width_m + (1.0F - alpha) * track.width_m;
        track.score = alpha * detection.score + (1.0F - alpha) * track.score;
        track.hits = std::min(track.hits + 1, 1000000);
        track.confirmed = track.confirmed || track.hits >= track_confirm_hits_;
        track.missed = 0;
        used[static_cast<size_t>(best)] = true;
      }
      const auto &track = obstacle_tracks_[static_cast<size_t>(best)];
      detection.track_id = track.track_id;
      detection.forward_m = track.forward_m;
      detection.left_m = track.left_m;
      detection.width_m = track.width_m;
      detection.confirmed = track.confirmed;
    }

    for (size_t i = 0; i < obstacle_tracks_.size(); ++i) {
      if (i >= used.size() || !used[i]) ++obstacle_tracks_[i].missed;
    }
    obstacle_tracks_.erase(
      std::remove_if(obstacle_tracks_.begin(), obstacle_tracks_.end(), [this](const auto &track) {
        return track.missed > track_max_missed_frames_;
      }), obstacle_tracks_.end());

    // Re-publish bounded held tracks so PointCloud, Collision Monitor, local
    // costmap, marker, and control mixer all see the same persistent state.
    for (const auto &track : obstacle_tracks_) {
      if (track.missed <= 0) continue;
      MetricDetectionRuntime held;
      held.track_id = track.track_id;
      held.class_id = track.class_id;
      held.score = track.score * static_cast<float>(
        std::pow(0.90, static_cast<double>(track.missed)));
      held.forward_homography_m = track.forward_m;
      held.forward_m = track.forward_m;
      held.left_m = track.left_m;
      held.width_m = track.width_m;
      held.missed_frames = track.missed;
      held.confirmed = track.confirmed;
      if (held.confirmed) detections.push_back(held);
    }
    detections.erase(
      std::remove_if(detections.begin(), detections.end(), [](const auto &d) { return !d.confirmed; }),
      detections.end());
  }

  // Menerbitkan PointCloud, marker, dan JSON diagnostik obstacle. JSON akhir juga
  // membawa geometry bbox sehingga logger kalibrasi statik tidak lagi menerima field kosong.
  void updateAndPublishObstacles(
    const rclcpp::Time &stamp,
    std::vector<MetricDetectionRuntime> detections)
  {
    updateObstacleTracks(detections);
    last_confirmed_obstacle_count_ = static_cast<int>(detections.size());

    // PointCloud dipakai local costmap + Collision Monitor yang berjalan jauh
    // di bawah 30 FPS kamera. Batasi publish ke 12 Hz agar callback DDS tidak
    // membanjiri executor Collision Monitor; tracking internal tetap 30 FPS.
    const bool cloud_due =
      last_obstacle_cloud_pub_time_.nanoseconds() == 0 ||
      (stamp - last_obstacle_cloud_pub_time_).seconds() < 0.0 ||
      (stamp - last_obstacle_cloud_pub_time_).seconds() >=
        (1.0 / obstacle_cloud_max_publish_rate_hz_);
    const bool costmap_ready =
      local_costmap_ready_.load(std::memory_order_acquire);
    if (cloud_due && costmap_ready) {
      last_obstacle_cloud_pub_time_ = stamp;
      publishClearingFan(stamp);
    }

    if (cloud_due && costmap_ready &&
        object_points_pub_ && object_points_pub_->get_subscription_count() > 0U) {
      std::vector<std::array<float, 5>> points;
      for (const auto &detection : detections) {
        const float half = 0.5F * detection.width_m;
        for (int i = 0; i < points_per_box_; ++i) {
          const float ratio = points_per_box_ <= 1 ? 0.5F :
            static_cast<float>(i) / static_cast<float>(points_per_box_ - 1);
          const float offset = -half + ratio * (2.0F * half);
          points.push_back({
            detection.forward_m, detection.left_m + offset, 0.20F, detection.score,
            static_cast<float>(detection.track_id)});
        }
      }
      object_points_pub_->publish(makeTrackedPointCloud(stamp, points));
    }

    if (obstacle_markers_pub_ && obstacle_markers_pub_->get_subscription_count() > 0U) {
      visualization_msgs::msg::MarkerArray array;
      for (const auto &detection : detections) {
        visualization_msgs::msg::Marker cube;
        cube.header.stamp = stamp;
        cube.header.frame_id = metric_frame_id_;
        cube.ns = "yolop_obstacles";
        cube.id = detection.track_id;
        cube.type = visualization_msgs::msg::Marker::CUBE;
        cube.action = visualization_msgs::msg::Marker::ADD;
        cube.pose.position.x = detection.forward_m;
        cube.pose.position.y = detection.left_m;
        cube.pose.position.z = 0.45;
        cube.pose.orientation.w = 1.0;
        cube.scale.x = 0.45;
        cube.scale.y = std::max(0.20F, detection.width_m);
        cube.scale.z = detection.class_id == 0 ? 0.90 : 0.60;
        cube.color.a = 0.65F;
        cube.color.r = 1.0F;
        cube.color.g = detection.missed_frames > 0 ? 0.75F : 0.25F;
        cube.color.b = 0.05F;
        cube.lifetime.sec = 0;
        cube.lifetime.nanosec = 350000000U;
        array.markers.push_back(cube);

        visualization_msgs::msg::Marker label;
        label.header = cube.header;
        label.ns = "yolop_obstacle_labels";
        label.id = 10000 + detection.track_id;
        label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        label.action = visualization_msgs::msg::Marker::ADD;
        label.pose.position.x = detection.forward_m;
        label.pose.position.y = detection.left_m;
        label.pose.position.z = 1.05;
        label.pose.orientation.w = 1.0;
        label.scale.z = 0.20;
        label.color.a = 1.0F;
        label.color.r = label.color.g = label.color.b = 1.0F;
        std::ostringstream text;
        text << detectionSemanticName(detection.class_id) << " cls=" << detection.class_id
             << " #" << detection.track_id << " x=" << std::fixed << std::setprecision(2) << detection.forward_m
             << "m y=" << detection.left_m << 'm';
        if (detection.missed_frames > 0) {
          text << " hold=" << detection.missed_frames;
        }
        label.text = text.str();
        label.lifetime = cube.lifetime;
        array.markers.push_back(label);
      }
      obstacle_markers_pub_->publish(std::move(array));
    }

    if (perception_obstacle_metrics_pub_ && perception_obstacle_metrics_pub_->get_subscription_count() > 0U) {
      const int64_t ns = stamp.nanoseconds();
      std::ostringstream json;
      json << std::fixed << std::setprecision(5)
           << "{\"stamp\":{\"sec\":" << ns / 1000000000LL
           << ",\"nanosec\":" << ns % 1000000000LL << "},\"frame_id\":\""
           << metric_frame_id_ << "\",\"mode\":\"gpu_homography_ready\",\"count\":"
           << detections.size() << ",\"detections\":[";
      for (size_t i = 0; i < detections.size(); ++i) {
        if (i) json << ',';
        const auto &d = detections[i];
        json << "{\"track_id\":" << d.track_id
             << ",\"class_name\":\"" << detectionSemanticName(d.class_id)
             << "\",\"class_id\":" << d.class_id
             << ",\"score\":" << d.score
             << ",\"missed_frames\":" << d.missed_frames
             << ",\"center_x_px\":" << d.center_x_px
             << ",\"center_y_px\":" << d.center_y_px
             << ",\"bottom_y_px\":" << d.bottom_y_px
             << ",\"width_px\":" << d.width_px
             << ",\"height_px\":" << d.height_px
             << ",\"area_px\":" << d.area_px
             << ",\"forward_homography_m\":" << d.forward_homography_m
             << ",\"forward_area_m\":null"
             << ",\"forward_m\":" << d.forward_m
             << ",\"left_m\":" << d.left_m
             << ",\"metric_width_m\":" << d.width_m << '}';
      }
      json << "]}";
      std_msgs::msg::String message;
      message.data = json.str();
      perception_obstacle_metrics_pub_->publish(std::move(message));
    }

    {
      std::lock_guard<std::mutex> lock(perception_state_mutex_);
      latest_obstacles_ = detections;
      last_obstacle_time_ = std::chrono::steady_clock::now();
      have_obstacles_ = true;
    }
  }

  // Membuat visual centerline/boundary lane dan state teks untuk RViz.
  void publishLaneRuntimeVisuals(
    const rclcpp::Time &stamp,
    const LaneGeometryRuntime &geometry,
    const std::string &state,
    const std::string &reason)
  {
    if (lane_centerline_pub_ && lane_centerline_pub_->get_subscription_count() > 0U) {
      nav_msgs::msg::Path path;
      path.header.stamp = stamp;
      path.header.frame_id = metric_frame_id_;
      for (const auto &sample : geometry.center_samples) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = sample[0];
        pose.pose.position.y = sample[1];
        pose.pose.position.z = 0.05;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
      }
      lane_centerline_pub_->publish(std::move(path));
    }

    if (!lane_safety_markers_pub_ || lane_safety_markers_pub_->get_subscription_count() == 0U) return;
    visualization_msgs::msg::MarkerArray output;
    const auto add_line = [&](int id, const std::string &ns,
                              const std::vector<std::array<float, 2>> &samples,
                              double z, double width) {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = stamp;
      marker.header.frame_id = metric_frame_id_;
      marker.ns = ns;
      marker.id = id;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = width;
      marker.color.a = 0.9F;
      marker.color.r = ns == "lane_center" ? 0.2F : 0.9F;
      marker.color.g = ns == "lane_center" ? 0.9F : 0.7F;
      marker.color.b = 0.2F;
      marker.lifetime.sec = 0;
      marker.lifetime.nanosec = 300000000U;
      for (const auto &sample : samples) {
        geometry_msgs::msg::Point point;
        point.x = sample[0];
        point.y = sample[1];
        point.z = z;
        marker.points.push_back(point);
      }
      output.markers.push_back(std::move(marker));
    };
    if (geometry.valid) {
      add_line(1, "lane_left", geometry.left_samples, 0.04, 0.05);
      add_line(2, "lane_right", geometry.right_samples, 0.04, 0.05);
      add_line(3, "lane_center", geometry.center_samples, 0.07, 0.07);
    }
    visualization_msgs::msg::Marker text;
    text.header.stamp = stamp;
    text.header.frame_id = metric_frame_id_;
    text.ns = "lane_state";
    text.id = 10;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = 1.0;
    text.pose.position.z = 1.2;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.24;
    text.color.a = text.color.r = text.color.g = text.color.b = 1.0F;
    text.text = state + ": " + reason;
    text.lifetime.sec = 0;
    text.lifetime.nanosec = 300000000U;
    output.markers.push_back(std::move(text));
    lane_safety_markers_pub_->publish(std::move(output));
  }

  // Mengubah geometri lane hasil CUDA menjadi state safety terkonfirmasi dan
  // menyimpan snapshot untuk control timer tanpa serialisasi JSON internal.
  void updateIntegratedLaneRuntime(
    const rclcpp::Time &stamp,
    const LaneGeometryRuntime &geometry)
  {
    if (!integrated_runtime_enabled_ || !lane_state_filter_) return;
    const bool valid = geometry.valid && geometry.confidence >= minimum_metric_confidence_;
    const auto decision = safety::classifyLaneState(
      valid,
      geometry.left_clearance_m,
      geometry.right_clearance_m,
      geometry.center_error_m,
      lane_state_filter_->state(),
      lane_thresholds_);
    const std::string confirmed = lane_state_filter_->update(decision.state);

    {
      std::lock_guard<std::mutex> lock(perception_state_mutex_);
      latest_lane_geometry_ = geometry;
      latest_lane_valid_ = valid;
      latest_lane_state_ = confirmed;
      latest_lane_candidate_state_ = decision.state;
      latest_lane_critical_ = decision.critical;
      latest_lane_reason_ = decision.reason;
      last_lane_time_ = std::chrono::steady_clock::now();
      have_lane_ = true;
    }

    if (lane_state_pub_) {
      const int64_t ns = stamp.nanoseconds();
      std::ostringstream json;
      json << std::boolalpha << std::fixed << std::setprecision(5)
           << "{\"stamp\":{\"sec\":" << ns / 1000000000LL
           << ",\"nanosec\":" << ns % 1000000000LL << "},\"frame_id\":\""
           << metric_frame_id_ << "\",\"valid\":" << valid
           << ",\"state\":\"" << confirmed
           << "\",\"candidate_state\":\"" << decision.state
           << "\",\"critical\":" << decision.critical
           << ",\"reason\":\"" << decision.reason
           << "\",\"nominal_road_width_m\":" << nominal_road_width_m_
           << ",\"vehicle_width_m\":" << lane_vehicle_width_
           << ",\"warning_clearance_m\":" << lane_thresholds_.warning_clearance_m
           << ",\"critical_clearance_m\":" << lane_thresholds_.critical_clearance_m
           << ",\"release_clearance_m\":" << lane_thresholds_.release_clearance_m;
      if (valid) {
        json << ",\"left_clearance_m\":" << geometry.left_clearance_m
             << ",\"right_clearance_m\":" << geometry.right_clearance_m
             << ",\"center_error_m\":" << geometry.center_error_m
             << ",\"heading_error_rad\":" << geometry.heading_error_rad
             << ",\"road_width_m\":" << geometry.road_width_m
             << ",\"confidence\":" << geometry.confidence;
      }
      json << '}';
      std_msgs::msg::String message;
      message.data = json.str();
      lane_state_pub_->publish(std::move(message));
    }
    publishLaneRuntimeVisuals(stamp, geometry, confirmed, decision.reason);
  }

  // Memperbarui latch obstacle untuk mencegah stop/release berosilasi dari satu frame.
  // Fungsi: Menahan status blocked/clear beberapa frame agar keputusan stop tidak berosilasi.
  bool updateBlockedLatch(bool raw_blocked) {
    if (raw_blocked) {
      ++blocked_count_;
      clear_count_ = 0;
      if (blocked_count_ >= blocked_confirm_frames_) blocked_latched_ = true;
    } else {
      ++clear_count_;
      blocked_count_ = 0;
      if (clear_count_ >= clear_confirm_frames_) blocked_latched_ = false;
    }
    return blocked_latched_;
  }

  // Menerbitkan koridor recenter dan keputusan control agar operator dapat
  // melihat mengapa perintah Nav2 diteruskan, dikoreksi, atau dihentikan.
  void publishControlMarkers(
    const std::string &state,
    double target_center,
    const safety::ObstacleGateResult *gate)
  {
    if (!control_markers_pub_ || control_markers_pub_->get_subscription_count() == 0U) return;
    const auto stamp = now();
    visualization_msgs::msg::MarkerArray output;
    visualization_msgs::msg::Marker clear;
    clear.header.stamp = stamp;
    clear.header.frame_id = metric_frame_id_;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    output.markers.push_back(clear);

    if (gate != nullptr) {
      visualization_msgs::msg::Marker corridor;
      corridor.header = clear.header;
      corridor.ns = "recenter_swept_area";
      corridor.id = 1;
      corridor.type = visualization_msgs::msg::Marker::CUBE;
      corridor.action = visualization_msgs::msg::Marker::ADD;
      corridor.pose.position.x = 0.5 * (obstacle_gate_config_.forward_min_m + obstacle_gate_config_.forward_max_m);
      corridor.pose.position.y = 0.5 * (gate->swept_min_m + gate->swept_max_m);
      corridor.pose.position.z = 0.03;
      corridor.pose.orientation.w = 1.0;
      corridor.scale.x = obstacle_gate_config_.forward_max_m - obstacle_gate_config_.forward_min_m;
      corridor.scale.y = std::max(0.05, gate->swept_max_m - gate->swept_min_m);
      corridor.scale.z = 0.04;
      corridor.color.a = 0.22F;
      corridor.color.r = blocked_latched_ ? 1.0F : 0.0F;
      corridor.color.g = blocked_latched_ ? 0.2F : 1.0F;
      corridor.color.b = 0.1F;
      output.markers.push_back(corridor);

      visualization_msgs::msg::Marker arrow;
      arrow.header = clear.header;
      arrow.ns = "recenter_command";
      arrow.id = 2;
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;
      arrow.pose.orientation.w = 1.0;
      arrow.scale.x = 0.07;
      arrow.scale.y = 0.14;
      arrow.scale.z = 0.16;
      arrow.color.a = 1.0F;
      arrow.color.g = 1.0F;
      geometry_msgs::msg::Point start;
      start.x = 0.8; start.z = 0.20;
      geometry_msgs::msg::Point end;
      end.x = 2.0; end.y = target_center; end.z = 0.20;
      arrow.points = {start, end};
      output.markers.push_back(std::move(arrow));
    }

    visualization_msgs::msg::Marker text;
    text.header = clear.header;
    text.ns = "lane_control_text";
    text.id = 10;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = 1.0;
    text.pose.position.z = 1.65;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.18;
    text.color.a = 1.0F;
    text.color.r = (state == safety::BLOCKED_STOP || state == safety::LANE_LOST) ? 1.0F : 0.1F;
    text.color.g = state == safety::NORMAL ? 1.0F : 0.7F;
    text.text = "MIX " + control_mode_ + ": " + state;
    output.markers.push_back(std::move(text));
    control_markers_pub_->publish(std::move(output));
  }

  // Timer kontrol 30 Hz. Callback hanya membaca snapshot kecil yang sudah
  // dihasilkan worker perception; tidak pernah menunggu CUDA/TensorRT.
  // Fungsi: Menjalankan mixer safety periodik berdasarkan snapshot lane/obstacle dan cmd Nav2 terbaru.
  void controlTick() {
    if (!integrated_runtime_enabled_ || !safe_cmd_pub_) return;
    ++control_sequence_;
    const auto steady_now = std::chrono::steady_clock::now();

    geometry_msgs::msg::Twist nav_cmd;
    LaneGeometryRuntime lane;
    std::string lane_state = safety::LANE_LOST;
    bool lane_valid = false;
    bool critical = false;
    std::vector<MetricDetectionRuntime> obstacles;
    bool cmd_fresh = false, lane_fresh = false, obstacle_fresh = false;
    {
      std::lock_guard<std::mutex> lock(perception_state_mutex_);
      if (have_nav_cmd_) {
        cmd_fresh = std::chrono::duration<double>(steady_now - last_nav_cmd_time_).count() <= cmd_timeout_sec_;
        if (cmd_fresh) nav_cmd = last_nav_cmd_;
      }
      if (have_lane_) {
        lane_fresh = std::chrono::duration<double>(steady_now - last_lane_time_).count() <= lane_state_timeout_sec_;
        if (lane_fresh) {
          lane = latest_lane_geometry_;
          lane_state = latest_lane_state_;
          lane_valid = latest_lane_valid_;
          critical = latest_lane_critical_;
        }
      }
      if (have_obstacles_) {
        obstacle_fresh = std::chrono::duration<double>(steady_now - last_obstacle_time_).count() <= obstacle_state_timeout_sec_;
        if (obstacle_fresh) obstacles = latest_obstacles_;
      }
    }

    std::string decision = safety::NORMAL;
    geometry_msgs::msg::Twist desired = nav_cmd;
    bool raw_blocked = false;
    safety::ObstacleGateResult gate_result;
    bool gate_valid = false;

    if (!lane_safety_enabled_) {
      decision = "DISABLED_PASSTHROUGH";
      blocked_latched_ = false;
    } else if (!camera_metric_calibration_validated_) {
      // Kamera/inferensi boleh diuji sebelum homography metrik disertifikasi,
      // tetapi hasil pixel-space tidak boleh memperoleh authority steering.
      decision = "CALIBRATION_REQUIRED_PASSTHROUGH";
      blocked_latched_ = false;
    } else if (!cmd_fresh) {
      decision = "NAV_CMD_STALE_STOP";
      desired = geometry_msgs::msg::Twist{};
      blocked_latched_ = false;
    } else if (!lane_fresh || !lane_valid || lane_state == safety::LANE_LOST) {
      decision = safety::LANE_LOST;
      desired = geometry_msgs::msg::Twist{};
      blocked_latched_ = false;
    } else if (lane_state == safety::NORMAL) {
      decision = safety::NORMAL;
      blocked_latched_ = false;
      blocked_count_ = 0;
      clear_count_ = 0;
    } else if (lane_state == safety::RECENTER_LEFT || lane_state == safety::RECENTER_RIGHT) {
      std::vector<safety::ObstacleMetric> gate_obstacles;
      if (obstacle_fresh) {
        gate_obstacles.reserve(obstacles.size());
        for (const auto &item : obstacles) {
          gate_obstacles.push_back({item.track_id, item.forward_m, item.left_m, item.width_m});
        }
      }
      gate_result = safety::obstacleBlocksRecenter(
        lane.center_error_m, gate_obstacles, obstacle_gate_config_);
      gate_valid = true;
      raw_blocked = !obstacle_fresh || gate_result.blocked;
      if (!obstacle_fresh) gate_result.blocking_track_ids = {-2};

      bool blocked = false;
      if (raw_blocked && (critical || !obstacle_fresh)) {
        blocked_latched_ = true;
        blocked_count_ = blocked_confirm_frames_;
        clear_count_ = 0;
        blocked = true;
      } else {
        blocked = updateBlockedLatch(raw_blocked);
      }
      if (blocked) {
        decision = safety::BLOCKED_STOP;
        desired = geometry_msgs::msg::Twist{};
      } else {
        decision = lane_state;
        const auto mixed = safety::mixRecenterCommand(
          nav_cmd.linear.x, nav_cmd.angular.z,
          lane.center_error_m, lane.heading_error_rad, critical, mixer_config_);
        desired = geometry_msgs::msg::Twist{};
        desired.linear.x = mixed.linear_x;
        desired.angular.z = mixed.angular_z;
      }
    } else {
      decision = safety::LANE_LOST;
      desired = geometry_msgs::msg::Twist{};
    }

    const bool applied = lane_safety_enabled_ && camera_metric_calibration_validated_ &&
      control_mode_ == "active";
    const geometry_msgs::msg::Twist output = applied ? desired : nav_cmd;
    safe_cmd_pub_->publish(output);

    if (control_state_pub_) {
      std::ostringstream json;
      json << std::boolalpha << std::fixed << std::setprecision(5)
           << "{\"sequence\":" << control_sequence_
           << ",\"mode\":\"" << control_mode_
           << "\",\"enabled\":" << lane_safety_enabled_
           << ",\"metric_calibrated\":" << camera_metric_calibration_validated_
           << ",\"applied\":" << applied
           << ",\"decision\":\"" << decision
           << "\",\"lane_state\":\"" << lane_state
           << "\",\"lane_valid\":" << lane_valid
           << ",\"lane_fresh\":" << lane_fresh
           << ",\"obstacle_metrics_fresh\":" << obstacle_fresh
           << ",\"nav_cmd_fresh\":" << cmd_fresh
           << ",\"critical\":" << critical
           << ",\"center_error_m\":" << lane.center_error_m
           << ",\"heading_error_rad\":" << lane.heading_error_rad
           << ",\"raw_recenter_blocked\":" << raw_blocked
           << ",\"recenter_blocked\":" << blocked_latched_
           << ",\"nav_cmd\":{\"linear_x\":" << nav_cmd.linear.x
           << ",\"angular_z\":" << nav_cmd.angular.z
           << "},\"calculated_cmd\":{\"linear_x\":" << desired.linear.x
           << ",\"angular_z\":" << desired.angular.z
           << "},\"output_cmd\":{\"linear_x\":" << output.linear.x
           << ",\"angular_z\":" << output.angular.z << "}}";
      std_msgs::msg::String state_message;
      state_message.data = json.str();
      control_state_pub_->publish(std::move(state_message));
    }
    publishControlMarkers(decision, lane.center_error_m, gate_valid ? &gate_result : nullptr);
  }

  void publishCameraHealthState(
    bool healthy,
    const std::string &reason,
    double mean_luma,
    double stddev_luma,
    double extreme_fraction,
    double mean_gradient)
  {
    if (camera_health_pub_) {
      std_msgs::msg::Bool message;
      message.data = healthy;
      camera_health_pub_->publish(message);
    }
    if (camera_health_state_pub_) {
      std::ostringstream json;
      json << std::boolalpha << std::fixed << std::setprecision(3)
           << "{\"healthy\":" << healthy
           << ",\"reason\":\"" << reason << "\""
           << ",\"mean_luma\":" << mean_luma
           << ",\"stddev_luma\":" << stddev_luma
           << ",\"extreme_fraction\":" << extreme_fraction
           << ",\"mean_gradient\":" << mean_gradient
           << "}";
      std_msgs::msg::String message;
      message.data = json.str();
      camera_health_state_pub_->publish(message);
    }
  }

  void updateCameraHealth() {
    if (!camera_health_enabled_ || !h_camera_health_stats_) return;
    const double count = static_cast<double>(h_camera_health_stats_[CAMERA_HEALTH_COUNT]);
    if (count < 1.0) {
      camera_healthy_latched_ = false;
      camera_health_bad_count_ = camera_health_bad_confirm_frames_;
      camera_health_good_count_ = 0;
      publishCameraHealthState(false, "NO_SAMPLES", 0.0, 0.0, 1.0, 0.0);
      return;
    }

    const double sum = static_cast<double>(h_camera_health_stats_[CAMERA_HEALTH_SUM]);
    const double sum_sq = static_cast<double>(h_camera_health_stats_[CAMERA_HEALTH_SUM_SQ]);
    const double mean = sum / count;
    const double variance = std::max(0.0, sum_sq / count - mean * mean);
    const double stddev = std::sqrt(variance);
    const double dark_fraction =
      static_cast<double>(h_camera_health_stats_[CAMERA_HEALTH_DARK]) / count;
    const double bright_fraction =
      static_cast<double>(h_camera_health_stats_[CAMERA_HEALTH_BRIGHT]) / count;
    const double extreme_fraction = std::max(dark_fraction, bright_fraction);
    const double mean_gradient =
      static_cast<double>(h_camera_health_stats_[CAMERA_HEALTH_GRADIENT]) / count;

    const bool saturated = extreme_fraction >= camera_health_extreme_fraction_;
    const bool low_information =
      stddev < camera_health_min_stddev_ && mean_gradient < camera_health_min_gradient_;
    const bool raw_healthy = !saturated && !low_information;
    std::string reason = "OK";
    if (saturated) {
      reason = dark_fraction >= bright_fraction ? "DARK_OCCLUDED" : "BRIGHT_OCCLUDED";
    } else if (low_information) {
      reason = "LOW_INFORMATION";
    }

    if (raw_healthy) {
      camera_health_good_count_ = std::min(
        camera_health_good_count_ + 1, camera_health_good_confirm_frames_);
      camera_health_bad_count_ = 0;
      if (camera_health_good_count_ >= camera_health_good_confirm_frames_) {
        camera_healthy_latched_ = true;
      }
    } else {
      camera_health_bad_count_ = std::min(
        camera_health_bad_count_ + 1, camera_health_bad_confirm_frames_);
      camera_health_good_count_ = 0;
      if (camera_health_bad_count_ >= camera_health_bad_confirm_frames_) {
        camera_healthy_latched_ = false;
      }
    }

    // Saat transisi belum terkonfirmasi, state latch lama dipertahankan agar
    // satu frame gelap/overexposed tidak menyebabkan braking palsu.
    publishCameraHealthState(
      camera_healthy_latched_, reason, mean, stddev, extreme_fraction, mean_gradient);
  }

  void publishNearFieldState(
    bool emergency,
    const std::string &reason,
    int class_id,
    double confidence,
    double drivable_fraction)
  {
    if (emergency_stop_pub_) {
      std_msgs::msg::Bool message;
      message.data = emergency;
      emergency_stop_pub_->publish(message);
    }
    if (near_field_state_pub_) {
      std::ostringstream json;
      json << std::boolalpha << std::fixed << std::setprecision(3)
           << "{\"emergency\":" << emergency
           << ",\"reason\":\"" << reason << "\""
           << ",\"raw_class_id\":" << class_id
           << ",\"confidence\":" << confidence
           << ",\"near_field_drivable_fraction\":" << drivable_fraction
           << "}";
      std_msgs::msg::String message;
      message.data = json.str();
      near_field_state_pub_->publish(message);
    }
  }

  void updateNearFieldEmergency() {
    if (!near_field_emergency_enabled_ || !h_detection_count_ || !h_detections_) return;
    const int count = std::clamp(*h_detection_count_, 0, max_detections_);
    const double corridor_width = static_cast<double>(width_) * near_field_center_corridor_fraction_;
    const double corridor_left = (static_cast<double>(width_) - corridor_width) * 0.5;
    const double corridor_right = corridor_left + corridor_width;
    const double minimum_height = static_cast<double>(height_) * near_field_min_bbox_height_fraction_;

    bool raw_hazard = false;
    int hazard_class = -1;
    double hazard_confidence = 0.0;
    double near_field_drivable_fraction = 1.0;
    bool drivable_guard_hazard = false;
    if (near_field_drivable_guard_enabled_ && h_camera_health_stats_) {
      const double roi_count = static_cast<double>(h_camera_health_stats_[NEAR_FIELD_SAMPLE_COUNT]);
      const double road_count = static_cast<double>(h_camera_health_stats_[NEAR_FIELD_DRIVABLE_COUNT]);
      if (roi_count > 0.0) {
        near_field_drivable_fraction = road_count / roi_count;
        drivable_guard_hazard = near_field_drivable_fraction < near_field_min_drivable_fraction_;
      } else {
        near_field_drivable_fraction = 0.0;
        drivable_guard_hazard = true;
      }
    }
    for (int i = 0; i < count; ++i) {
      const DetectionGpu &detection = h_detections_[static_cast<size_t>(i)];
      if (detection.confidence < near_field_min_confidence_) continue;
      if ((detection.clip_flags & DETECTION_CLIP_BOTTOM) == 0) continue;
      const double box_height = static_cast<double>(detection.y2 - detection.y1);
      if (box_height < minimum_height) continue;
      const bool overlaps_ego_corridor =
        static_cast<double>(detection.x2) >= corridor_left &&
        static_cast<double>(detection.x1) <= corridor_right;
      if (!overlaps_ego_corridor) continue;
      raw_hazard = true;
      if (detection.confidence > hazard_confidence) {
        hazard_confidence = detection.confidence;
        hazard_class = detection.class_id;
      }
    }

    raw_hazard = raw_hazard || drivable_guard_hazard;
    if (raw_hazard) {
      near_field_hit_count_ = std::min(near_field_hit_count_ + 1, near_field_confirm_frames_);
      near_field_clear_count_ = 0;
      if (near_field_hit_count_ >= near_field_confirm_frames_) {
        near_field_emergency_latched_ = true;
      }
    } else {
      near_field_clear_count_ = std::min(near_field_clear_count_ + 1, near_field_release_frames_);
      near_field_hit_count_ = 0;
      if (near_field_clear_count_ >= near_field_release_frames_) {
        near_field_emergency_latched_ = false;
      }
    }

    const std::string reason = drivable_guard_hazard ? "NEAR_FIELD_DRIVABLE_OCCLUDED" :
      (raw_hazard ? "BOTTOM_TRUNCATED_EGO_CORRIDOR" : "CLEAR");
    publishNearFieldState(
      near_field_emergency_latched_, reason, hazard_class, hazard_confidence,
      near_field_drivable_fraction);
  }

  void publishCameraConnected(bool connected) {
    if (!camera_connected_pub_) return;
    std_msgs::msg::Bool msg;
    msg.data = connected;
    camera_connected_pub_->publish(msg);
  }

  // Membuka Astra sebagai kamera UVC/V4L2 dan mencetak jalur memori aktif.
  // Fungsi: Membuka kamera V4L2 dan memastikan mode resolusi/FPS sesuai kebijakan strict/fallback.
  void initializeCamera() {
    const int requested_width = width_;
    const int requested_height = height_;
    bool opened = false;
    while (rclcpp::ok() && !opened) {
      try {
        camera_.openDevice(
          rgb_device_, width_, height_, fps_, pixel_format_, allow_mjpeg_,
          use_userptr_, v4l2_buffer_count_, allow_resolution_fallback_for_fps_);
        opened = true;
      } catch (const std::exception & exception) {
        publishCameraConnected(false);
        if (strict_camera_mode_ || !camera_hotplug_retry_) {
          throw;
        }
        const std::string camera_error = oneLineDiagnostic(exception.what());
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(),
          static_cast<int64_t>(camera_retry_log_interval_sec_ * 1000.0),
          "Kamera V4L2 belum siap; hot-plug wait aktif. %s",
          camera_error.c_str());
        std::this_thread::sleep_for(
          std::chrono::duration<double>(camera_retry_interval_sec_));
      }
    }
    if (!opened) {
      publishCameraConnected(false);
      return;
    }
    publishCameraConnected(true);

    width_ = camera_.width();
    height_ = camera_.height();

    const bool resolution_changed =
      width_ != requested_width || height_ != requested_height;
    const bool fps_changed =
      std::abs(camera_.actualFps() - static_cast<double>(fps_)) > 0.5;

    if (strict_camera_mode_ && (resolution_changed || fps_changed)) {
      std::ostringstream error;
      error << "Mode kamera ketat gagal. Diminta "
            << requested_width << "x" << requested_height << "@" << fps_
            << " FPS " << fourccToString(camera_.pixelFormat()) << ", tetapi driver memberikan "
            << width_ << "x" << height_ << "@" << std::fixed
            << std::setprecision(2) << camera_.actualFps() << " FPS. "
            << "Periksa: v4l2-ctl --device=" << camera_.devicePath()
            << " --list-formats-ext";
      throw std::runtime_error(error.str());
    }

    if (fps_changed && !strict_camera_mode_) {
      RCLCPP_WARN(
        get_logger(),
        "Kamera tidak menyediakan FPS yang diminta secara eksak: requested=%d, actual=%.2f. "
        "Pipeline tetap berjalan dalam mode robust.",
        fps_, camera_.actualFps());
    }

    if (resolution_changed) {
      const double scale_x = static_cast<double>(width_) / requested_width;
      const double scale_y = static_cast<double>(height_) / requested_height;
      camera_fx_ *= scale_x;
      camera_cx_ *= scale_x;
      camera_fy_ *= scale_y;
      camera_cy_ *= scale_y;
      RCLCPP_WARN(
        get_logger(),
        "Mode %dx%d tidak menyediakan %d FPS %s; otomatis memakai %dx%d "
        "(maksimum terdaftar %.2f FPS) agar target FPS tercapai.",
        requested_width, requested_height, fps_, fourccToString(camera_.pixelFormat()).c_str(),
        width_, height_, camera_.selectedModeMaxFps());
    }

    RCLCPP_INFO(
      get_logger(),
      "V4L2 RGB: %s %dx%d requested=%d FPS actual=%.2f FPS %s, memory=%s, CUDA-mapped=%s, bytesperline=%zu sizeimage=%zu",
      camera_.devicePath().c_str(), width_, height_, fps_, camera_.actualFps(),
      fourccToString(camera_.pixelFormat()).c_str(),
      camera_.memoryMode() == V4L2Capture::MemoryMode::USERPTR ? "USERPTR" : "MMAP",
      camera_.directCudaMapped() ? "yes" : "no", camera_.bytesPerLine(), camera_.sizeImage());
    if (strict_camera_mode_) {
      RCLCPP_INFO(
        get_logger(),
        "Mode kamera ketat tervalidasi: %dx%d@%.2f FPS %s.",
        width_, height_, camera_.actualFps(), fourccToString(camera_.pixelFormat()).c_str());
    }
    if (std::abs(camera_.actualFps() - static_cast<double>(fps_)) > 0.5) {
      RCLCPP_WARN(
        get_logger(),
        "Driver kamera tidak dapat memenuhi %d FPS dan memilih %.2f FPS. "
        "Periksa mode aktual dengan v4l2-ctl --list-formats-ext.",
        fps_, camera_.actualFps());
    }
    if (camera_.pixelFormat() == V4L2_PIX_FMT_MJPEG) {
      RCLCPP_INFO(
        get_logger(),
        "MJPEG 1280x720 dipilih untuk mempertahankan target 30 FPS UVC; "
        "entropy decode memakai OpenCV CPU lalu preprocessing/inference kembali ke GPU.");
    }
  }

  // Fungsi: Memeriksa apakah OpenCV target mendukung OpenGL untuk preview GPU tanpa copy ke host.
  static bool opencvBuildHasOpenGl() {
    const std::string build_info = cv::getBuildInformation();
    for (const std::string &key : {std::string("OpenGL support:"), std::string("OpenGL:")}) {
      const size_t position = build_info.find(key);
      if (position == std::string::npos) continue;
      const size_t line_end = build_info.find('\n', position);
      const std::string line = build_info.substr(position, line_end - position);
      if (line.find("YES") != std::string::npos) return true;
    }
    return false;
  }

  // Jendela OpenCV dibuat sekali pada ukuran yang diminta. CUDA/OpenGL hanya
  // diaktifkan bila build OpenCV benar-benar mencantumkan OpenGL=YES. Build GTK
  // tanpa OpenGL langsung memakai pinned-host HighGUI tanpa memicu exception
  // "No OpenGL support" dan tanpa mencoba menghancurkan window yang belum ada.
  // Fungsi: Menyiapkan window preview opsional dan memilih jalur CUDA-OpenGL atau pinned-host fallback.
  void initializeOpenCvWindow() {
    if (!show_opencv_cuda_window_ || opencv_window_initialized_) return;

    constexpr const char *window_name = "YOLOP-v2 CUDA";
    const bool request_gl = opencv_use_cuda_opengl_ && opencvBuildHasOpenGl();
    if (opencv_use_cuda_opengl_ && !request_gl) {
      RCLCPP_INFO(
        get_logger(),
        "OpenCV dibangun tanpa dukungan HighGUI OpenGL; viewer memakai pinned-host HighGUI.");
    }

    try {
      const int flags = request_gl ? (cv::WINDOW_NORMAL | cv::WINDOW_OPENGL)
                                   : cv::WINDOW_NORMAL;
      cv::namedWindow(window_name, flags);
      if (opencv_window_fullscreen_) {
        cv::setWindowProperty(window_name, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
      } else {
        cv::resizeWindow(window_name, opencv_window_width_, opencv_window_height_);
      }
      cv::waitKey(1);

      opencv_cuda_opengl_active_ = false;
      if (request_gl) {
        const double gl_property = cv::getWindowProperty(window_name, cv::WND_PROP_OPENGL);
        if (gl_property > 0.5) {
          try {
            cv::cuda::setGlDevice(gpu_device_);
            opencv_cuda_opengl_active_ = true;
          } catch (const cv::Exception &interop_error) {
            RCLCPP_WARN(
              get_logger(),
              "CUDA/OpenGL interop tidak tersedia (%s); menggunakan pinned-host HighGUI.",
              interop_error.what());
          }
        }
      }
      opencv_window_initialized_ = true;
      RCLCPP_INFO(
        get_logger(),
        "OpenCV viewer: %dx%d%s, backend=%s",
        opencv_window_width_, opencv_window_height_,
        opencv_window_fullscreen_ ? " fullscreen" : "",
        opencv_cuda_opengl_active_ ? "CUDA GpuMat -> OpenGL" : "pinned host -> HighGUI");
    } catch (const cv::Exception &error) {
      RCLCPP_WARN(
        get_logger(),
        "OpenCV viewer tidak tersedia (%s); viewer dinonaktifkan.", error.what());
      show_opencv_cuda_window_ = false;
      opencv_cuda_opengl_active_ = false;
      opencv_window_initialized_ = false;
    }
  }

  // Fungsi: Mencatat pemakaian VRAM pada tahap startup untuk audit kapasitas GPU.
  void logCudaMemory(const char *stage) const {
    size_t free_bytes = 0;
    size_t total_bytes = 0;
    if (cudaMemGetInfo(&free_bytes, &total_bytes) == cudaSuccess) {
      const double mib = 1024.0 * 1024.0;
      RCLCPP_INFO(
        get_logger(), "GPU RAM %s: used=%.1f MiB free=%.1f MiB total=%.1f MiB",
        stage, static_cast<double>(total_bytes - free_bytes) / mib,
        static_cast<double>(free_bytes) / mib,
        static_cast<double>(total_bytes) / mib);
    }
  }

  void ensureHostOutputBuffers(
    bool need_drivable,
    bool need_lane,
    bool need_class,
    bool need_detections)
  {
    const size_t mask_bytes = maskBytes();
    if (need_drivable && !h_drivable_mask_) {
      CUDA_CHECK(cudaHostAlloc(
        reinterpret_cast<void **>(&h_drivable_mask_), mask_bytes, cudaHostAllocPortable));
    }
    if (need_lane && !h_lane_mask_) {
      CUDA_CHECK(cudaHostAlloc(
        reinterpret_cast<void **>(&h_lane_mask_), mask_bytes, cudaHostAllocPortable));
    }
    if (need_class && !h_class_mask_) {
      CUDA_CHECK(cudaHostAlloc(
        reinterpret_cast<void **>(&h_class_mask_), mask_bytes, cudaHostAllocPortable));
    }
    if (!h_detection_count_) {
      CUDA_CHECK(cudaHostAlloc(
        reinterpret_cast<void **>(&h_detection_count_), sizeof(int),
        cudaHostAllocPortable));
      *h_detection_count_ = 0;
    }
    if (need_detections && !h_detections_) {
      CUDA_CHECK(cudaHostAlloc(
        reinterpret_cast<void **>(&h_detections_),
        static_cast<size_t>(max_detections_) * sizeof(DetectionGpu),
        cudaHostAllocPortable));
    }
  }

  // Warm-up menghilangkan overhead inisialisasi pada frame pertama.
  // Fungsi: Menjalankan beberapa inference awal agar alokasi lazy TensorRT selesai sebelum operasi real-time.
  void warmupTensorRt() {
    CUDA_CHECK(cudaMemsetAsync(d_input_, 0, engine_.inputElements() * sizeof(float), stream_));
    for (int i = 0; i < warmup_iterations_; ++i) {
      if (!engine_.enqueue(stream_)) throw std::runtime_error("TensorRT warmup enqueueV3 failed");
    }
    CUDA_CHECK(cudaStreamSynchronize(stream_));
    RCLCPP_INFO(get_logger(), "TensorRT warmup complete (%d iterations)", warmup_iterations_);
    logCudaMemory("setelah TensorRT warmup");
  }

  template<typename PublisherT>
  // Mengecek apakah output perlu dibuat dan disalin ke RAM. Publisher yang tidak
  // dibuat dianggap tidak diperlukan. Dengan publish_only_when_subscribed=true,
  // salinan D2H hanya terjadi ketika benar-benar ada subscriber.
  // Fungsi: Menghindari copy/serialize output bila fitur dimatikan atau tidak ada subscriber.
  bool subscriberNeeded(const std::shared_ptr<PublisherT> &publisher, bool enabled) const {
    if (!enabled || !publisher) return false;
    if (!publish_only_when_subscribed_) return true;
    return publisher->get_subscription_count() +
           publisher->get_intra_process_subscription_count() > 0U;
  }

  // Memilih slot pinned memory tanpa pernah menunggu RViz.
  // Fungsi: Mengambil slot pinned-host kosong untuk publisher RViz asynchronous tanpa memblok inference.
  int acquireRvizFrameSlot() {
    std::lock_guard<std::mutex> lock(rviz_frame_mutex_);

    // Prioritaskan slot FREE. Bila RViz/DDS lebih lambat daripada inferensi,
    // gunakan kembali slot READY paling lama. Slot PUBLISHING tidak pernah
    // disentuh agar memcpy pada thread publisher tetap aman.
    for (size_t i = 0; i < rviz_frame_slots_.size(); ++i) {
      if (rviz_frame_slots_[i].state == RvizFrameState::FREE) {
        rviz_frame_slots_[i].state = RvizFrameState::FILLING;
        return static_cast<int>(i);
      }
    }

    size_t oldest_index = rviz_frame_slots_.size();
    uint64_t oldest_sequence = std::numeric_limits<uint64_t>::max();
    for (size_t i = 0; i < rviz_frame_slots_.size(); ++i) {
      if (rviz_frame_slots_[i].state == RvizFrameState::READY &&
          rviz_frame_slots_[i].sequence < oldest_sequence) {
        oldest_sequence = rviz_frame_slots_[i].sequence;
        oldest_index = i;
      }
    }
    if (oldest_index < rviz_frame_slots_.size()) {
      rviz_frame_slots_[oldest_index].state = RvizFrameState::FILLING;
      dropped_rviz_frames_.fetch_add(1, std::memory_order_relaxed);
      return static_cast<int>(oldest_index);
    }

    // Semua slot sedang diisi atau diserialisasi. Hanya preview frame ini yang
    // dilewati; inferensi TensorRT tetap berjalan tanpa menunggu RViz.
    dropped_rviz_frames_.fetch_add(1, std::memory_order_relaxed);
    return -1;
  }

  // Fungsi: Mengembalikan slot RViz yang batal diisi ke state kosong secara thread-safe.
  void releaseFillingRvizFrameSlot(int slot_index) {
    if (slot_index < 0) return;
    std::lock_guard<std::mutex> lock(rviz_frame_mutex_);
    auto &slot = rviz_frame_slots_[static_cast<size_t>(slot_index)];
    if (slot.state == RvizFrameState::FILLING) {
      slot.state = RvizFrameState::FREE;
    }
  }

  // Menandai slot selesai diisi dan membangunkan thread publisher.
  // Fungsi: Menandai frame pinned-host terbaru siap dipublish dan menerapkan kebijakan latest-frame-only.
  void commitRvizFrameSlot(int slot_index, const rclcpp::Time &stamp) {
    if (slot_index < 0) return;
    {
      std::lock_guard<std::mutex> lock(rviz_frame_mutex_);
      auto &slot = rviz_frame_slots_[static_cast<size_t>(slot_index)];
      slot.stamp = stamp;
      slot.sequence = ++rviz_frame_sequence_;
      slot.state = RvizFrameState::READY;

      // Simpan hanya satu frame READY terbaru. Ini adalah kebijakan utama untuk
      // latency rendah: RViz melihat kondisi terkini, bukan memutar antrean lama.
      for (size_t i = 0; i < rviz_frame_slots_.size(); ++i) {
        if (static_cast<int>(i) != slot_index &&
            rviz_frame_slots_[i].state == RvizFrameState::READY) {
          rviz_frame_slots_[i].state = RvizFrameState::FREE;
          dropped_rviz_frames_.fetch_add(1, std::memory_order_relaxed);
        }
      }
    }
    rviz_frame_cv_.notify_one();
  }

  // Thread khusus serialisasi/publish sensor_msgs/Image untuk RViz2.
  // Fungsi: Thread publisher RViz yang memindahkan frame siap menjadi Image ROS tanpa menahan CUDA worker.
  void rvizPublisherLoop() {
    try {
      sensor_msgs::msg::Image message;
      message.header.frame_id = frame_id_;
      message.height = static_cast<uint32_t>(height_);
      message.width = static_cast<uint32_t>(width_);
      message.encoding = "bgr8";
      message.is_bigendian = false;
      message.step = static_cast<uint32_t>(width_ * 3);
      message.data.resize(imageBytes());

      while (running_.load(std::memory_order_acquire) && rclcpp::ok()) {
        int selected_index = -1;
        {
          std::unique_lock<std::mutex> lock(rviz_frame_mutex_);
          rviz_frame_cv_.wait(lock, [this]() {
            if (!running_.load(std::memory_order_acquire)) return true;
            return std::any_of(
              rviz_frame_slots_.begin(), rviz_frame_slots_.end(),
              [](const RvizFrameSlot &slot) { return slot.state == RvizFrameState::READY; });
          });
          if (!running_.load(std::memory_order_acquire)) break;

          uint64_t newest_sequence = 0;
          for (size_t i = 0; i < rviz_frame_slots_.size(); ++i) {
            const auto &slot = rviz_frame_slots_[i];
            if (slot.state == RvizFrameState::READY && slot.sequence >= newest_sequence) {
              newest_sequence = slot.sequence;
              selected_index = static_cast<int>(i);
            }
          }
          if (selected_index < 0) continue;
          rviz_frame_slots_[static_cast<size_t>(selected_index)].state = RvizFrameState::PUBLISHING;
        }

        auto &slot = rviz_frame_slots_[static_cast<size_t>(selected_index)];
        if (subscriberNeeded(annotated_pub_, publish_annotated_)) {
          message.header.stamp = slot.stamp;
          std::memcpy(message.data.data(), slot.host, message.data.size());
          annotated_pub_->publish(message);
          published_rviz_frames_.fetch_add(1, std::memory_order_relaxed);
        }

        {
          std::lock_guard<std::mutex> lock(rviz_frame_mutex_);
          slot.state = RvizFrameState::FREE;
        }
      }
    } catch (const std::exception &e) {
      RCLCPP_FATAL(get_logger(), "RViz publisher worker stopped: %s", e.what());
      running_.store(false, std::memory_order_release);
      if (rclcpp::ok()) rclcpp::shutdown();
    }
  }

  // Thread utama capture -> CUDA -> TensorRT -> postprocess.
  // Fungsi: Loop real-time capture→CUDA→TensorRT→postprocess dengan kebijakan drop frame lama.
  void processingLoop() {
    try {
      size_t frame_index = 0;
      auto stats_start = std::chrono::steady_clock::now();
      double accumulated_ms = 0.0;
      std::vector<double> timing_window_ms;
      timing_window_ms.reserve(300);
      while (running_.load() && rclcpp::ok()) {
        V4L2Frame frame;
        if (!camera_.dequeue(frame, 200)) continue;

        // Kuras frame yang sudah selesai ketika frame sebelumnya sedang diproses.
        // Hanya frame paling baru yang diinferensi agar latency V4L2 tidak tumbuh
        // saat FPS kamera sesaat lebih tinggi daripada FPS pipeline.
        for (int i = 1; i < std::max(2, v4l2_buffer_count_); ++i) {
          V4L2Frame newer_frame;
          if (!camera_.dequeue(newer_frame, 0)) break;
          camera_.queue(frame.index);
          frame = newer_frame;
          dropped_capture_frames_.fetch_add(1, std::memory_order_relaxed);
        }
        try {
          const auto started = std::chrono::steady_clock::now();
          processFrame(frame);
          const auto finished = std::chrono::steady_clock::now();
          const double frame_process_ms = std::chrono::duration<double, std::milli>(finished - started).count();
          accumulated_ms += frame_process_ms;
          timing_window_ms.push_back(frame_process_ms);
          ++frame_index;
          if (frame_index % 300U == 0U) {
            const double seconds = std::chrono::duration<double>(finished - stats_start).count();
            const double pipeline_fps = 300.0 / std::max(1.0e-6, seconds);
            const double pipeline_ms = accumulated_ms / 300.0;
            std::vector<double> sorted_timings = timing_window_ms;
            std::sort(sorted_timings.begin(), sorted_timings.end());
            const size_t p95_index = sorted_timings.empty() ? 0U :
              std::min(sorted_timings.size() - 1U, static_cast<size_t>(std::ceil(0.95 * sorted_timings.size()) - 1.0));
            const double pipeline_p95_ms = sorted_timings.empty() ? 0.0 : sorted_timings[p95_index];
            const double pipeline_min_ms = sorted_timings.empty() ? 0.0 : sorted_timings.front();
            const double pipeline_max_ms = sorted_timings.empty() ? 0.0 : sorted_timings.back();
            double variance_sum = 0.0;
            for (const double value : timing_window_ms) {
              const double delta = value - pipeline_ms;
              variance_sum += delta * delta;
            }
            const double pipeline_std_ms = timing_window_ms.size() > 1U ?
              std::sqrt(variance_sum / static_cast<double>(timing_window_ms.size() - 1U)) : 0.0;
            const auto capture_dropped = dropped_capture_frames_.load(std::memory_order_relaxed);
            const auto rviz_published = published_rviz_frames_.load(std::memory_order_relaxed);
            const auto rviz_dropped = dropped_rviz_frames_.load(std::memory_order_relaxed);
            const int raw_detections = h_detection_count_ ? std::min(*h_detection_count_, max_detections_) : 0;

            RCLCPP_INFO(
              get_logger(),
              "pipeline %.1f FPS, %.2f ms/frame, capture dropped=%llu, RViz published=%llu dropped=%llu, raw detections=%d metric candidates=%d confirmed obstacles=%d",
              pipeline_fps, pipeline_ms,
              static_cast<unsigned long long>(capture_dropped),
              static_cast<unsigned long long>(rviz_published),
              static_cast<unsigned long long>(rviz_dropped),
              raw_detections, last_metric_obstacle_count_, last_confirmed_obstacle_count_);

            if (performance_pub_) {
              std_msgs::msg::String perf;
              std::ostringstream json;
              json << "{\"backend\":\"gpu\",\"window_frames\":300"
                   << ",\"pipeline_fps\":" << std::fixed << std::setprecision(3) << pipeline_fps
                   << ",\"pipeline_ms_per_frame\":" << pipeline_ms
                   << ",\"pipeline_p95_ms\":" << pipeline_p95_ms
                   << ",\"pipeline_min_ms\":" << pipeline_min_ms
                   << ",\"pipeline_max_ms\":" << pipeline_max_ms
                   << ",\"pipeline_std_ms\":" << pipeline_std_ms
                   << ",\"capture_dropped_total\":" << capture_dropped
                   << ",\"rviz_published_total\":" << rviz_published
                   << ",\"rviz_dropped_total\":" << rviz_dropped
                   << ",\"raw_detection_count\":" << raw_detections
                   << ",\"metric_candidate_count\":" << last_metric_obstacle_count_
                   << ",\"confirmed_obstacle_count\":" << last_confirmed_obstacle_count_
                   << ",\"timing_scope\":\"processFrame_total\"}";
              perf.data = json.str();
              performance_pub_->publish(std::move(perf));
            }
            stats_start = finished;
            accumulated_ms = 0.0;
            timing_window_ms.clear();
          }
        } catch (...) {
          camera_.queue(frame.index);
          throw;
        }
        camera_.queue(frame.index);
      }
    } catch (const std::exception &e) {
      publishCameraConnected(false);
      RCLCPP_FATAL(get_logger(), "Perception worker stopped: %s", e.what());
      running_.store(false);
      if (rclcpp::ok()) rclcpp::shutdown();
    }
  }

  // YOLOPv2 official driving weights are trained for traffic-object/vehicle
  // detection. The TensorRT engine contains numeric class logits, not our old
  // COCO name table. V17 incorrectly interpreted raw class slot 3 as COCO
  // "motorcycle". V18 therefore reports the honest semantic label "vehicle"
  // and preserves raw_cls for calibration/debugging instead of inventing a
  // car/motor subtype that the supplied model metadata does not prove.
  static const char *detectionSemanticName(int class_id) {
    return class_id >= 0 ? "vehicle" : "unknown";
  }

  static const char *detectionHudName(int class_id) {
    return class_id >= 0 ? "kendaraan" : "unknown";
  }

  // Fungsi: Menghitung median vector kecil untuk robust lane-width statistics.
  static float medianValue(std::vector<float> values) {
    if (values.empty()) return 0.0F;
    const size_t middle = values.size() / 2U;
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle), values.end());
    float median = values[middle];
    if ((values.size() & 1U) == 0U) {
      const float lower = *std::max_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle));
      median = 0.5F * (lower + median);
    }
    return median;
  }

  // Fungsi: Menghaluskan metric lane dengan EMA setelah state awal tersedia.
  float laneEma(float value, float &state) {
    if (!lane_ema_initialized_) {
      state = value;
      return value;
    }
    state = lane_ema_alpha_ * value + (1.0F - lane_ema_alpha_) * state;
    return state;
  }

  // V17 calibration projection. Berbeda dari projectPixelToGroundGpu(), fungsi
  // ini sengaja TIDAK memotong hasil yang berada di luar BEV canvas, range,
  // lateral bound, atau drivable mask. Tujuannya murni menampilkan apa yang
  // dilihat YOLO + hasil homography mentah agar kalibrasi fisik bisa dilakukan.
  bool projectPixelToGroundRawHost(float px, float py, float &forward, float &left) const {
    if (ground_homography_host_.size() != 9U) return false;
    const auto &h = ground_homography_host_;
    const double w = static_cast<double>(h[6]) * px + static_cast<double>(h[7]) * py + h[8];
    if (!std::isfinite(w) || std::abs(w) < 1.0e-9) return false;
    const double bx = (static_cast<double>(h[0]) * px + static_cast<double>(h[1]) * py + h[2]) / w;
    const double by = (static_cast<double>(h[3]) * px + static_cast<double>(h[4]) * py + h[5]) / w;
    if (!std::isfinite(bx) || !std::isfinite(by)) return false;
    forward = static_cast<float>((static_cast<double>(ground_origin_y_) - by) * ground_scale_y_) +
      metric_forward_offset_;
    left = static_cast<float>((static_cast<double>(ground_origin_x_) - bx) * ground_scale_x_) +
      metric_lateral_offset_;
    return std::isfinite(forward) && std::isfinite(left);
  }

  void publishRawDetectionCalibration(const rclcpp::Time &stamp) {
    if (!h_detection_count_ || !h_detections_) return;
    const int count = std::max(0, std::min(*h_detection_count_, max_detections_));
    std::ostringstream summary;
    std::ostringstream detail;
    summary << std::fixed << std::setprecision(2);
    detail << std::fixed << std::setprecision(2);

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.header.stamp = stamp;
    clear.header.frame_id = metric_frame_id_;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    for (int i = 0; i < count; ++i) {
      const auto &d = h_detections_[i];
      const float cx = 0.5F * (d.x1 + d.x2);
      const float bottom = d.y2;
      float forward = 0.0F;
      float left = 0.0F;
      const bool projectable = projectPixelToGroundRawHost(cx, bottom, forward, left);
      const char *raw_name = detectionSemanticName(d.class_id);
      const char *name = detectionHudName(d.class_id);
      if (i) {
        summary << ' ';
        detail << " | ";
      }
      summary << name << "[cls=" << d.class_id << "](" << d.confidence << " x=";
      if (projectable) summary << forward << " y=" << left;
      else summary << "-- y=--";
      summary << ')';

      detail << raw_name << '/' << name << " raw_cls=" << d.class_id;
      detail << " conf=" << d.confidence << '(' << (d.confidence * 100.0F) << "%) x=";
      if (projectable) detail << forward << "m y=" << left << 'm';
      else detail << "-- y=--";
      detail << " bbox=[" << d.x1 << ',' << d.y1 << ',' << d.x2 << ',' << d.y2 << ']';

      if (projectable && raw_detection_markers_pub_) {
        visualization_msgs::msg::Marker point;
        point.header.stamp = stamp;
        point.header.frame_id = metric_frame_id_;
        point.ns = "yolop_raw_calibration";
        point.id = i * 2;
        point.type = visualization_msgs::msg::Marker::SPHERE;
        point.action = visualization_msgs::msg::Marker::ADD;
        point.pose.position.x = forward;
        point.pose.position.y = left;
        point.pose.position.z = 0.18;
        point.pose.orientation.w = 1.0;
        point.scale.x = 0.22;
        point.scale.y = 0.22;
        point.scale.z = 0.22;
        point.color.a = 0.95F;
        point.color.r = 1.0F;
        point.color.g = 0.85F;
        point.color.b = 0.1F;
        point.lifetime.nanosec = 350000000U;
        markers.markers.push_back(point);

        visualization_msgs::msg::Marker text = point;
        text.id = i * 2 + 1;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.pose.position.z = 0.55;
        text.scale.x = 0.0;
        text.scale.y = 0.0;
        text.scale.z = 0.24;
        std::ostringstream label;
        label << std::fixed << std::setprecision(2) << name << " cls=" << d.class_id
              << ' ' << d.confidence << " x=" << forward << " y=" << left;
        text.text = label.str();
        markers.markers.push_back(std::move(text));
      }
    }

    if (count == 0) {
      summary << "none";
      detail << "none";
    }

    if (raw_detection_summary_pub_) {
      std_msgs::msg::String msg;
      msg.data = summary.str();
      raw_detection_summary_pub_->publish(std::move(msg));
    }
    if (raw_detection_markers_pub_ && raw_detection_markers_pub_->get_subscription_count() > 0U) {
      raw_detection_markers_pub_->publish(std::move(markers));
    }

    const bool log_due = last_raw_detection_log_time_.nanoseconds() == 0 ||
      (stamp - last_raw_detection_log_time_).seconds() >= raw_detection_log_period_sec_;
    if (log_due) {
      RCLCPP_INFO(get_logger(), "YOLO RAW count=%d: %s", count, detail.str().c_str());
      last_raw_detection_log_time_ = stamp;
    }
  }

  // Mengubah buffer obstacle metrik dari GPU menjadi data runtime C++ dan,
  // bila diminta, JSON debug kompatibel dengan topic lama.
  // Fungsi: Mengambil hasil projection obstacle kecil dari host-pinned buffer dan meneruskan ke runtime/diagnostik.
  void publishObstacleMetrics(const rclcpp::Time &stamp) {
    std::vector<MetricDetectionRuntime> detections;
    detections.reserve(static_cast<size_t>(max_detections_));

    for (int i = 0; i < max_detections_; ++i) {
      const MetricDetectionGpu &metric = h_metric_detections_[static_cast<size_t>(i)];
      if (!metric.valid ||
          (!accept_all_detected_classes_as_obstacles_ && !isSafetyObstacleClass(metric.class_id)) ||
          metric.confidence < minimum_obstacle_confidence_) {
        continue;
      }
      detections.push_back(MetricDetectionRuntime{
        -1, metric.class_id, metric.confidence,
        metric.forward_m, metric.forward_m, metric.left_m, metric.width_m,
        metric.center_x_px, metric.center_y_px, metric.bottom_y_px,
        metric.width_px, metric.height_px, metric.area_px, 0});
    }

    last_metric_obstacle_count_ = static_cast<int>(detections.size());

    if (obstacle_metrics_pub_) {
      const int64_t ns = stamp.nanoseconds();
      std::ostringstream json;
      json << std::fixed << std::setprecision(5)
           << "{\"stamp\":{\"sec\":" << (ns / 1000000000LL)
           << ",\"nanosec\":" << (ns % 1000000000LL) << "},"
           << "\"frame_id\":\"" << metric_frame_id_ << "\","
           << "\"mode\":\"gpu_homography\",\"detections\":[";
      for (size_t i = 0; i < detections.size(); ++i) {
        if (i) json << ',';
        const auto &d = detections[i];
        json << "{\"track_id\":-1,\"class_name\":\"" << detectionSemanticName(d.class_id)
             << "\",\"class_id\":" << d.class_id
             << ",\"score\":" << d.score
             << ",\"forward_m\":" << d.forward_m
             << ",\"left_m\":" << d.left_m
             << ",\"metric_width_m\":" << d.width_m
             << ",\"center_x_px\":" << d.center_x_px
             << ",\"center_y_px\":" << d.center_y_px
             << ",\"bottom_y_px\":" << d.bottom_y_px
             << ",\"width_px\":" << d.width_px
             << ",\"height_px\":" << d.height_px
             << ",\"area_px\":" << d.area_px << '}';
      }
      json << "],\"count\":" << detections.size() << '}';
      std_msgs::msg::String message;
      message.data = json.str();
      obstacle_metrics_pub_->publish(std::move(message));
    }

    if (integrated_runtime_enabled_) {
      updateAndPublishObstacles(stamp, std::move(detections));
    }
  }

  // Safety class allow-list berada di parameter, bukan hard-coded pada command path.
  // Raw detection tetap dipublish untuk verifikasi mapping TensorRT aktual.
  bool isSafetyObstacleClass(int class_id) const {
    return std::find(
      safety_obstacle_class_ids_.begin(), safety_obstacle_class_ids_.end(),
      static_cast<int64_t>(class_id)) != safety_obstacle_class_ids_.end();
  }

  struct LaneSampleHost {
    float x{0.0F};
    float left{0.0F};
    float right{0.0F};
    float center{0.0F};
    float width{0.0F};
  };


  // Publish envelope drivable ke supervisor dan sebagai boundary cloud ke local costmap.
  // Titik boundary adalah "guard rail" virtual; MPPI tetap memilih trajectory sendiri.
  void publishDrivableSpace(const rclcpp::Time &stamp) {
    if (!integrated_runtime_enabled_) return;
    LaneGeometryRuntime lane_geometry;
    {
      std::lock_guard<std::mutex> lock(perception_state_mutex_);
      lane_geometry = latest_lane_geometry_;
    }
    const bool lane_constraint_valid = lane_geometry.valid &&
      lane_geometry.confidence >= drivable_lane_constraint_min_confidence_;
    std::vector<std::array<float, 4>> boundary_points;
    boundary_points.reserve(static_cast<size_t>(drivable_space_sample_rows_) * 2U);
    std::ostringstream json;
    const int64_t ns = stamp.nanoseconds();
    json << std::fixed << std::setprecision(4)
         << "{\"stamp\":{\"sec\":" << (ns / 1000000000LL)
         << ",\"nanosec\":" << (ns % 1000000000LL) << "},"
         << "\"frame_id\":\"" << metric_frame_id_ << "\",\"samples\":[";
    bool first = true;
    int valid_rows = 0;
    for (int i = 0; i < drivable_space_sample_rows_; ++i) {
      if (h_drivable_count_[static_cast<size_t>(i)] < drivable_space_min_pixels_per_row_) continue;
      const float t = drivable_space_sample_rows_ <= 1 ? 0.0F :
        static_cast<float>(i) / static_cast<float>(drivable_space_sample_rows_ - 1);
      const float forward = drivable_space_near_ + (drivable_space_far_ - drivable_space_near_) * t;
      float left = static_cast<float>(h_drivable_left_mm_[static_cast<size_t>(i)]) * 0.001F;
      float right = -static_cast<float>(h_drivable_right_mm_[static_cast<size_t>(i)]) * 0.001F;
      // Lane boundary adalah constraint tambahan terhadap drivable envelope,
      // tetapi hanya setelah lane geometry lolos robust filtering + confidence.
      // Ini mencegah noise satu pixel lane membuat virtual wall palsu di costmap.
      if (lane_constraint_valid) {
        auto constrain_from_samples = [forward](
          const std::vector<std::array<float, 2>> &samples, float &boundary, bool left_side) {
          float best_dx = std::numeric_limits<float>::infinity();
          float selected = boundary;
          bool found = false;
          for (const auto &sample : samples) {
            const float dx = std::fabs(sample[0] - forward);
            if (dx < best_dx && dx <= 0.15F) {
              best_dx = dx;
              selected = sample[1];
              found = true;
            }
          }
          if (!found) return;
          if (left_side && selected > 0.05F) {
            boundary = boundary > 0.05F ? std::min(boundary, selected) : selected;
          } else if (!left_side && selected < -0.05F) {
            boundary = boundary < -0.05F ? std::max(boundary, selected) : selected;
          }
        };
        constrain_from_samples(lane_geometry.left_samples, left, true);
        constrain_from_samples(lane_geometry.right_samples, right, false);
      }
      const bool left_valid = left > 0.05F;
      const bool right_valid = right < -0.05F;
      if (!left_valid && !right_valid) continue;
      ++valid_rows;
      if (!first) json << ',';
      first = false;
      json << '[' << forward << ',';
      if (left_valid) json << left; else json << "null";
      json << ',';
      if (right_valid) json << right; else json << "null";
      json << ']';
      if (left_valid) {
        boundary_points.push_back({forward,
          left + static_cast<float>(drivable_boundary_guard_offset_m_),
          static_cast<float>(drivable_boundary_z_m_), 1.0F});
      }
      if (right_valid) {
        boundary_points.push_back({forward,
          right - static_cast<float>(drivable_boundary_guard_offset_m_),
          static_cast<float>(drivable_boundary_z_m_), 1.0F});
      }
    }
    json << "],\"valid_rows\":" << valid_rows
         << ",\"sample_rows\":" << drivable_space_sample_rows_
         << ",\"valid\":" << (valid_rows >= std::max(4, drivable_space_sample_rows_ / 8) ? "true" : "false")
         << '}';
    if (drivable_space_pub_) {
      std_msgs::msg::String message;
      message.data = json.str();
      drivable_space_pub_->publish(std::move(message));
    }
    if (drivable_boundary_points_pub_) {
      drivable_boundary_points_pub_->publish(makePointCloud(stamp, boundary_points));
    }
  }

  // Mengubah lane bins kecil hasil CUDA menjadi geometri metrik, melakukan
  // reject outlier/EMA, lalu meneruskan hasil yang sama ke debug JSON dan safety C++.
  // Fungsi: Menyusun geometri lane dari bin CUDA, robust-filter, EMA, lalu memasukkannya ke safety runtime.
  void publishLaneMetrics(const rclcpp::Time &stamp) {
    constexpr int sentinel_limit = 1000000000;
    std::vector<LaneSampleHost> samples;
    std::vector<std::array<float, 2>> left_only_samples;
    std::vector<std::array<float, 2>> right_only_samples;
    samples.reserve(static_cast<size_t>(lane_sample_rows_));
    left_only_samples.reserve(static_cast<size_t>(lane_sample_rows_));
    right_only_samples.reserve(static_cast<size_t>(lane_sample_rows_));
    for (int i = 0; i < lane_sample_rows_; ++i) {
      const int left_mm = h_lane_left_mm_[static_cast<size_t>(i)];
      const int right_mm = h_lane_right_mm_[static_cast<size_t>(i)];
      const float t = lane_sample_rows_ <= 1 ? 0.0F :
        static_cast<float>(i) / static_cast<float>(lane_sample_rows_ - 1);
      const float x = lane_near_ + (lane_far_ - lane_near_) * t;
      const bool have_left = left_mm < sentinel_limit;
      const bool have_right = right_mm < sentinel_limit;
      const float left = have_left ? static_cast<float>(left_mm) * 0.001F : 0.0F;
      const float right = have_right ? -static_cast<float>(right_mm) * 0.001F : 0.0F;
      if (have_left && std::isfinite(left)) left_only_samples.push_back({x, left});
      if (have_right && std::isfinite(right)) right_only_samples.push_back({x, right});
      if (!have_left || !have_right) continue;
      const float width = left - right;
      if (!std::isfinite(width) || width < lane_min_width_ || width > lane_max_width_) continue;
      samples.push_back(LaneSampleHost{x, left, right, 0.5F * (left + right), width});
    }

    // Reject width outlier pada array kecil hasil GPU. Memindahkan operasi 72
    // elemen ini ke CUDA akan menambah launch/sync overhead tanpa keuntungan.
    if (!samples.empty()) {
      std::vector<float> widths;
      widths.reserve(samples.size());
      for (const auto &sample : samples) widths.push_back(sample.width);
      const float width_median = medianValue(widths);
      const float tolerance = std::max(0.35F, 0.15F * width_median);
      samples.erase(
        std::remove_if(samples.begin(), samples.end(), [&](const LaneSampleHost &sample) {
          return std::fabs(sample.width - width_median) > tolerance;
        }), samples.end());
    }

    bool single_side_estimated = false;
    std::string lane_source = "BOTH_BOUNDARIES";
    if (static_cast<int>(samples.size()) < lane_minimum_rows_) {
      const bool left_usable = static_cast<int>(left_only_samples.size()) >= lane_minimum_rows_;
      const bool right_usable = static_cast<int>(right_only_samples.size()) >= lane_minimum_rows_;
      if (left_usable || right_usable) {
        single_side_estimated = true;
        const float nominal_width = static_cast<float>(nominal_road_width_m_);
        samples.clear();
        if (left_usable && (!right_usable || left_only_samples.size() >= right_only_samples.size())) {
          lane_source = "LEFT_BOUNDARY_ESTIMATED_RIGHT";
          samples.reserve(left_only_samples.size());
          for (const auto &sample : left_only_samples) {
            const float left = sample[1];
            const float right = left - nominal_width;
            samples.push_back(
              LaneSampleHost{sample[0], left, right, 0.5F * (left + right), nominal_width});
          }
        } else {
          lane_source = "RIGHT_BOUNDARY_ESTIMATED_LEFT";
          samples.reserve(right_only_samples.size());
          for (const auto &sample : right_only_samples) {
            const float right = sample[1];
            const float left = right + nominal_width;
            samples.push_back(
              LaneSampleHost{sample[0], left, right, 0.5F * (left + right), nominal_width});
          }
        }
      }
    }

    const int64_t ns = stamp.nanoseconds();
    LaneGeometryRuntime geometry;
    geometry.left_samples.reserve(samples.size());
    geometry.right_samples.reserve(samples.size());
    geometry.center_samples.reserve(samples.size());
    for (const auto &sample : samples) {
      geometry.left_samples.push_back({sample.x, sample.left});
      geometry.right_samples.push_back({sample.x, sample.right});
      geometry.center_samples.push_back({sample.x, sample.center});
    }

    if (static_cast<int>(samples.size()) < lane_minimum_rows_) {
      if (lane_metrics_pub_) {
        std::ostringstream json;
        json << "{\"stamp\":{\"sec\":" << (ns / 1000000000LL)
             << ",\"nanosec\":" << (ns % 1000000000LL) << "},"
             << "\"frame_id\":\"" << metric_frame_id_ << "\","
             << "\"valid\":false,\"confidence\":0.0,\"valid_rows\":" << samples.size()
             << ",\"left_rows\":" << left_only_samples.size()
             << ",\"right_rows\":" << right_only_samples.size()
             << ",\"sample_rows\":" << lane_sample_rows_ << '}';
        std_msgs::msg::String message;
        message.data = json.str();
        lane_metrics_pub_->publish(std::move(message));
      }
      if (integrated_runtime_enabled_) updateIntegratedLaneRuntime(stamp, geometry);
      return;
    }

    float left_clearance = std::numeric_limits<float>::infinity();
    float right_clearance = std::numeric_limits<float>::infinity();
    std::vector<float> widths;
    widths.reserve(samples.size());
    for (const auto &sample : samples) {
      left_clearance = std::min(left_clearance, sample.left - 0.5F * lane_vehicle_width_);
      right_clearance = std::min(right_clearance, -sample.right - 0.5F * lane_vehicle_width_);
      widths.push_back(sample.width);
    }
    float road_width = medianValue(widths);
    float variance = 0.0F;
    for (float width : widths) {
      const float d = width - road_width;
      variance += d * d;
    }
    variance /= std::max<size_t>(1U, widths.size());
    const float width_std = std::sqrt(std::max(0.0F, variance));

    const auto closest_to_control = std::min_element(
      samples.begin(), samples.end(), [&](const LaneSampleHost &a, const LaneSampleHost &b) {
        return std::fabs(a.x - lane_control_lookahead_) < std::fabs(b.x - lane_control_lookahead_);
      });
    float center_error = closest_to_control->center;

    float mean_x = 0.0F, mean_y = 0.0F;
    for (const auto &sample : samples) { mean_x += sample.x; mean_y += sample.center; }
    mean_x /= static_cast<float>(samples.size());
    mean_y /= static_cast<float>(samples.size());
    float covariance = 0.0F, x_variance = 0.0F;
    for (const auto &sample : samples) {
      const float dx = sample.x - mean_x;
      covariance += dx * (sample.center - mean_y);
      x_variance += dx * dx;
    }
    const float slope = x_variance > 1.0e-6F ? covariance / x_variance : 0.0F;
    float heading_error = std::atan(slope);
    float confidence = std::clamp(
      (static_cast<float>(samples.size()) / static_cast<float>(lane_sample_rows_)) *
      std::exp(-width_std), 0.0F, 1.0F);
    if (single_side_estimated) confidence *= 0.55F;

    left_clearance = laneEma(left_clearance, lane_ema_left_clearance_);
    right_clearance = laneEma(right_clearance, lane_ema_right_clearance_);
    center_error = laneEma(center_error, lane_ema_center_error_);
    heading_error = laneEma(heading_error, lane_ema_heading_error_);
    road_width = laneEma(road_width, lane_ema_road_width_);
    confidence = laneEma(confidence, lane_ema_confidence_);
    lane_ema_initialized_ = true;

    geometry.valid = true;
    geometry.left_clearance_m = left_clearance;
    geometry.right_clearance_m = right_clearance;
    geometry.center_error_m = center_error;
    geometry.heading_error_rad = heading_error;
    geometry.road_width_m = road_width;
    geometry.confidence = confidence;
    geometry.single_side_estimated = single_side_estimated;
    geometry.lane_source = lane_source;

    if (lane_metrics_pub_) {
      std::ostringstream json;
      json << std::fixed << std::setprecision(5)
           << "{\"stamp\":{\"sec\":" << (ns / 1000000000LL)
           << ",\"nanosec\":" << (ns % 1000000000LL) << "},"
           << "\"frame_id\":\"" << metric_frame_id_ << "\",\"valid\":true"
           << ",\"left_clearance_m\":" << left_clearance
           << ",\"right_clearance_m\":" << right_clearance
           << ",\"center_error_m\":" << center_error
           << ",\"heading_error_rad\":" << heading_error
           << ",\"road_width_m\":" << road_width
           << ",\"confidence\":" << confidence
           << ",\"lane_source\":\"" << lane_source << "\""
           << ",\"single_side_estimated\":" << single_side_estimated
           << ",\"valid_rows\":" << samples.size()
           << ",\"sample_rows\":" << lane_sample_rows_;
      const auto write_samples = [&](const char *name, int selector) {
        json << ",\"" << name << "\":[";
        for (size_t i = 0; i < samples.size(); ++i) {
          if (i) json << ',';
          const float y = selector == 0 ? samples[i].left :
                          selector == 1 ? samples[i].right : samples[i].center;
          json << '[' << samples[i].x << ',' << y << ']';
        }
        json << ']';
      };
      write_samples("left_boundary_samples", 0);
      write_samples("right_boundary_samples", 1);
      write_samples("centerline_samples", 2);
      json << '}';
      std_msgs::msg::String message;
      message.data = json.str();
      lane_metrics_pub_->publish(std::move(message));
    }

    if (integrated_runtime_enabled_) updateIntegratedLaneRuntime(stamp, geometry);
  }

  // Memproses satu frame. Semua kernel berat dijalankan pada stream CUDA yang sama.
  // Fungsi: Memproses satu frame end-to-end pada CUDA stream dan hanya menyalin output kecil/yang diminta ke CPU.
  void processFrame(const V4L2Frame &frame) {
    const bool annotated_requested = subscriberNeeded(annotated_pub_, publish_annotated_);
    bool publish_annotated_frame = annotated_requested;
    if (publish_annotated_frame && rviz_max_publish_rate_hz_ > 0.0) {
      const auto publish_now = std::chrono::steady_clock::now();
      const auto minimum_period = std::chrono::duration<double>(1.0 / rviz_max_publish_rate_hz_);
      const bool first_publish =
        last_rviz_publish_time_.time_since_epoch().count() == 0;
      publish_annotated_frame = first_publish ||
        (publish_now - last_rviz_publish_time_) >= minimum_period;
      if (publish_annotated_frame) last_rviz_publish_time_ = publish_now;
    }
    const bool opencv_direct_gpu = show_opencv_cuda_window_ &&
      opencv_window_initialized_ && opencv_cuda_opengl_active_ && !draw_text_labels_cpu_;
    const bool opencv_host_frame = show_opencv_cuda_window_ && !opencv_direct_gpu;
    const bool need_annotated = publish_annotated_frame || show_opencv_cuda_window_;
    const bool need_annotated_host = publish_annotated_frame || opencv_host_frame || draw_text_labels_cpu_;
    const bool need_raw = subscriberNeeded(raw_pub_, publish_raw_);
    const bool need_drivable = subscriberNeeded(drivable_pub_, publish_drivable_mask_);
    const bool need_lane = subscriberNeeded(lane_pub_, publish_lane_mask_);
    const bool need_class = subscriberNeeded(class_mask_pub_, publish_class_mask_);
    const bool publish_detection_messages = subscriberNeeded(detections_pub_, publish_detections_);
    const bool need_lane_metrics = integrated_runtime_enabled_ ||
      subscriberNeeded(lane_metrics_pub_, publish_lane_metrics_);
    const bool need_obstacle_metrics = integrated_runtime_enabled_ ||
      subscriberNeeded(obstacle_metrics_pub_, publish_obstacle_metrics_);
    // Metric obstacle output is already self-contained in one GPU buffer.
    // Raw DetectionGpu only leaves VRAM for an explicit /yolop/detections
    // subscriber or CPU debug labels.
    const bool need_detection_host = integrated_runtime_enabled_ || publish_detection_messages ||
      (draw_text_labels_cpu_ && need_annotated);
    const bool need_segmentation = need_annotated || need_drivable || need_lane || need_class || need_lane_metrics;
    ensureHostOutputBuffers(need_drivable, need_lane, need_class, need_detection_host);
    int rviz_slot_index = -1;

    const dim3 image_block(32, 8);
    const dim3 image_grid(
      static_cast<unsigned int>((width_ + image_block.x - 1) / image_block.x),
      static_cast<unsigned int>((height_ + image_block.y - 1) / image_block.y));

    if (camera_.pixelFormat() == V4L2_PIX_FMT_YUYV) {
      const size_t stride = camera_.bytesPerLine();
      const size_t expected = stride * static_cast<size_t>(height_);
      if (stride < static_cast<size_t>(width_) * 2U || frame.bytes_used < expected) {
        throw std::runtime_error(
          "Short/invalid YUYV frame: bytes_used=" + std::to_string(frame.bytes_used) +
          " expected>=" + std::to_string(expected) +
          " stride=" + std::to_string(stride));
      }
      const uint8_t *source_device = frame.device;
      if (!source_device) {
        if (!d_capture_staging_) {
          throw std::runtime_error("V4L2 buffer is not CUDA-mapped and staging buffer is unavailable");
        }
        CUDA_CHECK(cudaMemcpyAsync(
          d_capture_staging_, frame.host, expected, cudaMemcpyHostToDevice, stream_));
        source_device = static_cast<const uint8_t *>(d_capture_staging_);
      }
      yuyvToBgrKernel<<<image_grid, image_block, 0, stream_>>>(
        source_device, d_bgr_, width_, height_, stride, flip_horizontal_);
      CUDA_CHECK(cudaGetLastError());
    } else if (camera_.pixelFormat() == V4L2_PIX_FMT_MJPEG) {
      cv::Mat jpeg(1, static_cast<int>(frame.bytes_used), CV_8UC1, frame.host);
      cv::Mat decoded = cv::imdecode(jpeg, cv::IMREAD_COLOR);
      if (decoded.empty()) throw std::runtime_error("OpenCV failed to decode V4L2 MJPEG frame");
      if (decoded.cols != width_ || decoded.rows != height_) {
        cv::resize(decoded, decoded, cv::Size(width_, height_), 0.0, 0.0, cv::INTER_LINEAR);
      }
      if (decoded.type() != CV_8UC3) {
        throw std::runtime_error("Decoded MJPEG frame is not CV_8UC3");
      }
      const size_t row_bytes = static_cast<size_t>(width_) * 3U;
      CUDA_CHECK(cudaMemcpy2DAsync(
        d_capture_staging_, row_bytes, decoded.data, decoded.step, row_bytes,
        static_cast<size_t>(height_), cudaMemcpyHostToDevice, stream_));
      copyOrFlipBgrKernel<<<image_grid, image_block, 0, stream_>>>(
        static_cast<const uint8_t *>(d_capture_staging_), d_bgr_, width_, height_, flip_horizontal_);
      CUDA_CHECK(cudaGetLastError());
    } else {
      throw std::runtime_error("Unsupported active V4L2 pixel format");
    }

    if (integrated_runtime_enabled_ && d_camera_health_stats_ && h_camera_health_stats_) {
      CUDA_CHECK(cudaMemsetAsync(
        d_camera_health_stats_, 0,
        static_cast<size_t>(CAMERA_HEALTH_STAT_COUNT) * sizeof(unsigned long long), stream_));
      const int sample_columns =
        (width_ + camera_health_sample_stride_px_ - 1) / camera_health_sample_stride_px_;
      const int sample_rows =
        (height_ + camera_health_sample_stride_px_ - 1) / camera_health_sample_stride_px_;
      const dim3 health_block(16, 16);
      const dim3 health_grid(
        static_cast<unsigned int>((sample_columns + static_cast<int>(health_block.x) - 1) /
          static_cast<int>(health_block.x)),
        static_cast<unsigned int>((sample_rows + static_cast<int>(health_block.y) - 1) /
          static_cast<int>(health_block.y)));
      if (camera_health_enabled_) {
        cameraHealthStatsKernel<<<health_grid, health_block, 0, stream_>>>(
          d_bgr_, width_, height_, camera_health_sample_stride_px_,
          camera_health_dark_luma_, camera_health_bright_luma_, d_camera_health_stats_);
        CUDA_CHECK(cudaGetLastError());
      }
    }

    if (need_raw && h_raw_) {
      CUDA_CHECK(cudaMemcpyAsync(h_raw_, d_bgr_, imageBytes(), cudaMemcpyDeviceToHost, stream_));
    }

    const dim3 model_block(32, 8);
    const dim3 model_grid(
      static_cast<unsigned int>((model_width_ + model_block.x - 1) / model_block.x),
      static_cast<unsigned int>((model_height_ + model_block.y - 1) / model_block.y));
    preprocessBgrKernel<<<model_grid, model_block, 0, stream_>>>(
      d_bgr_, width_, height_, d_input_, model_width_, model_height_, gain_, pad_x_, pad_y_);
    CUDA_CHECK(cudaGetLastError());

    if (!engine_.enqueue(stream_)) throw std::runtime_error("TensorRT enqueueV3 failed");

    CUDA_CHECK(cudaMemsetAsync(d_candidate_count_, 0, sizeof(int), stream_));
    for (int head = 0; head < 3; ++head) {
      const auto &output = engine_.detHead(head);
      const int grid_height = static_cast<int>(output.dims.d[2]);
      const int grid_width = static_cast<int>(output.dims.d[3]);
      const int channels = static_cast<int>(output.dims.d[1]);
      const int class_count = channels / 3 - 5;
      if (class_count <= 0) throw std::runtime_error("Invalid YOLO detection head channel count");
      const int stride = model_width_ / grid_width;
      const dim3 block(16, 16);
      const dim3 grid(
        static_cast<unsigned int>((grid_width + block.x - 1) / block.x),
        static_cast<unsigned int>((grid_height + block.y - 1) / block.y), 3U);
      const float *anchor_grid = static_cast<const float *>(engine_.anchorGrid(head).device);
      decodeHeadKernel<<<grid, block, 0, stream_>>>(
        static_cast<const float *>(output.device), anchor_grid,
        grid_width, grid_height, stride,
        class_count, confidence_threshold_, d_candidates_, d_candidate_count_, max_candidates_);
      CUDA_CHECK(cudaGetLastError());
    }

    const int nms_blocks = (max_candidates_ + 255) / 256;
    officialGreedyNmsKernel<<<1, 256, 0, stream_>>>(
      d_candidates_, d_candidate_count_, max_candidates_, max_detections_, iou_threshold_);
    CUDA_CHECK(cudaGetLastError());
    CUDA_CHECK(cudaMemsetAsync(d_final_count_, 0, sizeof(int), stream_));
    compactAndScaleKernel<<<nms_blocks, 256, 0, stream_>>>(
      d_candidates_, d_candidate_count_, max_candidates_, d_final_detections_, d_final_count_,
      max_detections_, gain_, pad_x_, pad_y_, width_, height_);
    CUDA_CHECK(cudaGetLastError());

    if (need_segmentation) {
      if (!d_drivable_mask_ || !d_lane_mask_) {
        throw std::runtime_error("Segmentation output requested but GPU mask buffers are unavailable");
      }
      const auto &drivable = engine_.drivableOutput();
      const auto &lane = engine_.laneOutput();
      segmentationKernel<<<image_grid, image_block, 0, stream_>>>(
        static_cast<const float *>(drivable.device), static_cast<const float *>(lane.device),
        model_width_, model_height_, d_drivable_mask_, d_lane_mask_, d_class_mask_,
        width_, height_, gain_, pad_x_, pad_y_, lane_threshold_);
      CUDA_CHECK(cudaGetLastError());

      if (integrated_runtime_enabled_ && near_field_emergency_enabled_ &&
          near_field_drivable_guard_enabled_ && d_camera_health_stats_) {
        const int sample_columns =
          (width_ + camera_health_sample_stride_px_ - 1) / camera_health_sample_stride_px_;
        const int sample_rows =
          (height_ + camera_health_sample_stride_px_ - 1) / camera_health_sample_stride_px_;
        const dim3 near_block(16, 16);
        const dim3 near_grid(
          static_cast<unsigned int>((sample_columns + static_cast<int>(near_block.x) - 1) /
            static_cast<int>(near_block.x)),
          static_cast<unsigned int>((sample_rows + static_cast<int>(near_block.y) - 1) /
            static_cast<int>(near_block.y)));
        nearFieldDrivableStatsKernel<<<near_grid, near_block, 0, stream_>>>(
          d_drivable_mask_, width_, height_, camera_health_sample_stride_px_,
          static_cast<float>(near_field_center_corridor_fraction_),
          static_cast<float>(near_field_bottom_roi_fraction_), d_camera_health_stats_);
        CUDA_CHECK(cudaGetLastError());
      }
    }

    if (integrated_runtime_enabled_ && d_camera_health_stats_ && h_camera_health_stats_) {
      CUDA_CHECK(cudaMemcpyAsync(
        h_camera_health_stats_, d_camera_health_stats_,
        static_cast<size_t>(CAMERA_HEALTH_STAT_COUNT) * sizeof(unsigned long long),
        cudaMemcpyDeviceToHost, stream_));
    }

    if (need_obstacle_metrics) {
      if (!d_ground_homography_ || !d_metric_detections_) {
        throw std::runtime_error("Obstacle metric GPU buffers are unavailable");
      }
      const int metric_blocks = (max_detections_ + 255) / 256;
      projectDetectionsMetricKernel<<<metric_blocks, 256, 0, stream_>>>(
        d_final_detections_, d_final_count_, max_detections_,
        d_drivable_mask_, width_, height_, require_drivable_contact_ ? 1 : 0,
        drivable_contact_radius_px_, drivable_contact_vertical_tolerance_px_,
        drivable_contact_min_samples_, drivable_contact_min_fraction_, d_ground_homography_,
        ground_origin_x_, ground_origin_y_, ground_scale_x_, ground_scale_y_,
        ground_canvas_width_, ground_canvas_height_,
        metric_forward_offset_, metric_lateral_offset_, metric_min_forward_,
        metric_max_forward_, metric_max_abs_left_, d_metric_detections_);
      CUDA_CHECK(cudaGetLastError());
    }

    if (need_lane_metrics) {
      if (!d_ground_homography_ || !d_lane_left_mm_ || !d_lane_right_mm_) {
        throw std::runtime_error("Lane metric GPU buffers are unavailable");
      }
      CUDA_CHECK(cudaMemsetAsync(
        d_lane_left_mm_, 0x7f, static_cast<size_t>(lane_sample_rows_) * sizeof(int), stream_));
      CUDA_CHECK(cudaMemsetAsync(
        d_lane_right_mm_, 0x7f, static_cast<size_t>(lane_sample_rows_) * sizeof(int), stream_));
      laneMetricBinsKernel<<<image_grid, image_block, 0, stream_>>>(
        d_lane_mask_, width_, height_, d_ground_homography_, ground_origin_x_, ground_origin_y_,
        ground_scale_x_, ground_scale_y_, ground_canvas_width_, ground_canvas_height_,
        lane_near_, lane_far_, lane_min_center_offset_,
        lane_sample_rows_, d_lane_left_mm_, d_lane_right_mm_);
      CUDA_CHECK(cudaGetLastError());
    }


    if (integrated_runtime_enabled_) {
      if (!d_ground_homography_ || !d_drivable_left_mm_ || !d_drivable_right_mm_ || !d_drivable_count_) {
        throw std::runtime_error("Drivable-space GPU buffers are unavailable");
      }
      CUDA_CHECK(cudaMemsetAsync(
        d_drivable_left_mm_, 0, static_cast<size_t>(drivable_space_sample_rows_) * sizeof(int), stream_));
      CUDA_CHECK(cudaMemsetAsync(
        d_drivable_right_mm_, 0, static_cast<size_t>(drivable_space_sample_rows_) * sizeof(int), stream_));
      CUDA_CHECK(cudaMemsetAsync(
        d_drivable_count_, 0, static_cast<size_t>(drivable_space_sample_rows_) * sizeof(int), stream_));
      const dim3 drivable_grid(
        (static_cast<unsigned>(width_ + drivable_space_pixel_stride_px_ - 1) /
         static_cast<unsigned>(drivable_space_pixel_stride_px_) + image_block.x - 1U) / image_block.x,
        (static_cast<unsigned>(height_ + drivable_space_pixel_stride_px_ - 1) /
         static_cast<unsigned>(drivable_space_pixel_stride_px_) + image_block.y - 1U) / image_block.y);
      drivableMetricBinsKernel<<<drivable_grid, image_block, 0, stream_>>>(
        d_drivable_mask_, width_, height_, d_ground_homography_, ground_origin_x_, ground_origin_y_,
        ground_scale_x_, ground_scale_y_, ground_canvas_width_, ground_canvas_height_,
        drivable_space_near_, drivable_space_far_, drivable_space_max_abs_left_,
        drivable_space_sample_rows_, drivable_space_pixel_stride_px_,
        d_drivable_left_mm_, d_drivable_right_mm_, d_drivable_count_);
      CUDA_CHECK(cudaGetLastError());
    }

    if (need_annotated) {
      overlayMasksKernel<<<image_grid, image_block, 0, stream_>>>(
        d_bgr_, d_drivable_mask_, d_lane_mask_, width_, height_, overlay_alpha_);
      CUDA_CHECK(cudaGetLastError());
      drawBoxesKernel<<<max_detections_, 256, 0, stream_>>>(
        d_bgr_, width_, height_, d_final_detections_, d_final_count_, max_detections_, box_thickness_);
      CUDA_CHECK(cudaGetLastError());
      if (async_rviz_publish_ && publish_annotated_frame) {
        rviz_slot_index = acquireRvizFrameSlot();
        if (rviz_slot_index >= 0) {
          CUDA_CHECK(cudaMemcpyAsync(
            rviz_frame_slots_[static_cast<size_t>(rviz_slot_index)].host,
            d_bgr_, imageBytes(), cudaMemcpyDeviceToHost, stream_));
        }
      } else if (need_annotated_host && h_annotated_) {
        CUDA_CHECK(cudaMemcpyAsync(h_annotated_, d_bgr_, imageBytes(), cudaMemcpyDeviceToHost, stream_));
      }
    }
    if (need_drivable) {
      CUDA_CHECK(cudaMemcpyAsync(h_drivable_mask_, d_drivable_mask_, maskBytes(), cudaMemcpyDeviceToHost, stream_));
    }
    if (need_lane) {
      CUDA_CHECK(cudaMemcpyAsync(h_lane_mask_, d_lane_mask_, maskBytes(), cudaMemcpyDeviceToHost, stream_));
    }
    if (need_class && d_class_mask_) {
      CUDA_CHECK(cudaMemcpyAsync(h_class_mask_, d_class_mask_, maskBytes(), cudaMemcpyDeviceToHost, stream_));
    }
    if (h_detection_count_) {
      // Hanya 4 byte/frame: simpan count asli GPU agar statistik runtime tidak
      // salah menunjukkan "0 detections" saat raw Detection2D tidak disubscribe.
      CUDA_CHECK(cudaMemcpyAsync(
        h_detection_count_, d_final_count_, sizeof(int), cudaMemcpyDeviceToHost, stream_));
    }
    if (need_detection_host) {
      CUDA_CHECK(cudaMemcpyAsync(
        h_detections_, d_final_detections_,
        static_cast<size_t>(max_detections_) * sizeof(DetectionGpu),
        cudaMemcpyDeviceToHost, stream_));
    }
    if (need_obstacle_metrics) {
      CUDA_CHECK(cudaMemcpyAsync(
        h_metric_detections_.data(), d_metric_detections_,
        static_cast<size_t>(max_detections_) * sizeof(MetricDetectionGpu),
        cudaMemcpyDeviceToHost, stream_));
    }
    if (need_lane_metrics) {
      CUDA_CHECK(cudaMemcpyAsync(
        h_lane_left_mm_.data(), d_lane_left_mm_,
        static_cast<size_t>(lane_sample_rows_) * sizeof(int), cudaMemcpyDeviceToHost, stream_));
      CUDA_CHECK(cudaMemcpyAsync(
        h_lane_right_mm_.data(), d_lane_right_mm_,
        static_cast<size_t>(lane_sample_rows_) * sizeof(int), cudaMemcpyDeviceToHost, stream_));
    }


    if (integrated_runtime_enabled_) {
      CUDA_CHECK(cudaMemcpyAsync(
        h_drivable_left_mm_.data(), d_drivable_left_mm_,
        static_cast<size_t>(drivable_space_sample_rows_) * sizeof(int), cudaMemcpyDeviceToHost, stream_));
      CUDA_CHECK(cudaMemcpyAsync(
        h_drivable_right_mm_.data(), d_drivable_right_mm_,
        static_cast<size_t>(drivable_space_sample_rows_) * sizeof(int), cudaMemcpyDeviceToHost, stream_));
      CUDA_CHECK(cudaMemcpyAsync(
        h_drivable_count_.data(), d_drivable_count_,
        static_cast<size_t>(drivable_space_sample_rows_) * sizeof(int), cudaMemcpyDeviceToHost, stream_));
    }

    CUDA_CHECK(cudaStreamSynchronize(stream_));

    const auto stamp = now();
    updateCameraHealth();
    updateNearFieldEmergency();
    // Lane geometry harus diperbarui lebih dahulu agar drivable envelope memakai
    // boundary lane yang sudah terfilter, bukan raw lane bins.
    if (need_lane_metrics) publishLaneMetrics(stamp);
    if (integrated_runtime_enabled_) publishDrivableSpace(stamp);
    if (need_raw && h_raw_) publishImage(raw_pub_, h_raw_, "bgr8", 3, stamp);
    if (need_annotated) {
      if (async_rviz_publish_ && publish_annotated_frame) {
        commitRvizFrameSlot(rviz_slot_index, stamp);
      } else {
        if (draw_text_labels_cpu_ && h_annotated_) drawCpuLabels();
        if (publish_annotated_frame && h_annotated_) {
          publishImage(annotated_pub_, h_annotated_, "bgr8", 3, stamp);
        }
      }
      if (show_opencv_cuda_window_) {
        constexpr const char *window_name = "YOLOP-v2 CUDA";
        try {
          if (opencv_direct_gpu) {
            // d_bgr_ sudah selesai karena stream disinkronkan di atas. HighGUI
            // OpenGL menerima GpuMat sehingga preview tidak memerlukan salinan
            // Device-to-Host.
            cv::cuda::GpuMat gpu_view(
              height_, width_, CV_8UC3, d_bgr_, static_cast<size_t>(width_) * 3U);
            cv::imshow(window_name, gpu_view);
          } else if (h_annotated_) {
            cv::Mat host_view(height_, width_, CV_8UC3, h_annotated_);
            cv::imshow(window_name, host_view);
          }
          const int key = cv::waitKey(1);
          if (key == 27 || key == 'q' || key == 'Q') {
            running_.store(false, std::memory_order_release);
            if (rclcpp::ok()) rclcpp::shutdown();
          }
        } catch (const cv::Exception &e) {
          if (opencv_cuda_opengl_active_) {
            RCLCPP_WARN(
              get_logger(),
              "CUDA -> OpenGL display gagal (%s); beralih ke pinned-host fallback.",
              e.what());
            opencv_cuda_opengl_active_ = false;
          } else {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 5000,
              "OpenCV display gagal (%s); viewer dinonaktifkan", e.what());
            show_opencv_cuda_window_ = false;
          }
        }
      }
    }
    if (need_drivable) publishImage(drivable_pub_, h_drivable_mask_, "mono8", 1, stamp);
    if (need_lane) publishImage(lane_pub_, h_lane_mask_, "mono8", 1, stamp);
    if (need_class) publishImage(class_mask_pub_, h_class_mask_, "mono8", 1, stamp);
    if (publish_detection_messages) publishDetections(stamp);
    if (integrated_runtime_enabled_ && need_detection_host) publishRawDetectionCalibration(stamp);
    if (need_obstacle_metrics) publishObstacleMetrics(stamp);
    if (subscriberNeeded(camera_info_pub_, true)) publishCameraInfo(stamp);
  }

  // Opsi debug: menulis class-id/confidence di CPU. Dinonaktifkan pada launch utama.
  // Fungsi: Menambahkan label teks debug di CPU; dinonaktifkan pada autonomous untuk menjaga throughput.
  void drawCpuLabels() {
    cv::Mat image(height_, width_, CV_8UC3, h_annotated_);
    const int count = std::min(*h_detection_count_, max_detections_);
    for (int i = 0; i < count; ++i) {
      const auto &d = h_detections_[i];
      std::ostringstream label;
      label << d.class_id << ' ' << std::fixed << std::setprecision(2) << d.confidence;
      cv::putText(image, label.str(), cv::Point(static_cast<int>(d.x1), std::max(15, static_cast<int>(d.y1) - 4)),
                  cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
    }
  }

  template<typename PublisherT>
  void publishImage(
    const std::shared_ptr<PublisherT> &publisher,
    const uint8_t *data,
    const std::string &encoding,
    int channels,
    const rclcpp::Time &stamp)
  {
    auto message = std::make_unique<sensor_msgs::msg::Image>();
    message->header.stamp = stamp;
    message->header.frame_id = frame_id_;
    message->height = static_cast<uint32_t>(height_);
    message->width = static_cast<uint32_t>(width_);
    message->encoding = encoding;
    message->is_bigendian = false;
    message->step = static_cast<uint32_t>(width_ * channels);
    message->data.resize(static_cast<size_t>(message->step) * height_);
    std::memcpy(message->data.data(), data, message->data.size());
    publisher->publish(std::move(message));
  }

  // Mengubah DetectionGpu menjadi vision_msgs/Detection2DArray ROS 2 Humble.
  // Fungsi: Mengubah bbox final menjadi vision_msgs Detection2DArray saat subscriber membutuhkannya.
  void publishDetections(const rclcpp::Time &stamp) {
    vision_msgs::msg::Detection2DArray message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id_;
    const int count = std::min(*h_detection_count_, max_detections_);
    message.detections.reserve(static_cast<size_t>(count));
    for (int i = 0; i < count; ++i) {
      const auto &source = h_detections_[i];
      vision_msgs::msg::Detection2D detection;
      detection.header = message.header;
      // Pada vision_msgs ROS 2 Humble, BoundingBox2D.center bertipe
      // vision_msgs/Pose2D. Koordinat pusat disimpan pada field position.
      detection.bbox.center.position.x = (source.x1 + source.x2) * 0.5;
      detection.bbox.center.position.y = (source.y1 + source.y2) * 0.5;
      detection.bbox.center.theta = 0.0;
      detection.bbox.size_x = std::max(0.0F, source.x2 - source.x1);
      detection.bbox.size_y = std::max(0.0F, source.y2 - source.y1);
      vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
      hypothesis.hypothesis.class_id = std::to_string(source.class_id);
      hypothesis.hypothesis.score = source.confidence;
      detection.results.push_back(std::move(hypothesis));
      message.detections.push_back(std::move(detection));
    }
    detections_pub_->publish(std::move(message));
  }

  // Mempublikasikan parameter intrinsik kamera. Ganti dengan hasil kalibrasi nyata.
  // Fungsi: Menerbitkan metadata intrinsics kamera yang disesuaikan dengan resolusi aktif.
  void publishCameraInfo(const rclcpp::Time &stamp) {
    sensor_msgs::msg::CameraInfo info;
    info.header.stamp = stamp;
    info.header.frame_id = frame_id_;
    info.width = static_cast<uint32_t>(width_);
    info.height = static_cast<uint32_t>(height_);
    info.distortion_model = "plumb_bob";
    info.d = camera_distortion_;
    const double fx = camera_fx_;
    const double fy = camera_fy_;
    const double cx = camera_cx_;
    const double cy = camera_cy_;
    info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
    info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    info.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
    camera_info_pub_->publish(std::move(info));
  }

  // Fungsi: Menghitung jumlah byte satu frame BGR aktif.
  size_t imageBytes() const { return static_cast<size_t>(width_) * height_ * 3U; }
  // Fungsi: Menghitung jumlah byte satu mask mono8 aktif.
  size_t maskBytes() const { return static_cast<size_t>(width_) * height_; }

  template<typename T>
  // Fungsi: Melepas pointer VRAM CUDA bila teralokasi lalu men-null-kan pointer.
  static void freeCudaDevice(T *&pointer) {
    if (pointer) {
      cudaFree(static_cast<void *>(pointer));
      pointer = nullptr;
    }
  }

  template<typename T>
  // Fungsi: Melepas pinned host memory CUDA bila teralokasi lalu men-null-kan pointer.
  static void freeCudaHost(T *&pointer) {
    if (pointer) {
      cudaFreeHost(static_cast<void *>(pointer));
      pointer = nullptr;
    }
  }

  void releaseBuffers() {
    freeCudaDevice(d_bgr_);
    if (d_capture_staging_) {
      cudaFree(d_capture_staging_);
      d_capture_staging_ = nullptr;
    }
    freeCudaDevice(d_input_);
    freeCudaDevice(d_drivable_mask_);
    freeCudaDevice(d_lane_mask_);
    freeCudaDevice(d_class_mask_);
    freeCudaDevice(d_candidates_);
    freeCudaDevice(d_candidate_count_);
    freeCudaDevice(d_final_detections_);
    freeCudaDevice(d_final_count_);
    freeCudaDevice(d_camera_health_stats_);
    freeCudaDevice(d_ground_homography_);
    freeCudaDevice(d_metric_detections_);
    freeCudaDevice(d_lane_left_mm_);
    freeCudaDevice(d_lane_right_mm_);
    freeCudaDevice(d_drivable_left_mm_);
    freeCudaDevice(d_drivable_right_mm_);
    freeCudaDevice(d_drivable_count_);

    for (auto &slot : rviz_frame_slots_) {
      freeCudaHost(slot.host);
      slot.state = RvizFrameState::FREE;
    }
    rviz_frame_slots_.clear();
    freeCudaHost(h_annotated_);
    freeCudaHost(h_raw_);
    freeCudaHost(h_drivable_mask_);
    freeCudaHost(h_lane_mask_);
    freeCudaHost(h_class_mask_);
    freeCudaHost(h_detections_);
    freeCudaHost(h_detection_count_);
    freeCudaHost(h_camera_health_stats_);
  }

  std::string engine_path_;
  std::string rgb_device_;
  std::string pixel_format_;
  std::string frame_id_;
  std::string annotated_topic_;
  std::string raw_topic_;
  std::string camera_info_topic_;
  std::string detections_topic_;
  std::string drivable_mask_topic_;
  std::string lane_mask_topic_;
  std::string lane_metrics_topic_;
  std::string drivable_space_topic_;
  std::string drivable_boundary_points_topic_;
  std::string obstacle_metrics_topic_;
  std::string performance_topic_{"/perception/performance"};
  std::string metric_frame_id_;
  std::string class_mask_topic_;
  int gpu_device_{0};
  int width_{640};
  int height_{480};
  int fps_{30};
  int model_width_{640};
  int model_height_{384};
  int v4l2_buffer_count_{3};
  bool allow_resolution_fallback_for_fps_{false};
  bool strict_camera_mode_{true};
  bool camera_hotplug_retry_{true};
  double camera_retry_interval_sec_{2.0};
  double camera_retry_log_interval_sec_{30.0};
  int max_candidates_{4096};
  int max_detections_{300};
  int warmup_iterations_{5};
  int box_thickness_{2};
  bool allow_mjpeg_{false};
  bool use_userptr_{true};
  bool flip_horizontal_{true};
  bool publish_annotated_{true};
  bool publish_raw_{false};
  bool publish_drivable_mask_{false};
  bool publish_lane_mask_{true};
  bool publish_class_mask_{false};
  bool publish_detections_{false};
  bool publish_lane_metrics_{true};
  bool publish_obstacle_metrics_{true};
  bool publish_only_when_subscribed_{true};
  bool show_opencv_cuda_window_{false};
  bool opencv_use_cuda_opengl_{true};
  bool opencv_window_fullscreen_{false};
  bool opencv_window_initialized_{false};
  bool opencv_cuda_opengl_active_{false};
  int opencv_window_width_{640};
  int opencv_window_height_{480};
  bool draw_text_labels_cpu_{false};
  bool async_rviz_publish_{true};
  std::string publisher_reliability_{"reliable"};
  int rviz_publish_buffer_count_{2};
  double rviz_max_publish_rate_hz_{10.0};
  std::chrono::steady_clock::time_point last_rviz_publish_time_{};
  float confidence_threshold_{0.30F};
  float iou_threshold_{0.45F};
  float lane_threshold_{0.50F};
  float overlay_alpha_{0.45F};
  float gain_{1.0F};
  float pad_x_{0.0F};
  float pad_y_{0.0F};
  double camera_fx_{910.0};
  double camera_fy_{910.0};
  double camera_cx_{640.0};
  double camera_cy_{360.0};
  std::vector<double> camera_distortion_{0.0, 0.0, 0.0, 0.0, 0.0};

  int ground_calibration_width_{1280};
  int ground_calibration_height_{720};
  std::vector<double> ground_src_points_;
  std::vector<double> ground_dst_points_;
  int ground_canvas_width_{1280};
  int ground_canvas_height_{720};
  bool camera_metric_calibration_validated_{false};
  float ground_origin_x_{596.8667F};
  float ground_origin_y_{719.0F};
  float ground_scale_x_{0.003909304F};
  float ground_scale_y_{0.009735744F};
  float metric_forward_offset_{0.0F};
  float metric_lateral_offset_{0.0F};
  float metric_min_forward_{0.20F};
  float metric_max_forward_{4.0F};
  float metric_max_abs_left_{2.5F};
  float lane_vehicle_width_{0.55F};
  float lane_min_width_{3.5F};
  float lane_max_width_{6.5F};
  float lane_near_{1.0F};
  float lane_far_{3.0F};
  float lane_control_lookahead_{2.0F};
  int lane_sample_rows_{72};
  int lane_minimum_rows_{18};
  float lane_min_center_offset_{0.08F};
  float lane_ema_alpha_{0.35F};
  float drivable_space_near_{0.35F};
  float drivable_space_far_{4.0F};
  float drivable_space_max_abs_left_{2.8F};
  int drivable_space_sample_rows_{64};
  int drivable_space_min_pixels_per_row_{8};
  int drivable_space_pixel_stride_px_{4};
  double drivable_boundary_guard_offset_m_{0.0};
  double drivable_boundary_z_m_{0.20};
  double drivable_lane_constraint_min_confidence_{0.25};
  bool lane_ema_initialized_{false};
  float lane_ema_left_clearance_{0.0F};
  float lane_ema_right_clearance_{0.0F};
  float lane_ema_center_error_{0.0F};
  float lane_ema_heading_error_{0.0F};
  float lane_ema_road_width_{0.0F};
  float lane_ema_confidence_{0.0F};
  std::vector<float> ground_homography_host_;

  // ------------------------------------------------------------------------
  // STATE RUNTIME PERCEPTION C++ TERINTEGRASI
  // ------------------------------------------------------------------------
  bool integrated_runtime_enabled_{true};
  bool lane_safety_enabled_{false};
  std::string lane_state_topic_{"/perception/lane_safety_state"};
  std::string lane_centerline_topic_{"/perception/lane_centerline"};
  std::string lane_safety_markers_topic_{"/perception/lane_safety_markers"};
  double nominal_road_width_m_{5.0};
  safety::LaneThresholds lane_thresholds_{};
  double minimum_metric_confidence_{0.10};
  int state_confirm_frames_{3};
  int release_confirm_frames_{5};
  int lost_confirm_frames_{2};
  std::unique_ptr<safety::ConfirmedState> lane_state_filter_;

  std::string object_points_topic_{"/perception/object_points"};
  std::string object_clearing_points_topic_{"/perception/object_clearing_points"};
  std::string obstacle_markers_topic_{"/perception/obstacle_markers"};
  std::string perception_obstacle_metrics_topic_{"/perception/obstacle_metrics"};
  std::string raw_detection_summary_topic_{"/perception/raw_detections"};
  std::string raw_detection_markers_topic_{"/perception/raw_detection_markers"};
  double minimum_obstacle_confidence_{0.30};
  double raw_detection_log_period_sec_{0.50};
  rclcpp::Time last_raw_detection_log_time_{0, 0, RCL_ROS_TIME};
  bool accept_all_detected_classes_as_obstacles_{false};
  std::vector<int64_t> safety_obstacle_class_ids_{0, 2, 3};
  bool require_drivable_contact_{true};
  int drivable_contact_radius_px_{12};
  int drivable_contact_vertical_tolerance_px_{8};
  int drivable_contact_min_samples_{3};
  float drivable_contact_min_fraction_{0.20F};
  int points_per_box_{5};
  double obstacle_cloud_max_publish_rate_hz_{12.0};
  rclcpp::Time last_obstacle_cloud_pub_time_{0, 0, RCL_ROS_TIME};
  double clearing_fov_rad_{100.0 * M_PI / 180.0};
  double clearing_range_m_{4.5};
  int clearing_ray_count_{41};
  double clearing_obstacle_margin_m_{0.20};
  double track_match_distance_m_{0.90};
  double track_ema_alpha_{0.45};
  int track_confirm_hits_{3};
  int track_max_missed_frames_{8};
  int next_track_id_{1};
  int last_metric_obstacle_count_{0};
  int last_confirmed_obstacle_count_{0};
  std::vector<ObstacleTrackRuntime> obstacle_tracks_;

  bool camera_health_enabled_{true};
  std::string camera_health_topic_{"/perception/camera_healthy"};
  std::string camera_health_state_topic_{"/perception/camera_health_state"};
  int camera_health_sample_stride_px_{16};
  int camera_health_dark_luma_{18};
  int camera_health_bright_luma_{245};
  double camera_health_extreme_fraction_{0.97};
  double camera_health_min_stddev_{5.0};
  double camera_health_min_gradient_{2.0};
  int camera_health_bad_confirm_frames_{5};
  int camera_health_good_confirm_frames_{5};
  bool camera_healthy_latched_{false};
  int camera_health_bad_count_{0};
  int camera_health_good_count_{0};

  bool near_field_emergency_enabled_{true};
  std::string emergency_stop_topic_{"/perception/emergency_stop"};
  std::string near_field_state_topic_{"/perception/near_field_state"};
  double near_field_min_confidence_{0.25};
  double near_field_center_corridor_fraction_{0.65};
  double near_field_min_bbox_height_fraction_{0.20};
  bool near_field_drivable_guard_enabled_{true};
  double near_field_bottom_roi_fraction_{0.22};
  double near_field_min_drivable_fraction_{0.12};
  int near_field_confirm_frames_{2};
  int near_field_release_frames_{5};
  bool near_field_emergency_latched_{false};
  int near_field_hit_count_{0};
  int near_field_clear_count_{0};

  std::string control_mode_{"active"};
  std::string nav_cmd_topic_{"/cmd_vel_nav_smoothed"};
  std::string safe_cmd_topic_{"/cmd_vel/perception_advisory"};
  std::string control_state_topic_{"/perception/lane_control_state"};
  std::string control_markers_topic_{"/perception/lane_control_markers"};
  double control_rate_hz_{30.0};
  double cmd_timeout_sec_{0.50};
  double lane_state_timeout_sec_{0.50};
  double obstacle_state_timeout_sec_{0.70};
  safety::MixerConfig mixer_config_{};
  safety::ObstacleGateConfig obstacle_gate_config_{};
  int blocked_confirm_frames_{2};
  int clear_confirm_frames_{5};
  bool blocked_latched_{false};
  int blocked_count_{0};
  int clear_count_{0};
  uint64_t control_sequence_{0};

  std::mutex perception_state_mutex_;
  geometry_msgs::msg::Twist last_nav_cmd_;
  LaneGeometryRuntime latest_lane_geometry_;
  std::vector<MetricDetectionRuntime> latest_obstacles_;
  std::string latest_lane_state_{safety::LANE_LOST};
  std::string latest_lane_candidate_state_{safety::LANE_LOST};
  std::string latest_lane_reason_{"startup"};
  bool latest_lane_valid_{false};
  bool latest_lane_critical_{false};
  bool have_nav_cmd_{false};
  bool have_lane_{false};
  bool have_obstacles_{false};
  std::chrono::steady_clock::time_point last_nav_cmd_time_{};
  std::chrono::steady_clock::time_point last_lane_time_{};
  std::chrono::steady_clock::time_point last_obstacle_time_{};

  V4L2Capture camera_;
  TensorRtEngine engine_;
  cudaStream_t stream_{nullptr};
  uint8_t *d_bgr_{nullptr};
  void *d_capture_staging_{nullptr};
  float *d_input_{nullptr};
  uint8_t *d_drivable_mask_{nullptr};
  uint8_t *d_lane_mask_{nullptr};
  uint8_t *d_class_mask_{nullptr};
  DetectionGpu *d_candidates_{nullptr};
  int *d_candidate_count_{nullptr};
  DetectionGpu *d_final_detections_{nullptr};
  int *d_final_count_{nullptr};
  unsigned long long *d_camera_health_stats_{nullptr};
  float *d_ground_homography_{nullptr};
  MetricDetectionGpu *d_metric_detections_{nullptr};
  int *d_lane_left_mm_{nullptr};
  int *d_lane_right_mm_{nullptr};
  int *d_drivable_left_mm_{nullptr};
  int *d_drivable_right_mm_{nullptr};
  int *d_drivable_count_{nullptr};
  uint8_t *h_annotated_{nullptr};
  uint8_t *h_raw_{nullptr};
  uint8_t *h_drivable_mask_{nullptr};
  uint8_t *h_lane_mask_{nullptr};
  uint8_t *h_class_mask_{nullptr};
  DetectionGpu *h_detections_{nullptr};
  int *h_detection_count_{nullptr};
  unsigned long long *h_camera_health_stats_{nullptr};
  std::vector<MetricDetectionGpu> h_metric_detections_;
  std::vector<int> h_lane_left_mm_;
  std::vector<int> h_lane_right_mm_;
  std::vector<int> h_drivable_left_mm_;
  std::vector<int> h_drivable_right_mm_;
  std::vector<int> h_drivable_count_;

  std::vector<RvizFrameSlot> rviz_frame_slots_;
  std::mutex rviz_frame_mutex_;
  std::condition_variable rviz_frame_cv_;
  uint64_t rviz_frame_sequence_{0};
  std::atomic<uint64_t> published_rviz_frames_{0};
  std::atomic<uint64_t> dropped_rviz_frames_{0};
  std::atomic<uint64_t> dropped_capture_frames_{0};

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr camera_connected_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr drivable_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr lane_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lane_metrics_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr drivable_space_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr drivable_boundary_points_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr obstacle_metrics_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr performance_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr class_mask_pub_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lane_state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr lane_centerline_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lane_safety_markers_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr object_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr object_clearing_points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_markers_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr raw_detection_summary_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr raw_detection_markers_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr perception_obstacle_metrics_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr camera_health_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr camera_health_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr near_field_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_state_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr control_markers_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_cmd_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::atomic<bool> local_costmap_ready_{false};
  std::atomic<bool> running_{false};
  std::thread worker_;
  std::thread rviz_publisher_worker_;
};

}  // namespace perception (akhir implementasi paket)

// Entry point ROS 2. Constructor node dapat melempar exception bila CUDA,
// TensorRT engine, atau kamera tidak valid; error dicetak jelas ke terminal.
// Fungsi: Entry point node perception; init ROS, spin, dan shutdown pipeline secara deterministik.
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<perception::AstraYolopGpuNode>();
    if (rclcpp::ok()) {
      rclcpp::spin(node);
    }
  } catch (const std::exception &e) {
    std::fprintf(stderr, "[perception] fatal: %s\n", e.what());
    exit_code = 1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return exit_code;
}
