#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct DetectionSample {
  std::string class_name;
  int track_id{-1};
  double score{0.0};
  double center_x_px{0.0};
  double center_y_px{0.0};
  double bottom_y_px{0.0};
  double width_px{0.0};
  double height_px{0.0};
  double area_px{0.0};
  std::optional<double> forward_homography_m;
  std::optional<double> forward_area_m;
  double forward_m{0.0};
  double left_m{0.0};
};

// Mengubah string menjadi huruf kecil untuk pencocokan nama kelas yang stabil.
// Fungsi: Menormalkan teks class/filter menjadi huruf kecil.
std::string lowerCopy(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

// Mengambil isi string JSON untuk key tertentu dari object datar YOLOP.
// Fungsi: Mengambil field string sederhana dari object JSON metrics tanpa dependency parser tambahan.
std::optional<std::string> jsonString(const std::string &object, const std::string &key) {
  const std::string needle = "\"" + key + "\":";
  const size_t key_pos = object.find(needle);
  if (key_pos == std::string::npos) return std::nullopt;
  size_t pos = key_pos + needle.size();
  while (pos < object.size() && std::isspace(static_cast<unsigned char>(object[pos]))) ++pos;
  if (pos >= object.size() || object[pos] != '"') return std::nullopt;
  ++pos;
  std::string result;
  bool escaped = false;
  for (; pos < object.size(); ++pos) {
    const char c = object[pos];
    if (escaped) {
      result.push_back(c);
      escaped = false;
    } else if (c == '\\') {
      escaped = true;
    } else if (c == '"') {
      return result;
    } else {
      result.push_back(c);
    }
  }
  return std::nullopt;
}

// Mengambil angka JSON untuk key tertentu; null dikembalikan sebagai nullopt.
// Fungsi: Mengambil field numerik finite dari object JSON metrics.
std::optional<double> jsonNumber(const std::string &object, const std::string &key) {
  const std::string needle = "\"" + key + "\":";
  const size_t key_pos = object.find(needle);
  if (key_pos == std::string::npos) return std::nullopt;
  const char *start = object.c_str() + key_pos + needle.size();
  while (*start != '\0' && std::isspace(static_cast<unsigned char>(*start))) ++start;
  if (std::strncmp(start, "null", 4) == 0) return std::nullopt;
  char *end = nullptr;
  const double value = std::strtod(start, &end);
  if (end == start || !std::isfinite(value)) return std::nullopt;
  return value;
}

// Memecah array detections JSON menjadi object-object datar tanpa dependency JSON tambahan.
// Fungsi: Memisahkan array detections JSON menjadi object-object untuk proses kalibrasi.
std::vector<std::string> detectionObjects(const std::string &json) {
  std::vector<std::string> objects;
  const size_t key = json.find("\"detections\"");
  if (key == std::string::npos) return objects;
  const size_t array_begin = json.find('[', key);
  if (array_begin == std::string::npos) return objects;

  int depth = 0;
  bool in_string = false;
  bool escaped = false;
  size_t object_begin = std::string::npos;
  for (size_t i = array_begin + 1; i < json.size(); ++i) {
    const char c = json[i];
    if (in_string) {
      if (escaped) escaped = false;
      else if (c == '\\') escaped = true;
      else if (c == '"') in_string = false;
      continue;
    }
    if (c == '"') {
      in_string = true;
    } else if (c == '{') {
      if (depth == 0) object_begin = i;
      ++depth;
    } else if (c == '}') {
      --depth;
      if (depth == 0 && object_begin != std::string::npos) {
        objects.push_back(json.substr(object_begin, i - object_begin + 1));
        object_begin = std::string::npos;
      }
    } else if (c == ']' && depth == 0) {
      break;
    }
  }
  return objects;
}

// Mengubah satu object JSON runtime menjadi sample terstruktur untuk CSV.
// Fungsi: Memvalidasi satu detection dan mengubahnya menjadi record bbox/homography terstruktur.
std::optional<DetectionSample> parseDetection(const std::string &object) {
  const auto class_name = jsonString(object, "class_name");
  const auto score = jsonNumber(object, "score");
  const auto forward = jsonNumber(object, "forward_m");
  const auto left = jsonNumber(object, "left_m");
  if (!class_name || !score || !forward || !left) return std::nullopt;

  DetectionSample sample;
  sample.class_name = *class_name;
  sample.score = *score;
  sample.forward_m = *forward;
  sample.left_m = *left;
  if (const auto v = jsonNumber(object, "track_id")) sample.track_id = static_cast<int>(*v);
  if (const auto v = jsonNumber(object, "center_x_px")) sample.center_x_px = *v;
  if (const auto v = jsonNumber(object, "center_y_px")) sample.center_y_px = *v;
  if (const auto v = jsonNumber(object, "bottom_y_px")) sample.bottom_y_px = *v;
  if (const auto v = jsonNumber(object, "width_px")) sample.width_px = *v;
  if (const auto v = jsonNumber(object, "height_px")) sample.height_px = *v;
  if (const auto v = jsonNumber(object, "area_px")) sample.area_px = *v;
  sample.forward_homography_m = jsonNumber(object, "forward_homography_m");
  sample.forward_area_m = jsonNumber(object, "forward_area_m");
  return sample;
}

// Menulis field CSV sederhana dan melakukan quote bila teks mengandung delimiter.
// Fungsi: Melakukan escaping satu field CSV agar label/test-id aman disimpan.
std::string csvField(const std::string &value) {
  if (value.find_first_of(",\"\n\r") == std::string::npos) return value;
  std::string escaped = "\"";
  for (char c : value) escaped += (c == '"' ? "\"\"" : std::string(1, c));
  escaped += '"';
  return escaped;
}

// Memformat angka secara konsisten agar hasil kalibrasi mudah diproses ulang.
// Fungsi: Mengubah angka finite menjadi teks CSV dengan presisi konsisten.
std::string numberField(double value, int precision = 6) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(precision) << value;
  return out.str();
}

}  // namespace

class BboxStaticTestLogger final : public rclcpp::Node {
public:
  // Membaca parameter pengujian, menyiapkan CSV, lalu subscribe metrics C++ runtime.
  // Fungsi: Menginisialisasi parameter target, file CSV, dan subscriber obstacle metrics.
  BboxStaticTestLogger() : Node("bbox_static_test_logger") {
    declare_parameter<std::string>("metrics_topic", "/perception/obstacle_metrics");
    declare_parameter<std::string>("output_csv", "~/bbox_static_test.csv");
    declare_parameter<std::string>("test_id", "static_test");
    declare_parameter<std::string>("target_class", "");
    declare_parameter<double>("actual_forward_m", -1.0);
    declare_parameter<double>("actual_left_m", 0.0);
    declare_parameter<double>("minimum_score", 0.30);
    declare_parameter<double>("sample_period_sec", 0.20);
    declare_parameter<int>("max_samples", 50);

    topic_ = get_parameter("metrics_topic").as_string();
    output_path_ = expandHome(get_parameter("output_csv").as_string());
    test_id_ = get_parameter("test_id").as_string();
    target_class_ = lowerCopy(get_parameter("target_class").as_string());
    actual_forward_m_ = get_parameter("actual_forward_m").as_double();
    actual_left_m_ = get_parameter("actual_left_m").as_double();
    minimum_score_ = get_parameter("minimum_score").as_double();
    sample_period_sec_ = std::max(0.02, get_parameter("sample_period_sec").as_double());
    max_samples_ = std::max(1, static_cast<int>(get_parameter("max_samples").as_int()));
    if (!(actual_forward_m_ > 0.0)) {
      throw std::invalid_argument("actual_forward_m harus lebih besar dari nol");
    }

    if (output_path_.has_parent_path()) std::filesystem::create_directories(output_path_.parent_path());
    ensureHeader();
    subscription_ = create_subscription<std_msgs::msg::String>(
      topic_, rclcpp::QoS(10),
      std::bind(&BboxStaticTestLogger::onMetrics, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(), "Logging %d sample ke %s; test=%s class=%s actual=(%.3f, %.3f)",
      max_samples_, output_path_.c_str(), test_id_.c_str(),
      target_class_.empty() ? "any" : target_class_.c_str(), actual_forward_m_, actual_left_m_);
  }

private:
  // Mengembangkan awalan ~/ agar parameter path sama nyaman dengan versi Python lama.
  // Fungsi: Mengembangkan awalan ~/ pada path output kalibrasi.
  static std::filesystem::path expandHome(const std::string &path) {
    if (path.rfind("~/", 0) == 0) {
      if (const char *home = std::getenv("HOME")) return std::filesystem::path(home) / path.substr(2);
    }
    return std::filesystem::path(path);
  }

  // Membuat header hanya untuk file baru/kosong supaya beberapa jarak dapat di-append.
  // Fungsi: Membuat direktori/file CSV dan header bila dataset belum ada.
  void ensureHeader() const {
    if (std::filesystem::exists(output_path_) && std::filesystem::file_size(output_path_) > 0U) return;
    std::ofstream out(output_path_);
    if (!out) throw std::runtime_error("Tidak dapat membuat CSV: " + output_path_.string());
    out << "test_id,sample_time_unix,target_class,actual_forward_m,actual_left_m,"
        << "track_id,score,center_x_px,center_y_px,bottom_y_px,width_px,height_px,area_px,"
        << "forward_homography_m,forward_area_m,forward_m,left_m,forward_error_m,left_error_m\n";
  }

  // Memilih target paling yakin sesuai class dan minimum score seperti logger lama.
  // Fungsi: Memilih detection yang sesuai class dan ambang confidence untuk sample statik.
  std::optional<DetectionSample> selectDetection(const std::string &json) const {
    std::optional<DetectionSample> best;
    for (const auto &object : detectionObjects(json)) {
      auto sample = parseDetection(object);
      if (!sample || sample->score < minimum_score_) continue;
      if (!target_class_.empty() && lowerCopy(sample->class_name) != target_class_) continue;
      if (!best || sample->score > best->score) best = std::move(sample);
    }
    return best;
  }

  // Mencatat satu sample periodik; shutdown dilakukan setelah jumlah target tercapai.
  // Fungsi: Callback metrics; memilih bbox, menambahkan metadata jarak ground-truth, lalu append CSV.
  void onMetrics(const std_msgs::msg::String::SharedPtr message) {
    const auto steady_now = std::chrono::steady_clock::now();
    if (have_last_sample_ &&
        std::chrono::duration<double>(steady_now - last_sample_).count() < sample_period_sec_) return;

    const auto selected = selectDetection(message->data);
    if (!selected) return;
    const auto &d = *selected;
    const double unix_time = std::chrono::duration<double>(
      std::chrono::system_clock::now().time_since_epoch()).count();

    std::ofstream out(output_path_, std::ios::app);
    if (!out) {
      RCLCPP_ERROR(get_logger(), "Tidak dapat append CSV %s", output_path_.c_str());
      return;
    }
    out << csvField(test_id_) << ',' << numberField(unix_time) << ',' << csvField(d.class_name) << ','
        << numberField(actual_forward_m_) << ',' << numberField(actual_left_m_) << ','
        << d.track_id << ',' << numberField(d.score) << ',' << numberField(d.center_x_px) << ','
        << numberField(d.center_y_px) << ',' << numberField(d.bottom_y_px) << ','
        << numberField(d.width_px) << ',' << numberField(d.height_px) << ',' << numberField(d.area_px) << ',';
    if (d.forward_homography_m) out << numberField(*d.forward_homography_m);
    out << ',';
    if (d.forward_area_m) out << numberField(*d.forward_area_m);
    out << ',' << numberField(d.forward_m) << ',' << numberField(d.left_m) << ','
        << numberField(d.forward_m - actual_forward_m_) << ','
        << numberField(d.left_m - actual_left_m_) << '\n';

    last_sample_ = steady_now;
    have_last_sample_ = true;
    ++sample_count_;
    RCLCPP_INFO(
      get_logger(), "[%d/%d] %s area=%.1f x=%.3f y=%.3f",
      sample_count_, max_samples_, d.class_name.c_str(), d.area_px, d.forward_m, d.left_m);
    if (sample_count_ >= max_samples_) {
      RCLCPP_INFO(get_logger(), "Static-test selesai; logger berhenti.");
      rclcpp::shutdown();
    }
  }

  std::string topic_;
  std::filesystem::path output_path_;
  std::string test_id_;
  std::string target_class_;
  double actual_forward_m_{-1.0};
  double actual_left_m_{0.0};
  double minimum_score_{0.30};
  double sample_period_sec_{0.20};
  int max_samples_{50};
  int sample_count_{0};
  bool have_last_sample_{false};
  std::chrono::steady_clock::time_point last_sample_{};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

// Entry point ROS 2 untuk tool kalibrasi bbox statik.
// Fungsi: Entry point logger bbox ROS 2.
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<BboxStaticTestLogger>());
  } catch (const std::exception &error) {
    std::cerr << "bbox_static_test_logger gagal: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
