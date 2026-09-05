#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {
using Samples = std::map<std::string, std::vector<std::pair<double, double>>>;

struct FitResult {
  double a{0.0};
  double b{0.0};
  double mae{0.0};
  double rmse{0.0};
};

// Menghapus whitespace awal/akhir tanpa mengubah isi field CSV.
// Fungsi: Menghapus whitespace tepi saat membaca CSV/YAML teks.
std::string trim(const std::string &text) {
  const size_t first = text.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return {};
  const size_t last = text.find_last_not_of(" \t\r\n");
  return text.substr(first, last - first + 1);
}

// Mengubah teks ke lower-case agar nama class dari beberapa sesi tetap tergabung.
// Fungsi: Menormalkan nama class agar grouping dataset konsisten.
std::string lowerCopy(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

// Parser CSV kecil yang memahami quote ganda; cukup untuk file logger bawaan.
// Fungsi: Memecah baris CSV dengan dukungan quoted-field sederhana.
std::vector<std::string> splitCsv(const std::string &line) {
  std::vector<std::string> fields;
  std::string field;
  bool quoted = false;
  for (size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (c == '"') {
      if (quoted && i + 1 < line.size() && line[i + 1] == '"') {
        field.push_back('"');
        ++i;
      } else {
        quoted = !quoted;
      }
    } else if (c == ',' && !quoted) {
      fields.push_back(field);
      field.clear();
    } else {
      field.push_back(c);
    }
  }
  fields.push_back(field);
  return fields;
}

// Membaca hanya kolom target_class, area_px, dan actual_forward_m yang diperlukan fit.
// Fungsi: Membaca dataset logger dan mengelompokkan pasangan area-pixel/jarak per class.
Samples loadRows(const std::filesystem::path &path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("Tidak dapat membuka CSV: " + path.string());
  std::string line;
  if (!std::getline(in, line)) return {};
  const auto header = splitCsv(line);
  std::unordered_map<std::string, size_t> index;
  for (size_t i = 0; i < header.size(); ++i) index[trim(header[i])] = i;
  for (const char *required : {"target_class", "area_px", "actual_forward_m"}) {
    if (!index.count(required)) throw std::runtime_error(std::string("Kolom CSV hilang: ") + required);
  }

  Samples grouped;
  while (std::getline(in, line)) {
    const auto fields = splitCsv(line);
    const size_t largest = std::max({index["target_class"], index["area_px"], index["actual_forward_m"]});
    if (fields.size() <= largest) continue;
    try {
      const std::string class_name = lowerCopy(trim(fields[index["target_class"]]));
      const double area = std::stod(trim(fields[index["area_px"]]));
      const double distance = std::stod(trim(fields[index["actual_forward_m"]]));
      if (!class_name.empty() && std::isfinite(area) && std::isfinite(distance) && area > 1.0 && distance > 0.0) {
        grouped[class_name].push_back({area, distance});
      }
    } catch (const std::exception &) {
      // Baris parsial/korup diabaikan agar satu sesi buruk tidak menggagalkan seluruh dataset.
    }
  }
  return grouped;
}

// Least-squares analitik untuk d = a/sqrt(area) + b tanpa NumPy/BLAS dependency.
// Fungsi: Melakukan least-squares model distance=a/sqrt(area)+b dan menghitung MAE/RMSE.
FitResult fit(const std::vector<std::pair<double, double>> &samples) {
  const double n = static_cast<double>(samples.size());
  double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
  for (const auto &[area, distance] : samples) {
    const double x = 1.0 / std::sqrt(area);
    sx += x; sy += distance; sxx += x * x; sxy += x * distance;
  }
  const double denominator = n * sxx - sx * sx;
  if (std::abs(denominator) < 1.0e-15) {
    throw std::runtime_error("Variasi area bbox terlalu kecil untuk melakukan fit");
  }
  FitResult result;
  result.a = (n * sxy - sx * sy) / denominator;
  result.b = (sy - result.a * sx) / n;
  double absolute = 0.0, squared = 0.0;
  for (const auto &[area, distance] : samples) {
    const double prediction = result.a / std::sqrt(area) + result.b;
    const double error = prediction - distance;
    absolute += std::abs(error);
    squared += error * error;
  }
  result.mae = absolute / n;
  result.rmse = std::sqrt(squared / n);
  return result;
}

// Membaca template sebagai baris agar komentar dan setting runtime tidak hilang.
// Fungsi: Membaca file konfigurasi menjadi baris-baris agar patch model mempertahankan struktur lain.
std::vector<std::string> readLines(const std::filesystem::path &path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("Tidak dapat membuka template: " + path.string());
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(in, line)) lines.push_back(line);
  return lines;
}

// Memperbarui/memasukkan model di bawah area_models sambil mempertahankan YAML lain.
void upsertAreaModel(
  std::vector<std::string> &lines, const std::string &class_name,
  const FitResult &fit_result, size_t sample_count)
{
  size_t area_index = lines.size();
  for (size_t i = 0; i < lines.size(); ++i) {
    if (trim(lines[i]) == "area_models:") { area_index = i; break; }
  }
  if (area_index == lines.size()) {
    lines.push_back("");
    lines.push_back("area_models:");
    area_index = lines.size() - 1;
  }

  size_t section_end = lines.size();
  size_t existing = lines.size();
  const std::string prefix = "  " + class_name + ":";
  for (size_t i = area_index + 1; i < lines.size(); ++i) {
    const std::string &line = lines[i];
    if (!line.empty() && line[0] != ' ' && line[0] != '#') { section_end = i; break; }
    if (line.rfind(prefix, 0) == 0) existing = i;
  }

  std::ostringstream replacement;
  replacement << std::setprecision(10)
              << "  " << class_name << ": {enabled: true, a: " << fit_result.a
              << ", b: " << fit_result.b << ", sample_count: " << sample_count
              << ", mae_m: " << std::setprecision(8) << fit_result.mae
              << ", rmse_m: " << fit_result.rmse << "}";
  if (existing < lines.size()) lines[existing] = replacement.str();
  else lines.insert(lines.begin() + static_cast<std::ptrdiff_t>(section_end), replacement.str());
}

// Menampilkan penggunaan command-line C++ pengganti fit_bbox_calibration.py.
// Fungsi: Menampilkan cara memakai fitter standalone dari terminal.
void printUsage(const char *program) {
  std::cerr << "Usage: " << program
            << " --csv FILE --template FILE --output FILE [--minimum-samples N]\n";
}

}  // namespace

// Entry point standalone: fit semua class yang punya sample cukup lalu tulis YAML hasil.
// Fungsi: Entry point fitter; parse argumen, fit tiap class, dan tulis model ke config tujuan.
int main(int argc, char **argv) {
  try {
    std::filesystem::path csv, template_path, output;
    int minimum_samples = 6;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if ((arg == "--csv" || arg == "--template" || arg == "--output" || arg == "--minimum-samples") && i + 1 >= argc) {
        throw std::invalid_argument("Nilai hilang setelah " + arg);
      }
      if (arg == "--csv") csv = argv[++i];
      else if (arg == "--template") template_path = argv[++i];
      else if (arg == "--output") output = argv[++i];
      else if (arg == "--minimum-samples") minimum_samples = std::max(2, std::stoi(argv[++i]));
      else { printUsage(argv[0]); throw std::invalid_argument("Argumen tidak dikenal: " + arg); }
    }
    if (csv.empty() || template_path.empty() || output.empty()) {
      printUsage(argv[0]);
      return 2;
    }

    const Samples grouped = loadRows(csv);
    if (grouped.empty()) throw std::runtime_error("Tidak ada row kalibrasi valid dalam CSV");
    auto lines = readLines(template_path);
    int fitted = 0;
    for (const auto &[class_name, samples] : grouped) {
      if (static_cast<int>(samples.size()) < minimum_samples) {
        std::cout << "SKIP " << class_name << ": " << samples.size()
                  << " sample < " << minimum_samples << '\n';
        continue;
      }
      const FitResult result = fit(samples);
      upsertAreaModel(lines, class_name, result, samples.size());
      ++fitted;
      std::cout << std::fixed << std::setprecision(6) << class_name
                << ": distance=" << result.a << "/sqrt(area)+" << result.b
                << ", n=" << samples.size() << ", MAE=" << std::setprecision(4)
                << result.mae << "m, RMSE=" << result.rmse << "m\n";
    }
    if (fitted == 0) throw std::runtime_error("Tidak ada class dengan sample yang cukup; output tidak ditulis");

    if (output.has_parent_path()) std::filesystem::create_directories(output.parent_path());
    std::ofstream out(output);
    if (!out) throw std::runtime_error("Tidak dapat menulis output: " + output.string());
    for (const auto &line : lines) out << line << '\n';
    std::cout << "Calibration tertulis: " << output << '\n';
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "fit_bbox_calibration gagal: " << error.what() << '\n';
    return 1;
  }
}
