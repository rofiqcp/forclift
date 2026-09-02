#pragma once

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <climits>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

namespace agv_usb_serial
{

struct DeviceInfo
{
  std::string canonical;
  std::string preferred;
  std::vector<std::string> aliases;
  std::string tty_name;
  std::string driver;
  std::string manufacturer;
  std::string product;
  std::string serial;
  std::string vid;
  std::string pid;
};

inline std::string trim(std::string s)
{
  while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) s.erase(s.begin());
  while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.pop_back();
  return s;
}

inline std::string lower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return s;
}

inline std::string basename_of(const std::string & path)
{
  const auto pos = path.find_last_of('/');
  return pos == std::string::npos ? path : path.substr(pos + 1U);
}

inline std::string dirname_of(const std::string & path)
{
  const auto pos = path.find_last_of('/');
  if (pos == std::string::npos) return ".";
  if (pos == 0U) return "/";
  return path.substr(0U, pos);
}

inline std::string resolve_path(const std::string & path)
{
  char out[PATH_MAX] = {0};
  if (!path.empty() && ::realpath(path.c_str(), out) != nullptr) {
    return std::string(out);
  }
  return path;
}

inline std::string read_text_file(const std::string & path)
{
  std::ifstream f(path);
  if (!f.good()) return {};
  std::string s;
  std::getline(f, s);
  return trim(s);
}

inline std::string read_link_basename(const std::string & path)
{
  char buf[PATH_MAX] = {0};
  const ssize_t n = ::readlink(path.c_str(), buf, sizeof(buf) - 1U);
  if (n <= 0) return {};
  buf[n] = '\0';
  return basename_of(std::string(buf));
}

inline std::vector<std::string> directory_entries(const std::string & directory)
{
  std::vector<std::string> out;
  DIR * dir = ::opendir(directory.c_str());
  if (!dir) return out;
  while (auto * ent = ::readdir(dir)) {
    if (ent->d_name[0] == '.') continue;
    out.emplace_back(directory + "/" + ent->d_name);
  }
  ::closedir(dir);
  std::sort(out.begin(), out.end());
  return out;
}

inline void add_alias(std::map<std::string, DeviceInfo> & devices, const std::string & path)
{
  if (::access(path.c_str(), F_OK) != 0) return;
  const std::string canonical = resolve_path(path);
  const std::string tty = basename_of(canonical);
  if (tty.rfind("ttyUSB", 0U) != 0U && tty.rfind("ttyACM", 0U) != 0U) return;

  auto & d = devices[canonical];
  d.canonical = canonical;
  d.tty_name = tty;
  if (std::find(d.aliases.begin(), d.aliases.end(), path) == d.aliases.end()) {
    d.aliases.push_back(path);
  }
}

inline void fill_usb_identity(DeviceInfo & d)
{
  const std::string class_device = "/sys/class/tty/" + d.tty_name + "/device";
  std::string current = resolve_path(class_device);
  if (current.empty() || current == class_device) return;

  // Walk from tty interface to the parent USB device. Different usbserial
  // drivers expose idVendor/idProduct/manufacturer/product at different levels.
  for (int depth = 0; depth < 10 && current.size() > 1U; ++depth) {
    if (d.driver.empty()) {
      d.driver = read_link_basename(current + "/driver");
    }
    if (d.vid.empty()) d.vid = read_text_file(current + "/idVendor");
    if (d.pid.empty()) d.pid = read_text_file(current + "/idProduct");
    if (d.serial.empty()) d.serial = read_text_file(current + "/serial");
    if (d.manufacturer.empty()) d.manufacturer = read_text_file(current + "/manufacturer");
    if (d.product.empty()) d.product = read_text_file(current + "/product");

    const std::string parent = dirname_of(current);
    if (parent == current) break;
    current = parent;
  }
}

inline bool alias_has_prefix(const DeviceInfo & d, const std::string & prefix)
{
  return std::any_of(d.aliases.begin(), d.aliases.end(), [&](const std::string & a) {
    return a.rfind(prefix, 0U) == 0U;
  });
}

inline bool alias_contains(const DeviceInfo & d, const std::string & needle)
{
  const std::string n = lower(needle);
  return std::any_of(d.aliases.begin(), d.aliases.end(), [&](const std::string & a) {
    return lower(a).find(n) != std::string::npos;
  });
}

inline std::string identity_text(const DeviceInfo & d)
{
  std::ostringstream ss;
  ss << d.canonical << ' ' << d.tty_name << ' ' << d.driver << ' '
     << d.manufacturer << ' ' << d.product << ' ' << d.serial << ' '
     << d.vid << ':' << d.pid;
  for (const auto & a : d.aliases) ss << ' ' << a;
  return lower(ss.str());
}

inline std::vector<DeviceInfo> discover_devices()
{
  std::map<std::string, DeviceInfo> grouped;

  // Persistent Linux aliases first. They survive ttyUSB index swaps caused by
  // a hub and are preserved as the preferred path when available.
  for (const auto & p : directory_entries("/dev/serial/by-id")) add_alias(grouped, p);
  for (const auto & p : directory_entries("/dev/serial/by-path")) add_alias(grouped, p);
  add_alias(grouped, "/dev/imu_yahboom");
  add_alias(grouped, "/dev/lidar");
  add_alias(grouped, "/dev/ydlidar");
  add_alias(grouped, "/dev/esc");

  for (int i = 0; i < 32; ++i) {
    add_alias(grouped, "/dev/ttyUSB" + std::to_string(i));
    add_alias(grouped, "/dev/ttyACM" + std::to_string(i));
  }

  std::vector<DeviceInfo> out;
  out.reserve(grouped.size());
  for (auto & kv : grouped) {
    auto & d = kv.second;
    fill_usb_identity(d);

    // Prefer an explicit application alias, then by-id, then by-path, then tty.
    auto pick = [&](const std::string & exact) -> bool {
      auto it = std::find(d.aliases.begin(), d.aliases.end(), exact);
      if (it != d.aliases.end()) {
        d.preferred = *it;
        return true;
      }
      return false;
    };
    if (!pick("/dev/imu_yahboom") && !pick("/dev/lidar") && !pick("/dev/ydlidar")) {
      auto by_id = std::find_if(d.aliases.begin(), d.aliases.end(), [](const std::string & a) {
        return a.rfind("/dev/serial/by-id/", 0U) == 0U;
      });
      if (by_id != d.aliases.end()) d.preferred = *by_id;
    }
    if (d.preferred.empty()) {
      auto by_path = std::find_if(d.aliases.begin(), d.aliases.end(), [](const std::string & a) {
        return a.rfind("/dev/serial/by-path/", 0U) == 0U;
      });
      if (by_path != d.aliases.end()) d.preferred = *by_path;
    }
    if (d.preferred.empty()) d.preferred = d.canonical;
    out.push_back(d);
  }

  std::sort(out.begin(), out.end(), [](const DeviceInfo & a, const DeviceInfo & b) {
    return a.preferred < b.preferred;
  });
  return out;
}

inline bool contains_any_identity(const DeviceInfo & d, const std::vector<std::string> & needles)
{
  const std::string text = identity_text(d);
  for (const auto & needle : needles) {
    if (!needle.empty() && text.find(lower(needle)) != std::string::npos) return true;
  }
  return false;
}

inline bool is_known_imu(const DeviceInfo & d)
{
  if (std::find(d.aliases.begin(), d.aliases.end(), "/dev/imu_yahboom") != d.aliases.end()) {
    return true;
  }
  // Hardware identity already proven on this AGV. Do not depend on ttyUSB index.
  if (alias_contains(d, "Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001")) {
    return true;
  }
  const std::string text = identity_text(d);
  return text.find("witmotion") != std::string::npos ||
         text.find("yahboom") != std::string::npos;
}

inline bool is_probable_imu(const DeviceInfo & d)
{
  if (is_known_imu(d)) return true;
  const std::string text = identity_text(d);
  return text.find("cp210") != std::string::npos ||
         text.find("silicon labs") != std::string::npos ||
         text.find("silicon_labs") != std::string::npos;
}

inline int imu_score(const DeviceInfo & d)
{
  int score = 0;
  if (std::find(d.aliases.begin(), d.aliases.end(), "/dev/imu_yahboom") != d.aliases.end()) score += 2000;
  if (alias_contains(d, "Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001")) score += 1800;
  if (alias_contains(d, "witmotion") || alias_contains(d, "yahboom")) score += 1600;
  if (is_probable_imu(d)) score += 700;
  if (alias_has_prefix(d, "/dev/serial/by-id/")) score += 100;
  if (alias_has_prefix(d, "/dev/serial/by-path/")) score += 50;
  return score;
}

inline int lidar_score(const DeviceInfo & d)
{
  int score = 0;
  if (std::find(d.aliases.begin(), d.aliases.end(), "/dev/lidar") != d.aliases.end()) score += 2000;
  if (std::find(d.aliases.begin(), d.aliases.end(), "/dev/ydlidar") != d.aliases.end()) score += 1900;
  if (alias_contains(d, "ydlidar")) score += 1700;
  if (!is_probable_imu(d)) score += 500;
  if (alias_has_prefix(d, "/dev/serial/by-id/")) score += 100;
  if (alias_has_prefix(d, "/dev/serial/by-path/")) score += 80;
  return score;
}

inline std::string lock_file_for(const std::string & device_path)
{
  const std::string canonical = resolve_path(device_path);
  std::string key = basename_of(canonical);
  if (key.empty()) key = "unknown";
  for (char & c : key) {
    if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_' && c != '-') c = '_';
  }
  return "/tmp/agv_usb_serial_" + key + ".lock";
}

inline int acquire_lock(const std::string & device_path, const std::string & owner)
{
  const std::string lock_path = lock_file_for(device_path);
  const int fd = ::open(lock_path.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0666);
  if (fd < 0) return -1;
  (void)::fchmod(fd, 0666);
  
  // Try to acquire exclusive lock. LOCK_NB makes it non-blocking.
  // If another process holds the lock, flock returns -1 with errno=EWOULDBLOCK.
  if (::flock(fd, LOCK_EX | LOCK_NB) != 0) {
    ::close(fd);
    errno = EBUSY;
    return -1;
  }
  
  // Lock acquired. Write owner info to the file for diagnostics.
  (void)::ftruncate(fd, 0);
  std::ostringstream ss;
  ss << owner << " pid=" << static_cast<long>(::getpid()) << " device="
     << resolve_path(device_path) << "\n";
  const std::string text = ss.str();
  (void)::write(fd, text.data(), text.size());
  
  // DO NOT unlink here. Keep the lock file so other processes can:
  // 1. See that the port is locked (file exists + flock test fails)
  // 2. Read owner info for diagnostics
  // The flock is automatically released when the FD is closed (process exit/crash).
  // We unlink in release_lock() for clean shutdown.

  return fd;
}

inline void release_lock(int & fd, const std::string & device_path)
{
  if (fd >= 0) {
    const std::string lock_path = lock_file_for(device_path);
    (void)::flock(fd, LOCK_UN);
    ::close(fd);
    fd = -1;
    // Unlink after close so the lock is fully released first.
    // On clean shutdown this removes the lock file.
    // Note: if the file was already unlinked by another mechanism this is harmless.
    (void)::unlink(lock_path.c_str());
  }
}

inline std::string describe(const DeviceInfo & d)
{
  std::ostringstream ss;
  ss << d.preferred << " -> " << d.canonical;
  if (!d.driver.empty()) ss << " driver=" << d.driver;
  if (!d.vid.empty() || !d.pid.empty()) ss << " usb=" << d.vid << ':' << d.pid;
  if (!d.manufacturer.empty()) ss << " mfg=\"" << d.manufacturer << '\"';
  if (!d.product.empty()) ss << " product=\"" << d.product << '\"';
  if (!d.serial.empty()) ss << " serial=\"" << d.serial << '\"';
  return ss.str();
}

}  // namespace agv_usb_serial
