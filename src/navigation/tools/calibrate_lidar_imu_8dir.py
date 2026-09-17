#!/usr/bin/python3
import argparse
import csv
import json
import math
import os
import statistics
import threading
import time
from datetime import datetime

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu, LaserScan

G = 9.80665
DIRECTIONS = [
    (0, "N"), (45, "NE"), (90, "E"), (135, "SE"),
    (180, "S"), (225, "SW"), (270, "W"), (315, "NW"),
]

QOS = QoSProfile(
    depth=100,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


def ts_msg(msg):
    s = msg.header.stamp
    return float(s.sec) + float(s.nanosec) * 1e-9


def basic_stats(values):
    vals = [float(v) for v in values if math.isfinite(float(v))]
    if not vals:
        return {"n": 0, "mean": None, "std": None, "min": None, "max": None}
    return {
        "n": len(vals),
        "mean": statistics.fmean(vals),
        "std": statistics.stdev(vals) if len(vals) > 1 else 0.0,
        "min": min(vals),
        "max": max(vals),
    }


def wrap180(deg):
    return (float(deg) + 180.0) % 360.0 - 180.0


def circular_mean_deg(rad_values):
    vals = [float(v) for v in rad_values if math.isfinite(float(v))]
    if not vals:
        return None
    s = statistics.fmean(math.sin(v) for v in vals)
    c = statistics.fmean(math.cos(v) for v in vals)
    return math.degrees(math.atan2(s, c))


class SensorRecorder(Node):
    def __init__(self):
        super().__init__("lidar_imu_8dir_calibrator")
        self.lock = threading.Lock()
        self.collecting = False
        self.current_deg = None
        self.current_label = None
        self.data = {"accel": [], "gyro": [], "mag": [], "euler": [], "lidar": []}
        self.seen = {k: 0 for k in self.data}

        self.create_subscription(Imu, "/imu/raw/accel", self.cb_accel, QOS)
        self.create_subscription(Imu, "/imu/raw/gyro", self.cb_gyro, QOS)
        self.create_subscription(Vector3Stamped, "/imu/raw/mag", self.cb_mag, QOS)
        self.create_subscription(Vector3Stamped, "/imu/raw/euler", self.cb_euler, QOS)
        self.create_subscription(LaserScan, "/scan", self.cb_scan, QOS)

    def _append(self, key, row):
        with self.lock:
            self.seen[key] += 1
            if self.collecting:
                self.data[key].append(row)

    def cb_accel(self, msg):
        a = msg.linear_acceleration
        self._append("accel", (ts_msg(msg), a.x, a.y, a.z))

    def cb_gyro(self, msg):
        g = msg.angular_velocity
        self._append("gyro", (ts_msg(msg), g.x, g.y, g.z))

    def cb_mag(self, msg):
        v = msg.vector
        self._append("mag", (ts_msg(msg), v.x, v.y, v.z))

    def cb_euler(self, msg):
        v = msg.vector
        self._append("euler", (ts_msg(msg), v.x, v.y, v.z))

    def cb_scan(self, msg):
        row = {
            "t": ts_msg(msg),
            "angle_min": float(msg.angle_min),
            "angle_increment": float(msg.angle_increment),
            "range_min": float(msg.range_min),
            "range_max": float(msg.range_max),
            "ranges": [float(x) for x in msg.ranges],
            "intensities": [float(x) for x in msg.intensities],
        }
        self._append("lidar", row)

    def begin(self, deg, label):
        with self.lock:
            self.data = {"accel": [], "gyro": [], "mag": [], "euler": [], "lidar": []}
            self.current_deg = deg
            self.current_label = label
            self.collecting = True

    def end(self):
        with self.lock:
            self.collecting = False
            return {k: list(v) for k, v in self.data.items()}

    def readiness(self):
        with self.lock:
            return dict(self.seen)


def vector_summary(rows, names=("x", "y", "z")):
    out = {}
    for i, name in enumerate(names, start=1):
        out[name] = basic_stats(r[i] for r in rows)
    return out


def lidar_summary(scans):
    valid_all = []
    per_scan_min = []
    per_scan_mean = []
    valid_counts = []
    for scan in scans:
        vals = [r for r in scan["ranges"] if math.isfinite(r)
                and scan["range_min"] <= r <= scan["range_max"]]
        valid_counts.append(len(vals))
        if vals:
            valid_all.extend(vals)
            per_scan_min.append(min(vals))
            per_scan_mean.append(statistics.fmean(vals))
    return {
        "scan_count": len(scans),
        "valid_beams_per_scan": basic_stats(valid_counts),
        "range_m_all_valid": basic_stats(valid_all),
        "nearest_m_per_scan": basic_stats(per_scan_min),
        "mean_range_m_per_scan": basic_stats(per_scan_mean),
    }


def direction_summary(deg, label, data):
    yaw_deg = circular_mean_deg(r[3] for r in data["euler"])
    return {
        "target_deg": deg,
        "label": label,
        "accel_mps2": vector_summary(data["accel"]),
        "gyro_rps": vector_summary(data["gyro"]),
        "mag_T": vector_summary(data["mag"]),
        "euler_rad": vector_summary(data["euler"], ("roll", "pitch", "yaw")),
        "yaw_circular_mean_deg": yaw_deg,
        "lidar": lidar_summary(data["lidar"]),
    }


def collect_axis(captures, sensor, axis_index):
    out = []
    for cap in captures:
        out.extend(r[axis_index] for r in cap["data"][sensor])
    return out


def mean_axis(cap, sensor, axis_index):
    vals = [r[axis_index] for r in cap["data"][sensor]]
    return statistics.fmean(vals) if vals else None


def safe_scale(radius, avg_radius):
    if radius is None or radius <= 1e-12:
        return 1.0
    return avg_radius / radius


def compute_calibration(captures, summaries):
    ax = basic_stats(collect_axis(captures, "accel", 1))["mean"]
    ay = basic_stats(collect_axis(captures, "accel", 2))["mean"]
    az = basic_stats(collect_axis(captures, "accel", 3))["mean"]
    gx = basic_stats(collect_axis(captures, "gyro", 1))["mean"]
    gy = basic_stats(collect_axis(captures, "gyro", 2))["mean"]
    gz = basic_stats(collect_axis(captures, "gyro", 3))["mean"]

    mag_x = [mean_axis(c, "mag", 1) for c in captures]
    mag_y = [mean_axis(c, "mag", 2) for c in captures]
    mag_x = [v for v in mag_x if v is not None]
    mag_y = [v for v in mag_y if v is not None]

    mx_off = (max(mag_x) + min(mag_x)) / 2.0 if mag_x else 0.0
    my_off = (max(mag_y) + min(mag_y)) / 2.0 if mag_y else 0.0
    rx = (max(mag_x) - min(mag_x)) / 2.0 if mag_x else None
    ry = (max(mag_y) - min(mag_y)) / 2.0 if mag_y else None
    radii = [r for r in (rx, ry) if r is not None and r > 1e-12]
    ravg = statistics.fmean(radii) if radii else 1.0

    yaw_means = [s["yaw_circular_mean_deg"] for s in summaries]
    yaw_zero = yaw_means[0] if yaw_means and yaw_means[0] is not None else 0.0

    sign_scores = {}
    for sign in (1.0, -1.0):
        errs = []
        for s, y in zip(summaries, yaw_means):
            if y is None:
                continue
            est = sign * wrap180(y - yaw_zero)
            errs.append(wrap180(est - s["target_deg"]))
        rmse = math.sqrt(statistics.fmean(e * e for e in errs)) if errs else None
        sign_scores[str(int(sign))] = rmse
    valid_scores = [(v, float(k)) for k, v in sign_scores.items() if v is not None]
    yaw_sign = min(valid_scores)[1] if valid_scores else 1.0

    return {
        "assumptions": [
            "Sensor dijaga level; rotasi hanya yaw pada 8 arah.",
            "Accel Z mengarah ke atas sehingga kondisi diam ~ +g.",
            "Kalibrasi magnetometer Z tidak dapat diidentifikasi dari rotasi yaw saja.",
        ],
        "accel_bias_mps2": {"x": ax or 0.0, "y": ay or 0.0, "z": (az - G) if az is not None else 0.0},
        "gyro_bias_rps": {"x": gx or 0.0, "y": gy or 0.0, "z": gz or 0.0},
        "mag_hard_iron_T": {"x": mx_off, "y": my_off, "z": None},
        "mag_soft_iron_scale_2d": {"x": safe_scale(rx, ravg), "y": safe_scale(ry, ravg), "z": 1.0},
        "yaw_relative": {"zero_raw_deg": yaw_zero, "direction_sign": yaw_sign,
                         "rmse_deg": sign_scores[str(int(yaw_sign))]},
    }


def write_imu_csv(path, captures):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["timestamp", "target_deg", "label", "sensor", "x", "y", "z", "unit"])
        units = {"accel": "m/s^2", "gyro": "rad/s", "mag": "T", "euler": "rad"}
        for cap in captures:
            for sensor in ("accel", "gyro", "mag", "euler"):
                for row in cap["data"][sensor]:
                    w.writerow([row[0], cap["deg"], cap["label"], sensor,
                                row[1], row[2], row[3], units[sensor]])


def write_lidar_csv(path, captures):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["timestamp", "target_deg", "label", "scan_index", "beam_index",
                    "angle_deg", "range_m", "intensity", "valid"])
        for cap in captures:
            for si, scan in enumerate(cap["data"]["lidar"]):
                for bi, r in enumerate(scan["ranges"]):
                    angle = scan["angle_min"] + bi * scan["angle_increment"]
                    valid = math.isfinite(r) and scan["range_min"] <= r <= scan["range_max"]
                    inten = scan["intensities"][bi] if bi < len(scan["intensities"]) else None
                    w.writerow([scan["t"], cap["deg"], cap["label"], si, bi,
                                math.degrees(angle), r, inten, int(valid)])


def write_summary_csv(path, summaries):
    headers = [
        "target_deg", "label", "yaw_mean_deg",
        "ax_mean", "ax_std", "ay_mean", "ay_std", "az_mean", "az_std",
        "gx_mean", "gx_std", "gy_mean", "gy_std", "gz_mean", "gz_std",
        "mx_mean_T", "mx_std_T", "my_mean_T", "my_std_T", "mz_mean_T", "mz_std_T",
        "lidar_scans", "lidar_valid_mean", "lidar_nearest_mean_m", "lidar_range_mean_m",
    ]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=headers)
        w.writeheader()
        for s in summaries:
            row = {"target_deg": s["target_deg"], "label": s["label"],
                   "yaw_mean_deg": s["yaw_circular_mean_deg"]}
            for pfx, group in (("a", s["accel_mps2"]), ("g", s["gyro_rps"]), ("m", s["mag_T"])):
                for axis in ("x", "y", "z"):
                    suffix = "_T" if pfx == "m" else ""
                    row[f"{pfx}{axis}_mean{suffix}"] = group[axis]["mean"]
                    row[f"{pfx}{axis}_std{suffix}"] = group[axis]["std"]
            ls = s["lidar"]
            row["lidar_scans"] = ls["scan_count"]
            row["lidar_valid_mean"] = ls["valid_beams_per_scan"]["mean"]
            row["lidar_nearest_mean_m"] = ls["nearest_m_per_scan"]["mean"]
            row["lidar_range_mean_m"] = ls["mean_range_m_per_scan"]["mean"]
            w.writerow(row)


def write_calibration_yaml(path, cal):
    a, g, m, s, y = (cal["accel_bias_mps2"], cal["gyro_bias_rps"],
                     cal["mag_hard_iron_T"], cal["mag_soft_iron_scale_2d"], cal["yaw_relative"])
    text = f"""# Hasil kalibrasi statis 8 arah (yaw-only)\naccel_bias_mps2:\n  x: {a['x']:.10g}\n  y: {a['y']:.10g}\n  z: {a['z']:.10g}\ngyro_bias_rps:\n  x: {g['x']:.10g}\n  y: {g['y']:.10g}\n  z: {g['z']:.10g}\nmag_hard_iron_T:\n  x: {m['x']:.10g}\n  y: {m['y']:.10g}\n  z: null  # tidak teridentifikasi dengan rotasi yaw saja\nmag_soft_iron_scale_2d:\n  x: {s['x']:.10g}\n  y: {s['y']:.10g}\n  z: 1.0\nyaw_relative:\n  zero_raw_deg: {y['zero_raw_deg']:.6f}\n  direction_sign: {y['direction_sign']:.0f}\n  rmse_deg: {y['rmse_deg'] if y['rmse_deg'] is not None else 'null'}\n"""
    with open(path, "w") as f:
        f.write(text)


def wait_for_topics(node, timeout=5.0):
    start = time.time()
    while time.time() - start < timeout:
        if all(v > 0 for v in node.readiness().values()):
            return True
        time.sleep(0.1)
    return False


def main():
    ap = argparse.ArgumentParser(description="Kalibrasi statis LiDAR + IMU Yahboom 8 arah")
    ap.add_argument("--duration", type=float, default=5.0, help="durasi sampling tiap arah (s)")
    ap.add_argument("--settle", type=float, default=2.0, help="waktu diam sebelum sampling (s)")
    ap.add_argument("--out", default="", help="folder output")
    args = ap.parse_args()

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    outdir = args.out or f"/home/otomasi2/forclift/log/calibration_imu_lidar_8dir_{stamp}"
    os.makedirs(outdir, exist_ok=True)

    rclpy.init()
    node = SensorRecorder()
    th = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    th.start()

    print("\n=== KALIBRASI STATIS LIDAR + IMU YAHBOOM: 8 ARAH ===", flush=True)
    print(f"Output: {outdir}", flush=True)
    print("Pastikan sensor/AGV level dan DIAM saat pengambilan data.", flush=True)
    print("Urutan rotasi: 0 -> 45 -> 90 -> ... -> 315 derajat.", flush=True)
    ok = wait_for_topics(node, 5.0)
    print(f"Status topic: {node.readiness()}", flush=True)
    if not ok:
        print("PERINGATAN: ada topic belum menerima data; tetap lanjut agar bisa didiagnosis.", flush=True)

    captures = []
    summaries = []
    try:
        for i, (deg, label) in enumerate(DIRECTIONS, start=1):
            input(f"\n[{i}/8] Hadapkan ke {deg} deg ({label}), lalu tekan ENTER saat sudah DIAM...")
            print(f"Stabilisasi {args.settle:.1f} s...", flush=True)
            time.sleep(args.settle)
            node.begin(deg, label)
            print(f"SAMPLING {args.duration:.1f} s - jangan gerakkan sensor...", flush=True)
            time.sleep(args.duration)
            data = node.end()
            cap = {"deg": deg, "label": label, "data": data}
            captures.append(cap)
            summ = direction_summary(deg, label, data)
            summaries.append(summ)
            print("Selesai arah %d deg: accel=%d gyro=%d mag=%d euler=%d lidar=%d scan" % (
                deg, len(data["accel"]), len(data["gyro"]), len(data["mag"]),
                len(data["euler"]), len(data["lidar"])), flush=True)
            print("  yaw mean = %s deg" % (
                "NA" if summ["yaw_circular_mean_deg"] is None else f"{summ['yaw_circular_mean_deg']:.3f}"), flush=True)

        cal = compute_calibration(captures, summaries)
        write_imu_csv(os.path.join(outdir, "imu_raw.csv"), captures)
        write_lidar_csv(os.path.join(outdir, "lidar_raw.csv"), captures)
        write_summary_csv(os.path.join(outdir, "static_summary_8dir.csv"), summaries)

        with open(os.path.join(outdir, "statistics_full.json"), "w") as f:
            json.dump({"directions": summaries, "calibration": cal}, f, indent=2)
        write_calibration_yaml(os.path.join(outdir, "imu_calibration_8dir.yaml"), cal)

        print("\n=== SELESAI ===", flush=True)
        print("Hasil:", flush=True)
        print(" - imu_raw.csv", flush=True)
        print(" - lidar_raw.csv", flush=True)
        print(" - static_summary_8dir.csv", flush=True)
        print(" - statistics_full.json", flush=True)
        print(" - imu_calibration_8dir.yaml", flush=True)
        print(f"Folder: {outdir}", flush=True)
        print(json.dumps(cal, indent=2), flush=True)

    except KeyboardInterrupt:
        print("\nDihentikan pengguna. Data sesi belum lengkap tidak diterapkan ke driver.", flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
