# Perception Test GUI — Architecture & Implementation Design

**Tujuan:** Membangun GUI pengujian persepsi yang memenuhi requirement strict:
- Setiap parameter GUI harus berasal dari YAML yang digunakan `autonomous.launch.py`
- Perubahan parameter di GUI harus benar-benar mengubah behavior autonomous runtime
- GUI harus menampilkan **Test Value, YAML Value, Runtime Value** + status MATCH/MISMATCH
- START TEST diblokir sampai `runtime == requested` terbukti via `ros2 param get`
- Cumulative OFAT: parameter best dari leaf sebelumnya dikunci dan dipertahankan

---

## 1. SUMBER PARAMETER (Audit Selesai)

### 1.1 Owner Node

| Parameter | Owner Node | YAML File | YAML Path |
|-----------|-----------|-----------|-----------|
| `v4l2_pixel_format` | `perception` | `astra_yolop_gpu.yaml` | `perception.ros__parameters.v4l2_pixel_format` |
| `use_v4l2_userptr_zero_copy` | `perception` | `astra_yolop_gpu.yaml` | `perception.ros__parameters.use_v4l2_userptr_zero_copy` |
| `rviz_max_publish_rate_hz` | `perception` | `astra_yolop_gpu.yaml` | `perception.ros__parameters.rviz_max_publish_rate_hz` |
| `minimum_obstacle_confidence` | `perception` | `astra_yolop_gpu.yaml` | `perception.ros__parameters.minimum_obstacle_confidence` |
| `drivable_contact_min_fraction` | `perception` | `astra_yolop_gpu.yaml` | `perception.ros__parameters.drivable_contact_min_fraction` |
| `track_confirm_hits` | `perception` | `astra_yolop_gpu.yaml` | `perception.ros__parameters.track_confirm_hits` |
| `track_max_missed_frames` | `perception` | `astra_yolop_gpu.yaml` | `perception.ros__parameters.track_max_missed_frames` |
| `track_ema_alpha` | `perception` | `astra_yolop_gpu.yaml` | `perception.ros__parameters.track_ema_alpha` |
| `lane_threshold` | `perception` | `astra_yolop_gpu.yaml` | `perception.ros__parameters.lane_threshold` |
| `lane_ema_alpha` | `perception` | `astra_yolop_gpu.yaml` | `perception.ros__parameters.lane_ema_alpha` |
| `edge_warning_clearance_m` | `perception` | `astra_yolop_gpu.yaml` | `perception.ros__parameters.edge_warning_clearance_m` |
| `edge_critical_clearance_m` | `perception` | `astra_yolop_gpu.yaml` | `perception.ros__parameters.edge_critical_clearance_m` |
| `edge_release_clearance_m` | `perception` | `astra_yolop_gpu.yaml` | `perception.ros__parameters.edge_release_clearance_m` |
| `trajectory_lateral_margin_m` | `trajectory_safety_supervisor` | `trajectory_safety.yaml` | `trajectory_safety_supervisor.ros__parameters.trajectory_lateral_margin_m` |
| `planning_lateral_margin_m` | `trajectory_safety_supervisor` | `trajectory_safety.yaml` | `trajectory_safety_supervisor.ros__parameters.planning_lateral_margin_m` |
| `slow_path_distance_m` | `trajectory_safety_supervisor` | `trajectory_safety.yaml` | `trajectory_safety_supervisor.ros__parameters.slow_path_distance_m` |
| `immediate_command_hard_stop_m` | `trajectory_safety_supervisor` | `trajectory_safety.yaml` | `trajectory_safety_supervisor.ros__parameters.immediate_command_hard_stop_m` |
| `hard_stop_path_distance_m` | `trajectory_safety_supervisor` | `trajectory_safety.yaml` | `trajectory_safety_supervisor.ros__parameters.hard_stop_path_distance_m` |
| `avoidance_speed_mps` | `trajectory_safety_supervisor` | `trajectory_safety.yaml` | `trajectory_safety_supervisor.ros__parameters.avoidance_speed_mps` |

### 1.2 Klasifikasi Live vs Startup

**Temuan audit source:**
- `astra_yolop_gpu_node.cpp` (line 2235-2540): semua parameter dibaca di constructor `readParameters()`, **tidak ada** `add_on_set_parameters_callback`.
- `trajectory_safety_supervisor.cpp` (line 375-440): semua parameter dibaca di `readParameters()` yang dipanggil constructor, **tidak ada** callback runtime.

**Kesimpulan: SEMUA parameter bersifat STARTUP-ONLY.**

Perubahan parameter memerlukan:
1. Write ke YAML source (via `YamlStore::set`)
2. Restart node owner (SIGTERM + respawn via launch)
3. Verifikasi runtime (`ros2 param get /node_name param_name`)

### 1.3 ⚠️ Launch Override — Harus Dibereskan Dulu

Di `autonomous.launch.py` (block `common_perception_parameters`, ~line 522-550) dan
`gui.launch.py` (declare args, ~line 100-138), nilai berikut di-hardcode sehingga
**menimpa YAML** (ROS rule: dict launch args setelah file YAML menang):

| Parameter | Override saat ini | Dampak |
|---|---|---|
| `v4l2_pixel_format` | `"MJPEG"` (gui.launch) | Edit YAML 4.1.3 tidak berpengaruh |
| `use_v4l2_userptr_zero_copy` | `"false"` (gui.launch) | Edit YAML 4.1.3 tidak berpengaruh |
| `rgb_width` / `rgb_height` / `camera_fps` | `1280` / `720` / `30` | Edit YAML resolusi tidak berpengaruh |
| `strict_camera_mode` | `"false"` | — |
| `allow_mjpeg_cpu_fallback` | `"true"` | — |
| `rviz_max_publish_rate_hz` | `8.0` (hardcoded di common_perception_parameters) | Edit YAML 4.1.4 tidak berpengaruh |
| `camera_retry_interval_sec` / `camera_retry_log_interval_sec` | `2.0` / `10.0` | — |
| `allow_resolution_fallback_for_fps` | `false` | — |
| flag `publish_*` (annotated/raw/detections/masks/...) | hardcoded | — |
| `annotated_topic`, `metric_frame_id`, `nav_cmd_topic`, `safe_cmd_topic` | hardcoded string | — |

**Perbaikan wajib (bagian dari Session 1):** ubah semua default di atas menjadi
`_yaml_ros_param(camera_params, "perception", "<key>", <default saat ini>)` sehingga
YAML menjadi single source of truth. Node tetap menerima nilai dari launch args,
tapi default launch args kini dibaca dari YAML yang sama — GUI edit → launch → node
menjadi satu rantai nilai.

---

## 2. DATA FLOW

```
User edits field
       ↓
ExperimentParameterPanel::setText
       ↓
YamlStore::set (atomic write YAML source)
       ↓
MainWindow::stageChange + autosave
       ↓
User clicks "Apply + Restart Node"
       ↓
RestartNodeFlow:
  1. Read requested value from GUI field
  2. Write to YAML (YamlStore)
  3. Send SIGTERM to owner node via ROS service or process kill
  4. Wait for launch respawn (perception_respawn:=true di launch)
  5. Poll /node_name/get_parameters sampai value == requested
  6. Timeout 15s → FAIL: "Node tidak apply parameter"
       ↓
Triple-status panel update:
  Test Value    : 0.30 (dari QLineEdit)
  YAML Value    : 0.30 (dari YamlStore::get)
  Runtime Value : 0.30 (dari ros2 param get)
  Status        : MATCH ✓
       ↓
User clicks "● Mulai Rekam Run"
       ↓
START INTERLOCK:
  - cek semua parameter teruji: runtime == test value
  - jika MISMATCH → BLOCK + pesan "Parameter belum aktif. Apply + Restart dulu."
  - jika MATCH → recording dimulai
       ↓
captureSample() mencatat metadata:
  - YAML snapshot (semua param aktif leaf)
  - Runtime verification timestamp
  - Parameter owner node + version
       ↓
Stop + Save Evidence:
  - CSV tabel + raw
  - PNG grafik (semua)
  - Manifest JSON (termasuk applied_parameters map)
```

---

## 3. GUI COMPONENT EXTENSION

### 3.1 ExperimentParameterPanel — Status Display

**Tambah widget baru per field editable:**

```cpp
struct ParamWidget {
  QString key;
  QString kind;           // "float", "int", "string"
  QString yamlFileKey;    // "perception", "trajectory_safety"
  QString yamlPath;       // "perception.ros__parameters.minimum_obstacle_confidence"
  QString ownerNode;      // "/perception", "/trajectory_safety_supervisor"
  QLineEdit *testValue;   // user input
  QLabel *yamlValue;      // dari YamlStore::get (read-only)
  QLabel *runtimeValue;   // dari ros2 param get (read-only)
  QLabel *status;         // "MATCH ✓" | "NOT APPLIED" | "RESTART REQUIRED"
  QPushButton *applyBtn;  // "Apply + Restart Node"
};
```

**Layout per field (4 baris):**

```
┌─────────────────────────────────────────────────┐
│ Label: Minimum Obstacle Confidence              │
├─────────────────────────────────────────────────┤
│ Test Value:    [0.30____________] [Apply+Restart│
│ YAML Value:    0.30 (dari file aktif)           │
│ Runtime Value: 0.25 (node sedang pakai ini)     │
│ Status:        ⚠ NOT APPLIED — restart required │
└─────────────────────────────────────────────────┘
```

### 3.2 Restart Node Flow (Qt Slot)

```cpp
void ExperimentParameterPanel::applyAndRestartNode(const QString &key) {
  auto w = widgets_.value(key);
  const QString requestedText = w.testValue->text().trimmed();
  QVariant requested = parseValue(requestedText, w.kind);
  
  // 1. Write YAML
  auto store = stores_.value(w.yamlFileKey);
  if (!store || !store->set(w.yamlPath, requested)) {
    showError("Gagal menulis YAML");
    return;
  }
  
  // 2. Request node restart via RosBridge signal
  emit restartNodeRequested(w.ownerNode, w.key, requested);
  
  // 3. UI feedback: status = "Restarting..."
  w.status->setText("Restarting node...");
  w.applyBtn->setEnabled(false);
}

// MainWindow slot (connected to restartNodeRequested):
void MainWindow::handleNodeRestart(const QString &nodeName, 
                                   const QString &paramKey,
                                   const QVariant &expectedValue) {
  // Implementation pilihan:
  // A. Via process kill + wait respawn (butuh process PID map)
  // B. Via ROS service /node_name/shutdown (kalau node punya)
  // C. SIGTERM ke child process launch tree (paling aman)
  
  // Implementasi awal: kirim SIGTERM ke node, tunggu respawn
  ros_->terminateNode(nodeName);
  
  // Poll sampai node hidup lagi + parameter applied
  QTimer::singleShot(2000, [this, nodeName, paramKey, expectedValue]() {
    verifyRuntimeParameter(nodeName, paramKey, expectedValue);
  });
}

void MainWindow::verifyRuntimeParameter(const QString &nodeName,
                                        const QString &paramKey,
                                        const QVariant &expected) {
  // Call RosBridge::getParameters
  ros_->getParameters(nodeName, QStringList{paramKey}, 
                      QString("verify_%1_%2").arg(nodeName, paramKey));
}

// RosBridge result handler:
void MainWindow::handleParametersResult(QString tag, bool ok, QVariantMap values) {
  if (!tag.startsWith("verify_")) return;
  
  // Extract node + param from tag
  // Compare values[paramKey] vs expected
  // Update ExperimentParameterPanel status widget
  
  if (ok && same(values[paramKey], expected)) {
    // Status → "MATCH ✓"
    experimentParamPanel_->setRuntimeStatus(paramKey, "MATCH", values[paramKey]);
  } else {
    // Status → "MISMATCH — retry atau manual check"
    experimentParamPanel_->setRuntimeStatus(paramKey, "MISMATCH", values[paramKey]);
  }
}
```

### 3.3 START TEST Interlock (ExperimentWorkspacePage)

```cpp
void ExperimentWorkspacePage::startRecording() {
  // INTERLOCK: cek semua parameter teruji sudah MATCH
  QStringList mismatch;
  const auto &fields = spec().parameterFields;
  for (const auto &f : fields) {
    if (f.kind == "yaml_readonly" || f.locked) continue; // skip read-only
    if (f.yamlFileKey.isEmpty()) continue; // GUI-only field (sample_rate, etc)
    
    QString status = paramPanel_->runtimeStatus(f.key);
    if (status != "MATCH") {
      mismatch << f.label;
    }
  }
  
  if (!mismatch.isEmpty()) {
    QMessageBox::warning(this, "Parameter Belum Aktif",
      QString("Parameter berikut belum diterapkan ke runtime:\n\n%1\n\n"
              "Klik 'Apply + Restart Node' untuk setiap parameter, "
              "tunggu status MATCH ✓, baru mulai rekam run.")
      .arg(mismatch.join("\n")));
    return;
  }
  
  // Status OK → lanjut recording (existing flow)
  saveTableState();
  session().rawRows.clear();
  // ... (existing code)
}
```

---

## 4. CUMULATIVE OFAT LOCK

### 4.1 Mekanisme Lock

**User workflow:**
1. Leaf 4.2.1 (minimum_obstacle_confidence): uji 0.20, 0.30, 0.40, 0.50
2. User pilih 0.30 sebagai BEST → klik "Lock as Baseline"
3. GUI:
   - `YamlStore::set("perception", "...minimum_obstacle_confidence", 0.30)`
   - Tambah field ke `lockedParameters_` map
   - Field jadi `QLineEdit::setReadOnly(true)` + background kuning + label "LOCKED"
4. Leaf 4.2.2 (drivable_contact): panel menampilkan `minimum_obstacle_confidence = 0.30 (LOCKED)`
5. Setiap Apply + Restart di leaf 4.2.2: nilai locked 0.30 tetap dipertahankan di YAML

**Struktur data:**

```cpp
// MainWindow or ExperimentWorkspacePage
QMap<QString, QVariant> lockedParameters_; 
// key = "perception|minimum_obstacle_confidence"
// value = 0.30

struct LockedParam {
  QString yamlFileKey;
  QString yamlPath;
  QVariant value;
  QString lockedAtLeaf; // "4.2.1"
};
QMap<QString, LockedParam> cumulativeBaseline_;

// Saat user klik "Lock as Baseline":
void ExperimentWorkspacePage::lockParameterAsBaseline(const QString &key) {
  auto w = paramPanel_->widget(key);
  LockedParam lp;
  lp.yamlFileKey = w.yamlFileKey;
  lp.yamlPath = w.yamlPath;
  lp.value = w.testValue->text(); // atau parseValue
  lp.lockedAtLeaf = currentLeafId_;
  cumulativeBaseline_[key] = lp;
  
  // Write to YAML immediately
  stores_[lp.yamlFileKey]->set(lp.yamlPath, lp.value);
  
  // Mark field locked
  paramPanel_->lockField(key);
}

// Saat pindah ke leaf baru:
void ExperimentWorkspacePage::selectLeaf(const QString &id) {
  // ... existing code
  
  // Inject locked parameters as read-only fields
  paramPanel_->pushLockedParameters(cumulativeBaseline_);
}
```

### 4.2 Persistence

**Simpan baseline ke file:**

```yaml
# ~/.hermes/profiles/default/perception_ofat_baseline.yaml
subsystem: perception
locked_parameters:
  - key: minimum_obstacle_confidence
    yaml_file: perception
    yaml_path: perception.ros__parameters.minimum_obstacle_confidence
    value: 0.30
    locked_at_leaf: "4.2.1"
    locked_timestamp: "2026-08-31T15:30:00Z"
```

Load saat GUI startup, restore saat switch ke perception tab.

---

## 5. VERIFICATION CHECKLIST

### 5.1 Pre-Implementation

- [x] Audit parameter source: YAML path valid
- [x] Audit node source: startup-only (no runtime callback)
- [x] Konfirmasi `autonomous.launch.py` load YAML yang sama dengan GUI
- [x] `YamlStore` atomic write sudah aman
- [x] `RosBridge::getParameters` sudah tersedia

### 5.2 Implementation

- [ ] Extend `ExperimentParameterField`: tambah `ownerNode`, `isLive` flag
- [ ] Extend `agv_experiment_catalog.hpp`: 15+ leaf × parameterFields
- [ ] Extend `ExperimentParameterPanel`: triple-status widget + Apply button
- [ ] Implement `MainWindow::handleNodeRestart` + verification polling
- [ ] Implement `ExperimentWorkspacePage::startRecording` interlock
- [ ] Implement cumulative OFAT lock mechanism
- [ ] Build + colcon test

### 5.3 Runtime Verification

- [ ] Launch `gui.launch.py` (sensors-only)
- [ ] Open Persepsi → 4.2.1
- [ ] Edit `minimum_obstacle_confidence` → 0.25
- [ ] Klik "Apply + Restart Node"
- [ ] Tunggu status MATCH ✓
- [ ] `ros2 param get /perception minimum_obstacle_confidence` → harus 0.25
- [ ] Klik "● Mulai Rekam Run" → harus lolos interlock
- [ ] Stop + Save → manifest JSON harus contain applied params

---

## 6. FILE CHANGES SUMMARY

| File | Change Type | LoC Est. |
|------|-------------|----------|
| `agv_experiment_catalog.hpp` | Extend parameterFields for 15+ leaves | +300 |
| `experiment_components.hpp` | Add triple-status widget struct | +20 |
| `experiment_components.cpp` | Implement new widgets + Apply flow | +150 |
| `main_window.cpp` | Node restart orchestration + verification | +120 |
| `system_and_gnss_pages.cpp` | START interlock + OFAT lock | +100 |
| `gui_core.cpp` | (minor) YamlStore helper for batch write | +30 |
| `agv_gui.cpp` | (minor) RosBridge signal wiring | +20 |
| **Total** | | **~740 LoC** |

Build time estimate: 1-2 menit (hanya GUI, tidak ada ROS msg/srv baru).

---

## 7. RISK & MITIGATION

| Risk | Impact | Mitigation |
|------|--------|------------|
| Node restart gagal (stuck) | User tidak bisa lanjut test | Timeout 15s + fallback: manual restart via terminal |
| Parameter YAML corrupt | Launch fail | `YamlStore` atomic write + backup before edit |
| Runtime verification timeout | False MISMATCH | Retry 3× dengan backoff; log ke file untuk debug |
| Launch override masih hardcode | YAML tidak jadi source of truth | Audit `autonomous.launch.py` line 522-550, pastikan semua dari YAML |
| User lupa lock baseline | OFAT tidak cumulative | UI reminder: "Parameter sebelumnya belum dikunci" |

---

## 8. ACCEPTANCE CRITERIA

✅ **PASS** jika:
1. User edit parameter → YAML file berubah (verified via `cat`)
2. Klik Apply + Restart → node benar-benar restart (log ROS menunjukkan shutdown + re-init)
3. Status panel: Runtime Value == Test Value setelah Apply
4. START TEST diblokir jika ada MISMATCH
5. Metadata experiment CSV/JSON mencatat parameter runtime yang digunakan
6. Cumulative lock: nilai best dari leaf sebelumnya tetap di YAML saat switch leaf

❌ **FAIL** jika:
1. Parameter berubah di GUI tapi runtime tetap pakai nilai lama
2. START TEST lolos meski runtime != requested
3. Metadata tidak mencatat parameter aktual
4. Lock tidak dipertahankan saat switch leaf

---

## 9. NEXT SESSION PLAN

**Session 1 (2-3 jam):**
- Extend catalog (15 leaves)
- Build triple-status widget
- Implement Apply + YAML write

**Session 2 (2-3 jam):**
- Implement node restart flow
- Implement runtime verification polling
- Implement START interlock

**Session 3 (1-2 jam):**
- Implement OFAT lock mechanism
- Build + integration test
- Runtime verification checklist

**Total: 5-8 jam kerja.**

---

**APPROVAL NEEDED:**

Apakah design ini sesuai requirement kamu? Kalau ada yang perlu diubah (misalnya UI layout, restart mechanism, verification timeout), kasih tau sekarang sebelum aku mulai coding.
