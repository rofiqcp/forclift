/*
 * converter_pt_to_onnx_to_engine.cpp
 *
 * Tujuan:
 *   Menyediakan utilitas yang dapat dipanggil melalui:
 *
 *     ros2 run perception converter_pt_to_onnx_to_engine
 *
 *   Program otomatis mencari file *.pt di folder models paket, mengekspornya
 *   menjadi ONNX, lalu membangun TensorRT engine dengan trtexec.
 *
 * Kenapa executable-nya C++ tetapi ekspor PT -> ONNX memakai Python?
 *   File .pt YOLOPv2 resmi adalah TorchScript/PyTorch. Jalur ekspor ONNX yang
 *   paling kompatibel disediakan oleh torch.onnx.export di Python. Program C++
 *   ini bertindak sebagai orkestrator ROS 2 yang:
 *     1. menemukan folder models secara otomatis,
 *     2. memilih semua model .pt,
 *     3. menulis exporter Python sementara yang tertanam di source ini,
 *     4. memvalidasi keluaran YOLOPv2,
 *     5. menjalankan trtexec dengan opsi optimal yang didukung versinya,
 *     6. membandingkan numerik 8 output TensorRT terhadap referensi .pt,
 *     7. mengganti engine secara atomik hanya bila parity lulus.
 *
 * Karakteristik engine yang dihasilkan secara default:
 *   input          : images [1, 3, 384, 640]
 *   detection head : 3 tensor dengan channel 255 (stride 8/16/32)
 *   anchor-grid    : 3 tensor [1,3,1,1,2] langsung dari model resmi
 *   drivable area  : 1 tensor [1,2,384,640]
 *   lane           : 1 tensor [1,1,384,640]
 *   precision      : FP16 bila trtexec mendukung --fp16
 *   workspace      : 4096 MiB
 *
 * Catatan penting:
 *   TensorRT engine bergantung pada GPU, versi CUDA, dan versi TensorRT tempat
 *   engine dibangun. Karena itu engine sebaiknya dibuat langsung di MiniPC/GPU
 *   yang akan menjalankan perception_node.
 */

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#if defined(__linux__)
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace fs = std::filesystem;

namespace {

constexpr const char *kPackageName = "perception";

struct Options {
  std::optional<fs::path> models_dir;
  std::optional<fs::path> pt_file;
  std::string python = "python3";
  std::optional<fs::path> trtexec;

  int batch = 1;
  int channels = 3;
  int height = 384;
  int width = 640;
  int opset = 17;
  int workspace_mib = 4096;
  int builder_optimization_level = 5;

  std::string precision = "fp16";
  bool force = false;
  bool simplify = false;
  bool validate = true;
  bool verbose = false;
  bool keep_temporary_exporter = false;
  bool onnx_only = false;
};

struct CommandResult {
  int exit_code = -1;
  std::string output;
};

// Mengubah teks path agar aman ketika dimasukkan ke shell POSIX.
std::string shellQuote(const std::string &value) {
  std::string quoted = "'";
  for (const char character : value) {
    if (character == '\'') {
      quoted += "'\\''";
    } else {
      quoted += character;
    }
  }
  quoted += "'";
  return quoted;
}

std::string shellQuote(const fs::path &value) {
  return shellQuote(value.string());
}

// Menjalankan perintah dan menangkap stdout + stderr. Output tetap ditampilkan
// setelah proses selesai supaya pesan dari PyTorch/ONNX/TensorRT tidak hilang.
CommandResult runCommandCapture(const std::string &command) {
  const std::string wrapped = command + " 2>&1";
  FILE *pipe = ::popen(wrapped.c_str(), "r");
  if (!pipe) {
    throw std::runtime_error("Gagal menjalankan perintah: " + command);
  }

  std::array<char, 4096> buffer{};
  std::string output;
  while (std::fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
    output += buffer.data();
  }

  const int status = ::pclose(pipe);
  int exit_code = status;
#if defined(__linux__)
  if (WIFEXITED(status)) {
    exit_code = WEXITSTATUS(status);
  } else if (WIFSIGNALED(status)) {
    exit_code = 128 + WTERMSIG(status);
  }
#endif
  return {exit_code, output};
}

bool commandExists(const std::string &command) {
  const auto result = runCommandCapture("command -v " + shellQuote(command));
  return result.exit_code == 0 && !result.output.empty();
}

std::string lowercase(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

bool hasExtension(const fs::path &path, const std::string &extension) {
  return lowercase(path.extension().string()) == lowercase(extension);
}

bool isRegularDirectory(const fs::path &path) {
  std::error_code error;
  return fs::is_directory(path, error) && !error;
}

bool isRegularFile(const fs::path &path) {
  std::error_code error;
  return fs::is_regular_file(path, error) && !error;
}

fs::path canonicalOrAbsolute(const fs::path &path) {
  std::error_code error;
  const fs::path canonical = fs::weakly_canonical(path, error);
  if (!error) {
    return canonical;
  }
  return fs::absolute(path);
}

std::optional<fs::path> executablePath() {
#if defined(__linux__)
  std::array<char, 4096> buffer{};
  const ssize_t length = ::readlink("/proc/self/exe", buffer.data(), buffer.size() - 1U);
  if (length > 0) {
    buffer[static_cast<size_t>(length)] = '\0';
    return fs::path(buffer.data());
  }
#endif
  return std::nullopt;
}

std::vector<std::string> splitColonList(const char *value) {
  std::vector<std::string> result;
  if (!value) {
    return result;
  }
  std::stringstream stream(value);
  std::string item;
  while (std::getline(stream, item, ':')) {
    if (!item.empty()) {
      result.push_back(item);
    }
  }
  return result;
}

void addCandidate(
  std::vector<fs::path> &candidates,
  std::set<std::string> &seen,
  const fs::path &candidate)
{
  if (candidate.empty()) {
    return;
  }
  const fs::path normalized = canonicalOrAbsolute(candidate);
  const std::string key = normalized.string();
  if (seen.insert(key).second) {
    candidates.push_back(normalized);
  }
}

// Mencari folder models dengan urutan prioritas:
//   1. argumen --models-dir,
//   2. environment ASTRA_YOLOP_MODELS_DIR,
//   3. external workspace models: <ros>/models (recommended),
//   4. legacy source/install locations only for explicit conversion compatibility.
fs::path resolveModelsDirectory(const Options &options) {
  if (options.models_dir) {
    const fs::path selected = canonicalOrAbsolute(*options.models_dir);
    if (!isRegularDirectory(selected)) {
      throw std::runtime_error("Folder --models-dir tidak ditemukan: " + selected.string());
    }
    return selected;
  }

  std::vector<fs::path> candidates;
  std::set<std::string> seen;

  // Keep generated model artifacts outside src/install. This is also the
  // only runtime engine directory used by perception_node.
  addCandidate(candidates, seen,
    "/home/sirobo/ros/models");

  if (const char *environment = std::getenv("ASTRA_YOLOP_MODELS_DIR")) {
    addCandidate(candidates, seen, environment);
  }

  const fs::path current = fs::current_path();
  addCandidate(candidates, seen, current / "models");
  addCandidate(candidates, seen, current / "src" / kPackageName / "models");
  addCandidate(candidates, seen, current / kPackageName / "models");

  fs::path cursor = current;
  for (int level = 0; level < 10; ++level) {
    addCandidate(candidates, seen, cursor / "src" / kPackageName / "models");
    addCandidate(candidates, seen, cursor / kPackageName / "models");
    if (!cursor.has_parent_path() || cursor.parent_path() == cursor) {
      break;
    }
    cursor = cursor.parent_path();
  }

  if (const auto executable = executablePath()) {
    const std::string executable_string = executable->string();
    const std::string marker = "/install/" + std::string(kPackageName) + "/";
    const size_t marker_position = executable_string.find(marker);
    if (marker_position != std::string::npos) {
      const fs::path workspace = executable_string.substr(0, marker_position);
      addCandidate(candidates, seen, workspace / "src" / kPackageName / "models");
    }

    // .../install/perception/lib/perception/executable
    fs::path prefix = executable->parent_path();
    if (prefix.filename() == kPackageName) {
      prefix = prefix.parent_path(); // lib
      if (prefix.filename() == "lib") {
        prefix = prefix.parent_path(); // install/perception
        addCandidate(candidates, seen, prefix / "share" / kPackageName / "models");
      }
    }
  }

  for (const std::string &prefix_text : splitColonList(std::getenv("AMENT_PREFIX_PATH"))) {
    addCandidate(candidates, seen, fs::path(prefix_text) / "share" / kPackageName / "models");
  }

  for (const fs::path &candidate : candidates) {
    if (isRegularDirectory(candidate)) {
      return candidate;
    }
  }

  std::ostringstream message;
  message << "Folder models perception tidak ditemukan. Lokasi yang diperiksa:\n";
  for (const fs::path &candidate : candidates) {
    message << "  - " << candidate.string() << '\n';
  }
  message << "Gunakan --models-dir /path/ke/perception/models bila struktur workspace berbeda.";
  throw std::runtime_error(message.str());
}

std::vector<fs::path> discoverPtModels(const fs::path &models_dir, const Options &options) {
  std::vector<fs::path> models;

  if (options.pt_file) {
    fs::path selected = *options.pt_file;
    if (selected.is_relative()) {
      selected = models_dir / selected;
    }
    selected = canonicalOrAbsolute(selected);
    if (!isRegularFile(selected) || !hasExtension(selected, ".pt")) {
      throw std::runtime_error("File --pt harus berupa file .pt yang valid: " + selected.string());
    }
    models.push_back(selected);
    return models;
  }

  for (const fs::directory_entry &entry : fs::directory_iterator(models_dir)) {
    if (entry.is_regular_file() && hasExtension(entry.path(), ".pt")) {
      models.push_back(canonicalOrAbsolute(entry.path()));
    }
  }

  std::sort(models.begin(), models.end(), [](const fs::path &a, const fs::path &b) {
    return a.filename().string() < b.filename().string();
  });

  if (models.empty()) {
    throw std::runtime_error(
      "Tidak ada file .pt di folder models: " + models_dir.string() +
      "\nSalin model, misalnya yolopv2.pt, ke folder tersebut lalu jalankan converter lagi.");
  }
  return models;
}

bool outputIsUpToDate(const fs::path &input, const fs::path &output) {
  if (!isRegularFile(input) || !isRegularFile(output)) {
    return false;
  }
  std::error_code input_error;
  std::error_code output_error;
  const auto input_time = fs::last_write_time(input, input_error);
  const auto output_time = fs::last_write_time(output, output_error);
  if (input_error || output_error) {
    return false;
  }
  return output_time >= input_time && fs::file_size(output) > 0U;
}

fs::path uniqueTemporaryPath(const fs::path &directory, const std::string &stem, const std::string &extension) {
  const auto timestamp = std::chrono::high_resolution_clock::now().time_since_epoch().count();
  std::ostringstream name;
  name << "." << stem << "_";
#if defined(__linux__)
  name << ::getpid() << "_";
#endif
  name << timestamp << extension;
  return directory / name.str();
}

void writeTextFile(const fs::path &path, const std::string &content) {
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  if (!output) {
    throw std::runtime_error("Tidak dapat menulis file sementara: " + path.string());
  }
  output << content;
  output.flush();
  if (!output) {
    throw std::runtime_error("Gagal menyelesaikan penulisan file: " + path.string());
  }
}

// Exporter Python ini sengaja tertanam di executable agar pengguna tidak perlu
// mengelola script tambahan. Fokus utamanya adalah model TorchScript YOLOPv2
// resmi. Exporter memilih hanya delapan output yang dibutuhkan node C++:
// tiga head deteksi, tiga anchor-grid, drivable-area, dan lane.
const char *kEmbeddedPythonExporter = R"PY(
import argparse
import inspect
import json
import sys
import traceback
from pathlib import Path

OUTPUT_NAMES = [
    "det_stride_8", "det_stride_16", "det_stride_32",
    "anchor_stride_8", "anchor_stride_16", "anchor_stride_32",
    "drivable", "lane",
]


def log(message):
    print(f"[YOLOPv2 OFFICIAL CONVERTER] {message}", flush=True)


def load_pytorch_model(pt_path):
    import torch
    errors = []
    try:
        model = torch.jit.load(str(pt_path), map_location="cpu")
        log("Model dibaca sebagai TorchScript resmi YOLOPv2.")
        return model
    except Exception as error:
        errors.append("torch.jit.load: " + repr(error))
    try:
        try:
            checkpoint = torch.load(str(pt_path), map_location="cpu", weights_only=False)
        except TypeError:
            checkpoint = torch.load(str(pt_path), map_location="cpu")
        if isinstance(checkpoint, torch.nn.Module):
            return checkpoint
        if isinstance(checkpoint, dict):
            for key in ("ema", "model", "module", "network", "net"):
                candidate = checkpoint.get(key)
                if isinstance(candidate, torch.nn.Module):
                    log(f"Model diambil dari checkpoint['{key}'].")
                    return candidate
    except Exception as error:
        errors.append("torch.load: " + repr(error))
    raise RuntimeError("Model .pt tidak dapat dimuat:\n  - " + "\n  - ".join(errors))


def expected_shapes(h, w):
    return {
        "det_stride_8": (1, 255, h // 8, w // 8),
        "det_stride_16": (1, 255, h // 16, w // 16),
        "det_stride_32": (1, 255, h // 32, w // 32),
        "anchor_stride_8": (1, 3, 1, 1, 2),
        "anchor_stride_16": (1, 3, 1, 1, 2),
        "anchor_stride_32": (1, 3, 1, 1, 2),
        "drivable": (1, 2, h, w),
        "lane": (1, 1, h, w),
    }


def extract_official_outputs(output, h, w):
    """Kontrak persis dari demo.py: [pred, anchor_grid], seg, ll = model(img)."""
    import torch
    if not isinstance(output, (list, tuple)) or len(output) != 3:
        raise RuntimeError(
            "Output root bukan struktur resmi YOLOPv2 3-item: ([pred,anchor_grid], seg, ll)."
        )
    det_pack, seg, lane = output
    if not isinstance(det_pack, (list, tuple)) or len(det_pack) != 2:
        raise RuntimeError("Output deteksi bukan [pred, anchor_grid] resmi YOLOPv2.")
    pred, anchor_grid = det_pack
    if not isinstance(pred, (list, tuple)) or len(pred) != 3:
        raise RuntimeError("pred harus berisi tepat 3 detection head stride 8/16/32.")
    if not isinstance(anchor_grid, (list, tuple)) or len(anchor_grid) != 3:
        raise RuntimeError("anchor_grid harus berisi tepat 3 tensor stride 8/16/32.")

    outputs = tuple(pred) + tuple(anchor_grid) + (seg, lane)
    if len(outputs) != 8 or not all(isinstance(t, torch.Tensor) for t in outputs):
        raise RuntimeError("Semua 8 output resmi YOLOPv2 harus berupa torch.Tensor.")

    shapes = expected_shapes(h, w)
    for name, tensor in zip(OUTPUT_NAMES, outputs):
        actual = tuple(int(v) for v in tensor.shape)
        if actual != shapes[name]:
            raise RuntimeError(f"Shape PT {name} salah: {actual} != {shapes[name]}")
        if not torch.isfinite(tensor.detach().float()).all():
            raise RuntimeError(f"Output PT {name} mengandung NaN/Inf.")
        log(f"PT {name}: shape={actual} dtype={tensor.dtype}")
    return outputs


class YolopV2OfficialOutputWrapper:
    @staticmethod
    def create(model):
        import torch

        class Wrapper(torch.nn.Module):
            def __init__(self, inner):
                super().__init__()
                self.inner = inner

            def forward(self, images):
                output = self.inner(images)
                det_pack = output[0]
                pred = det_pack[0]
                anchors = det_pack[1]
                return (
                    pred[0], pred[1], pred[2],
                    anchors[0], anchors[1], anchors[2],
                    output[1], output[2],
                )

        return Wrapper(model)


def static_shape(value_info):
    dims = []
    for d in value_info.type.tensor_type.shape.dim:
        dims.append(int(d.dim_value) if d.HasField("dim_value") else None)
    return dims


def validate_onnx_structure(path, h, w):
    import onnx
    model = onnx.load(str(path))
    onnx.checker.check_model(model)
    try:
        model = onnx.shape_inference.infer_shapes(model)
    except Exception as error:
        log(f"Shape inference warning: {error}")
    outputs = {o.name: static_shape(o) for o in model.graph.output}
    inputs = {i.name: static_shape(i) for i in model.graph.input}
    log("Input ONNX: " + "; ".join(f"{k}={v}" for k, v in inputs.items()))
    log("Output ONNX: " + "; ".join(f"{k}={v}" for k, v in outputs.items()))
    if inputs.get("images") != [1, 3, h, w]:
        raise RuntimeError(f"Input ONNX images salah: {inputs.get('images')} != {[1,3,h,w]}")
    if list(outputs.keys()) != OUTPUT_NAMES:
        raise RuntimeError(f"Nama/urutan output ONNX berubah: {list(outputs.keys())}")
    shapes = expected_shapes(h, w)
    for name in OUTPUT_NAMES:
        if outputs.get(name) != list(shapes[name]):
            raise RuntimeError(f"Shape ONNX {name} salah: {outputs.get(name)} != {list(shapes[name])}")
    return model


def parity_path(prefix, suffix):
    return Path(str(prefix) + suffix)


def make_parity_input(h, w):
    import torch
    torch.manual_seed(20260811)
    # Domain input setelah preprocessing resmi: RGB NCHW float dalam [0,1].
    return torch.rand((1, 3, h, w), dtype=torch.float32)


def save_pt_parity_reference(model, h, w, prefix):
    import numpy as np
    import torch
    x = make_parity_input(h, w)
    with torch.no_grad():
        outputs = extract_official_outputs(model(x), h, w)
    x.numpy().astype(np.float32).tofile(parity_path(prefix, ".input.f32"))
    for name, tensor in zip(OUTPUT_NAMES, outputs):
        tensor.detach().cpu().float().numpy().astype(np.float32).tofile(
            parity_path(prefix, f".{name}.f32")
        )
    metadata = {
        "input_shape": [1, 3, h, w],
        "output_names": OUTPUT_NAMES,
        "output_shapes": {k: list(v) for k, v in expected_shapes(h, w).items()},
        "seed": 20260811,
    }
    parity_path(prefix, ".meta.json").write_text(json.dumps(metadata, indent=2))
    log(f"Reference parity PT tersimpan: {prefix}.*")
    return x, outputs


def compare_arrays(name, expected, actual, precision, is_anchor=False):
    import numpy as np
    expected = np.asarray(expected, dtype=np.float32).reshape(-1)
    actual = np.asarray(actual, dtype=np.float32).reshape(-1)
    if expected.size != actual.size:
        raise RuntimeError(f"Parity {name}: jumlah elemen beda {actual.size} != {expected.size}")
    if not np.isfinite(actual).all():
        raise RuntimeError(f"Parity {name}: TensorRT mengandung NaN/Inf")
    diff = actual - expected
    abs_diff = np.abs(diff)
    rmse = float(np.sqrt(np.mean(diff * diff))) if diff.size else 0.0
    ref_rms = float(np.sqrt(np.mean(expected * expected))) if expected.size else 0.0
    nrmse = rmse / max(ref_rms, 1.0e-3)
    p99 = float(np.percentile(abs_diff, 99.0)) if abs_diff.size else 0.0
    ref_p99 = float(np.percentile(np.abs(expected), 99.0)) if expected.size else 0.0
    p99_norm = p99 / max(ref_p99, 1.0)
    max_abs = float(abs_diff.max()) if abs_diff.size else 0.0

    if is_anchor:
        ok = bool(np.allclose(actual, expected, rtol=1.0e-5, atol=1.0e-4))
        limit_text = "allclose(rtol=1e-5, atol=1e-4)"
    elif precision == "fp32":
        ok = nrmse <= 0.003 and p99_norm <= 0.010
        limit_text = "NRMSE<=0.003, P99norm<=0.010"
    else:
        # FP16 adalah jalur resmi CUDA demo.py. Toleransi ini menangkap regresi
        # besar tanpa menuntut bit-identik terhadap referensi PT float32 CPU.
        ok = nrmse <= 0.035 and p99_norm <= 0.080
        limit_text = "NRMSE<=0.035, P99norm<=0.080"

    log(
        f"PARITY {name}: NRMSE={nrmse:.6g} P99abs={p99:.6g} "
        f"P99norm={p99_norm:.6g} MAXabs={max_abs:.6g} -> {'PASS' if ok else 'FAIL'} "
        f"({limit_text})"
    )
    if not ok:
        raise RuntimeError(f"Numerical parity gagal pada output {name}")


def maybe_validate_onnxruntime(onnx_path, x, pt_outputs):
    try:
        import numpy as np
        import onnxruntime as ort
    except Exception:
        log("onnxruntime tidak tersedia; PT->ONNX numerical check dilewati. PT->TensorRT parity tetap wajib.")
        return
    session = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    ort_outputs = session.run(OUTPUT_NAMES, {"images": x.numpy().astype(np.float32)})
    for name, pt_tensor, ort_tensor in zip(OUTPUT_NAMES, pt_outputs, ort_outputs):
        expected = pt_tensor.detach().cpu().float().numpy()
        if not np.allclose(expected, ort_tensor, rtol=2.0e-4, atol=2.0e-5):
            diff = np.abs(expected - ort_tensor)
            raise RuntimeError(
                f"PT->ONNX parity gagal {name}: max_abs={float(diff.max())}, mean_abs={float(diff.mean())}"
            )
    log("PT -> ONNX numerical parity: PASS (onnxruntime).")


def export_onnx(args):
    import torch
    log(f"Python={sys.executable} PyTorch={torch.__version__}")
    log(f"PT={args.pt} ONNX={args.onnx} input=[1,3,{args.height},{args.width}] opset={args.opset}")
    if (args.batch, args.channels, args.height, args.width) != (1, 3, 384, 640):
        raise RuntimeError("YOLOPv2 official geometry wajib [1,3,384,640].")
    torch.set_grad_enabled(False)
    model = load_pytorch_model(Path(args.pt)).float().cpu().eval()
    dummy = torch.zeros((1, 3, args.height, args.width), dtype=torch.float32)
    with torch.no_grad():
        extract_official_outputs(model(dummy), args.height, args.width)

    wrapper = YolopV2OfficialOutputWrapper.create(model).eval()
    # Trace hanya wrapper I/O flatten; numerik inner model tetap berasal dari model resmi.
    traced = torch.jit.trace(wrapper, dummy, strict=False).eval()
    with torch.no_grad():
        traced_outputs = traced(dummy)
    shapes = expected_shapes(args.height, args.width)
    for name, tensor in zip(OUTPUT_NAMES, traced_outputs):
        if tuple(int(v) for v in tensor.shape) != shapes[name]:
            raise RuntimeError(f"Trace wrapper mengubah shape {name}: {tuple(tensor.shape)}")

    params = inspect.signature(torch.onnx.export).parameters
    kwargs = dict(
        export_params=True,
        opset_version=args.opset,
        do_constant_folding=True,
        input_names=["images"],
        output_names=OUTPUT_NAMES,
        dynamic_axes=None,
    )
    if "dynamo" in params:
        kwargs["dynamo"] = False
    try:
        torch.onnx.export(traced, dummy, args.onnx, **kwargs)
    except Exception:
        log(traceback.format_exc())
        raise

    validate_onnx_structure(Path(args.onnx), args.height, args.width)
    if args.simplify:
        try:
            import onnx
            from onnxsim import simplify
            original = onnx.load(args.onnx)
            simplified, ok = simplify(original)
            if not ok:
                raise RuntimeError("onnxsim check=False")
            onnx.save(simplified, args.onnx)
            validate_onnx_structure(Path(args.onnx), args.height, args.width)
            log("ONNX simplification valid (opsional, bukan default).")
        except Exception as error:
            raise RuntimeError(f"Simplifikasi diminta tetapi gagal: {error}")

    if args.validate:
        x, pt_outputs = save_pt_parity_reference(model, args.height, args.width, args.parity_prefix)
        maybe_validate_onnxruntime(Path(args.onnx), x, pt_outputs)
    log(f"ONNX VALID: {Path(args.onnx).stat().st_size/(1024*1024):.2f} MiB")


def load_trtexec_json(path):
    payload = json.loads(Path(path).read_text())
    if isinstance(payload, dict):
        for key in ("outputs", "tensors", "data"):
            if isinstance(payload.get(key), list):
                payload = payload[key]
                break
    if not isinstance(payload, list):
        raise RuntimeError("Format JSON --exportOutput trtexec tidak dikenali.")
    records = {}
    for item in payload:
        if not isinstance(item, dict) or "name" not in item:
            continue
        records[str(item["name"])] = item
    return records


def compare_trt(args):
    import numpy as np
    prefix = Path(args.parity_prefix)
    meta_path = parity_path(prefix, ".meta.json")
    if not meta_path.is_file():
        raise RuntimeError(f"Metadata parity PT tidak ditemukan: {meta_path}")
    meta = json.loads(meta_path.read_text())
    if meta.get("output_names") != OUTPUT_NAMES:
        raise RuntimeError("Metadata parity output names tidak sesuai kontrak V19.")
    records = load_trtexec_json(args.trt_json)
    missing = [name for name in OUTPUT_NAMES if name not in records]
    if missing:
        raise RuntimeError(f"TensorRT exportOutput tidak memiliki output: {missing}")

    for name in OUTPUT_NAMES:
        expected_shape = tuple(meta["output_shapes"][name])
        item = records[name]
        dims = item.get("dimensions", item.get("shape", []))
        if tuple(int(v) for v in dims) != expected_shape:
            raise RuntimeError(f"TensorRT shape {name} salah: {dims} != {expected_shape}")
        values = item.get("values")
        if not isinstance(values, list):
            raise RuntimeError(f"TensorRT JSON {name} tidak memiliki array values.")
        expected = np.fromfile(parity_path(prefix, f".{name}.f32"), dtype=np.float32)
        actual = np.asarray(values, dtype=np.float32)
        compare_arrays(name, expected, actual, args.precision, name.startswith("anchor_stride_"))
    log("PT -> TensorRT end-to-end numerical parity: PASS untuk seluruh 8 output.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=("export", "compare"), default="export")
    ap.add_argument("--pt")
    ap.add_argument("--onnx")
    ap.add_argument("--batch", type=int, default=1)
    ap.add_argument("--channels", type=int, default=3)
    ap.add_argument("--height", type=int, default=384)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--opset", type=int, default=17)
    ap.add_argument("--simplify", action="store_true")
    ap.add_argument("--validate", action="store_true")
    ap.add_argument("--parity-prefix", required=True)
    ap.add_argument("--trt-json")
    ap.add_argument("--precision", choices=("fp16", "fp32"), default="fp16")
    args = ap.parse_args()
    if args.mode == "export":
        if not args.pt or not args.onnx:
            ap.error("--pt dan --onnx wajib untuk mode export")
        export_onnx(args)
    else:
        if not args.trt_json:
            ap.error("--trt-json wajib untuk mode compare")
        compare_trt(args)


if __name__ == "__main__":
    main()
)PY";

fs::path writeTemporaryExporter(const fs::path &models_dir) {
  const fs::path exporter = uniqueTemporaryPath(models_dir, "yolopv2_exporter", ".py");
  writeTextFile(exporter, kEmbeddedPythonExporter);
  std::error_code error;
  fs::permissions(
    exporter,
    fs::perms::owner_read | fs::perms::owner_write | fs::perms::owner_exec,
    fs::perm_options::replace,
    error);
  return exporter;
}

std::optional<fs::path> findExecutableInPath(const std::string &name) {
  const auto result = runCommandCapture("command -v " + shellQuote(name));
  if (result.exit_code != 0) {
    return std::nullopt;
  }
  std::stringstream stream(result.output);
  std::string first_line;
  std::getline(stream, first_line);
  if (first_line.empty()) {
    return std::nullopt;
  }
  const fs::path candidate = first_line;
  if (isRegularFile(candidate)) {
    return canonicalOrAbsolute(candidate);
  }
  return std::nullopt;
}

fs::path resolveTrtexec(const Options &options) {
  if (options.trtexec) {
    const fs::path selected = canonicalOrAbsolute(*options.trtexec);
    if (!isRegularFile(selected)) {
      throw std::runtime_error("File trtexec tidak ditemukan: " + selected.string());
    }
    return selected;
  }

  if (const auto from_path = findExecutableInPath("trtexec")) {
    return *from_path;
  }

  const std::array<fs::path, 5> common_locations = {
    fs::path("/usr/src/tensorrt/bin/trtexec"),
    fs::path("/usr/local/TensorRT/bin/trtexec"),
    fs::path("/opt/tensorrt/bin/trtexec"),
    fs::path("/usr/bin/trtexec"),
    fs::path("/usr/local/bin/trtexec")
  };
  for (const fs::path &candidate : common_locations) {
    if (isRegularFile(candidate)) {
      return canonicalOrAbsolute(candidate);
    }
  }

  throw std::runtime_error(
    "trtexec tidak ditemukan. Pastikan TensorRT terpasang dan trtexec berada di PATH, "
    "atau gunakan --trtexec /usr/src/tensorrt/bin/trtexec.");
}

bool containsFlag(const std::string &help, const std::string &flag) {
  return help.find(flag) != std::string::npos;
}

std::string makeTrtexecCommand(
  const fs::path &trtexec,
  const fs::path &onnx,
  const fs::path &engine_temporary,
  const fs::path &timing_cache,
  const Options &options,
  const std::string &help,
  bool compatibility_mode,
  bool force_fp32)
{
  std::ostringstream command;
  command << shellQuote(trtexec)
          << " --onnx=" << shellQuote(onnx)
          << " --saveEngine=" << shellQuote(engine_temporary);

  const bool fp16_requested = lowercase(options.precision) == "fp16" && !force_fp32;
  if (fp16_requested && containsFlag(help, "--fp16")) {
    command << " --fp16";
  }

  if (containsFlag(help, "--skipInference")) {
    command << " --skipInference";
  } else if (containsFlag(help, "--buildOnly")) {
    command << " --buildOnly";
  }

  if (containsFlag(help, "--memPoolSize")) {
    command << " --memPoolSize=workspace:" << options.workspace_mib;
  } else if (containsFlag(help, "--workspace")) {
    command << " --workspace=" << options.workspace_mib;
  }

  if (!compatibility_mode && containsFlag(help, "--builderOptimizationLevel")) {
    command << " --builderOptimizationLevel=" << options.builder_optimization_level;
  }

  if (!compatibility_mode && containsFlag(help, "--timingCacheFile")) {
    command << " --timingCacheFile=" << shellQuote(timing_cache);
  }

  if (options.verbose && containsFlag(help, "--verbose")) {
    command << " --verbose";
  }
  return command.str();
}

void atomicReplace(const fs::path &temporary, const fs::path &destination) {
  if (!isRegularFile(temporary) || fs::file_size(temporary) == 0U) {
    throw std::runtime_error("Output sementara tidak valid: " + temporary.string());
  }

  std::error_code error;
  fs::rename(temporary, destination, error);
  if (!error) {
    return;
  }

  // rename dapat gagal bila destination sudah ada atau filesystem berbeda.
  fs::remove(destination, error);
  error.clear();
  fs::rename(temporary, destination, error);
  if (error) {
    throw std::runtime_error(
      "Gagal memindahkan output ke tujuan " + destination.string() + ": " + error.message());
  }
}


const std::array<const char *, 8> kOfficialOutputNames = {
  "det_stride_8", "det_stride_16", "det_stride_32",
  "anchor_stride_8", "anchor_stride_16", "anchor_stride_32",
  "drivable", "lane"
};

fs::path parityPrefixForOnnx(const fs::path &onnx) {
  return fs::path(onnx.string() + ".parity");
}

bool parityReferenceComplete(const fs::path &pt, const fs::path &onnx) {
  const fs::path prefix = parityPrefixForOnnx(onnx);
  std::vector<fs::path> required;
  required.push_back(fs::path(prefix.string() + ".input.f32"));
  required.push_back(fs::path(prefix.string() + ".meta.json"));
  for (const char *name : kOfficialOutputNames) {
    required.push_back(fs::path(prefix.string() + "." + name + ".f32"));
  }
  for (const fs::path &path : required) {
    if (!isRegularFile(path) || fs::file_size(path) == 0U || !outputIsUpToDate(pt, path)) {
      return false;
    }
  }
  return true;
}

void validateTensorRtParity(
  const fs::path &python_exporter,
  const fs::path &trtexec,
  const std::string &trtexec_help,
  const fs::path &temporary_engine,
  const fs::path &onnx,
  const Options &options,
  const std::string &effective_precision)
{
  if (!options.validate) {
    std::cout << "[PERINGATAN] Numerical parity PT->TensorRT dilewati karena --no-validate.\n";
    return;
  }
  if (!containsFlag(trtexec_help, "--loadInputs") || !containsFlag(trtexec_help, "--exportOutput")) {
    throw std::runtime_error(
      "trtexec target tidak mendukung --loadInputs/--exportOutput; validasi numerical parity wajib V19 "
      "sehingga engine tidak akan menggantikan engine lama. Upgrade TensorRT/trtexec atau gunakan --no-validate secara eksplisit.");
  }

  const fs::path prefix = parityPrefixForOnnx(onnx);
  const fs::path input = fs::path(prefix.string() + ".input.f32");
  const fs::path json_output = uniqueTemporaryPath(onnx.parent_path(), onnx.stem().string() + "_trt_parity", ".json");
  std::error_code cleanup_error;
  fs::remove(json_output, cleanup_error);

  std::ostringstream inference;
  inference << shellQuote(trtexec)
            << " --loadEngine=" << shellQuote(temporary_engine)
            << " --loadInputs=images:" << shellQuote(input)
            << " --exportOutput=" << shellQuote(json_output);
  if (containsFlag(trtexec_help, "--iterations")) inference << " --iterations=1";
  if (containsFlag(trtexec_help, "--warmUp")) inference << " --warmUp=0";
  if (containsFlag(trtexec_help, "--duration")) inference << " --duration=0";

  std::cout << "\n=== TAHAP 3: PT -> TENSORRT NUMERICAL PARITY ===\n";
  const auto infer_result = runCommandCapture(inference.str());
  std::cout << infer_result.output;
  if (infer_result.exit_code != 0 || !isRegularFile(json_output) || fs::file_size(json_output) == 0U) {
    fs::remove(json_output, cleanup_error);
    throw std::runtime_error("Inference parity TensorRT gagal; engine sementara ditolak.");
  }

  std::ostringstream compare;
  compare << shellQuote(options.python)
          << ' ' << shellQuote(python_exporter)
          << " --mode compare"
          << " --parity-prefix " << shellQuote(prefix)
          << " --trt-json " << shellQuote(json_output)
          << " --precision " << shellQuote(effective_precision);
  const auto compare_result = runCommandCapture(compare.str());
  std::cout << compare_result.output;
  fs::remove(json_output, cleanup_error);
  if (compare_result.exit_code != 0) {
    throw std::runtime_error("PT -> TensorRT numerical parity gagal; engine sementara ditolak dan engine lama dipertahankan.");
  }
}

void exportPtToOnnx(
  const fs::path &python_exporter,
  const fs::path &pt,
  const fs::path &onnx,
  const Options &options)
{
  const bool parity_ready = !options.validate || parityReferenceComplete(pt, onnx);
  if (!options.force && outputIsUpToDate(pt, onnx) && parity_ready) {
    std::cout << "[SKIP] ONNX + parity reference masih baru: " << onnx << '\n';
    return;
  }

  const fs::path temporary = uniqueTemporaryPath(onnx.parent_path(), onnx.stem().string(), ".onnx");
  std::error_code cleanup_error;
  fs::remove(temporary, cleanup_error);

  std::ostringstream command;
  command << shellQuote(options.python)
          << ' ' << shellQuote(python_exporter)
          << " --mode export"
          << " --pt " << shellQuote(pt)
          << " --onnx " << shellQuote(temporary)
          << " --batch " << options.batch
          << " --channels " << options.channels
          << " --height " << options.height
          << " --width " << options.width
          << " --opset " << options.opset
          << " --parity-prefix " << shellQuote(onnx.string() + ".parity");
  if (options.simplify) {
    command << " --simplify";
  }
  if (options.validate) {
    command << " --validate";
  }

  std::cout << "\n=== TAHAP 1: PT -> ONNX ===\n";
  std::cout << "Model : " << pt << '\n';
  std::cout << "ONNX  : " << onnx << '\n';

  const auto result = runCommandCapture(command.str());
  std::cout << result.output;
  if (result.exit_code != 0) {
    fs::remove(temporary, cleanup_error);
    throw std::runtime_error("Ekspor PT -> ONNX gagal dengan exit code " + std::to_string(result.exit_code));
  }

  atomicReplace(temporary, onnx);
  std::cout << "[OK] ONNX berhasil dibuat: " << onnx
            << " (" << (fs::file_size(onnx) / (1024.0 * 1024.0)) << " MiB)\n";
}

void buildOnnxToEngine(
  const fs::path &python_exporter,
  const fs::path &trtexec,
  const std::string &trtexec_help,
  const fs::path &onnx,
  const fs::path &engine,
  const Options &options)
{
  if (!options.force && outputIsUpToDate(onnx, engine)) {
    std::cout << "[SKIP] Engine masih baru dan tidak perlu dibuat ulang: " << engine << '\n';
    return;
  }

  const fs::path temporary = uniqueTemporaryPath(engine.parent_path(), engine.stem().string(), ".engine");
  const fs::path timing_cache = engine.parent_path() / "tensorrt_timing.cache";
  std::error_code cleanup_error;
  fs::remove(temporary, cleanup_error);

  std::cout << "\n=== TAHAP 2: ONNX -> TENSORRT ENGINE ===\n";
  std::cout << "ONNX       : " << onnx << '\n';
  std::cout << "Engine     : " << engine << '\n';
  std::cout << "Precision  : " << options.precision << '\n';
  std::cout << "Workspace  : " << options.workspace_mib << " MiB\n";

  struct Attempt {
    bool compatibility_mode;
    bool force_fp32;
    const char *description;
  };

  std::vector<Attempt> attempts;
  attempts.push_back({false, false, "opsi optimal + timing cache"});
  attempts.push_back({true, false, "opsi kompatibilitas TensorRT"});
  if (lowercase(options.precision) == "fp16") {
    attempts.push_back({true, true, "fallback FP32"});
  }

  std::string all_errors;
  for (size_t index = 0; index < attempts.size(); ++index) {
    fs::remove(temporary, cleanup_error);
    const Attempt &attempt = attempts[index];
    const std::string command = makeTrtexecCommand(
      trtexec,
      onnx,
      temporary,
      timing_cache,
      options,
      trtexec_help,
      attempt.compatibility_mode,
      attempt.force_fp32);

    std::cout << "\nPercobaan " << (index + 1U) << "/" << attempts.size()
              << ": " << attempt.description << '\n';
    if (options.verbose) {
      std::cout << "Perintah: " << command << '\n';
    }

    const auto result = runCommandCapture(command);
    std::cout << result.output;
    if (result.exit_code == 0 && isRegularFile(temporary) && fs::file_size(temporary) > 0U) {
      const bool fp16_effective =
        lowercase(options.precision) == "fp16" && !attempt.force_fp32 && containsFlag(trtexec_help, "--fp16");
      const std::string effective_precision = fp16_effective ? "fp16" : "fp32";
      try {
        validateTensorRtParity(
          python_exporter, trtexec, trtexec_help, temporary, onnx, options, effective_precision);
      } catch (const std::exception &error) {
        all_errors += "\n--- " + std::string(attempt.description) + " parity ---\n" + error.what() + "\n";
        fs::remove(temporary, cleanup_error);
        std::cerr << "[REJECT] " << error.what() << '\n';
        continue;
      }
      atomicReplace(temporary, engine);
      std::cout << "[OK] TensorRT engine berhasil dibuat + parity PASS: " << engine
                << " precision=" << effective_precision
                << " (" << (fs::file_size(engine) / (1024.0 * 1024.0)) << " MiB)\n";
      return;
    }

    all_errors += "\n--- " + std::string(attempt.description) + " ---\n" + result.output;
  }

  fs::remove(temporary, cleanup_error);
  throw std::runtime_error("Semua percobaan ONNX -> engine gagal." + all_errors);
}

class ConversionLock {
public:
  explicit ConversionLock(const fs::path &models_dir)
  : lock_dir_(models_dir / ".converter_pt_to_onnx_to_engine.lock")
  {
    std::error_code error;
    if (!fs::create_directory(lock_dir_, error)) {
      throw std::runtime_error(
        "Converter lain kemungkinan sedang berjalan karena lock tersedia: " + lock_dir_.string() +
        "\nBila tidak ada proses converter, hapus folder lock tersebut secara manual.");
    }
    std::ofstream owner(lock_dir_ / "info.txt");
#if defined(__linux__)
    owner << "pid=" << ::getpid() << '\n';
#endif
    owner << "cwd=" << fs::current_path().string() << '\n';
  }

  ~ConversionLock() {
    std::error_code error;
    fs::remove_all(lock_dir_, error);
  }

  ConversionLock(const ConversionLock &) = delete;
  ConversionLock &operator=(const ConversionLock &) = delete;

private:
  fs::path lock_dir_;
};

int parsePositiveInteger(const std::string &text, const std::string &option) {
  try {
    const int value = std::stoi(text);
    if (value <= 0) {
      throw std::runtime_error("");
    }
    return value;
  } catch (...) {
    throw std::runtime_error("Nilai " + option + " harus bilangan bulat positif: " + text);
  }
}

void printHelp(const char *program) {
  std::cout
    << "Konverter model perception: PT -> ONNX -> TensorRT engine\n\n"
    << "Pemakaian utama:\n"
    << "  ros2 run perception converter_pt_to_onnx_to_engine\n\n"
    << "Program tanpa argumen akan mengonversi semua *.pt dalam folder models.\n\n"
    << "Opsi:\n"
    << "  --models-dir PATH       Folder models manual\n"
    << "  --pt FILE               Hanya konversi satu file .pt\n"
    << "  --python COMMAND        Python executable, default: python3\n"
    << "  --trtexec PATH          Lokasi trtexec manual\n"
    << "  --height N              Tinggi input model, default: 384\n"
    << "  --width N               Lebar input model, default: 640\n"
    << "  --opset N               ONNX opset, default: 17\n"
    << "  --precision fp16|fp32   Precision engine, default: fp16\n"
    << "  --workspace N           Workspace TensorRT MiB, default: 4096\n"
    << "  --builder-level N       Optimization level, default: 5\n"
    << "  --force                 Buat ulang ONNX dan engine walau masih baru\n"
    << "  --onnx-only             Hanya buat ONNX, jangan membuat engine\n"
    << "  --simplify              Jalankan onnx-simplifier (opsional, default OFF)\n"
    << "  --no-validate           Lewati checker ONNX tambahan bila memungkinkan\n"
    << "  --keep-exporter         Simpan script exporter Python sementara\n"
    << "  --verbose               Tampilkan perintah trtexec lengkap\n"
    << "  -h, --help              Tampilkan bantuan ini\n\n"
    << "Contoh:\n"
    << "  ros2 run perception converter_pt_to_onnx_to_engine --force\n"
    << "  ros2 run perception converter_pt_to_onnx_to_engine --pt yolopv2.pt\n"
    << "  ros2 run perception converter_pt_to_onnx_to_engine --precision fp32\n";
  (void)program;
}

Options parseArguments(int argc, char **argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    auto requireValue = [&](const std::string &name) -> std::string {
      if (index + 1 >= argc) {
        throw std::runtime_error("Opsi " + name + " memerlukan nilai.");
      }
      return argv[++index];
    };

    if (argument == "-h" || argument == "--help") {
      printHelp(argv[0]);
      std::exit(EXIT_SUCCESS);
    } else if (argument == "--models-dir") {
      options.models_dir = requireValue(argument);
    } else if (argument == "--pt") {
      options.pt_file = requireValue(argument);
    } else if (argument == "--python") {
      options.python = requireValue(argument);
    } else if (argument == "--trtexec") {
      options.trtexec = requireValue(argument);
    } else if (argument == "--height") {
      options.height = parsePositiveInteger(requireValue(argument), argument);
    } else if (argument == "--width") {
      options.width = parsePositiveInteger(requireValue(argument), argument);
    } else if (argument == "--opset") {
      options.opset = parsePositiveInteger(requireValue(argument), argument);
    } else if (argument == "--workspace") {
      options.workspace_mib = parsePositiveInteger(requireValue(argument), argument);
    } else if (argument == "--builder-level") {
      options.builder_optimization_level = parsePositiveInteger(requireValue(argument), argument);
    } else if (argument == "--precision") {
      options.precision = lowercase(requireValue(argument));
      if (options.precision != "fp16" && options.precision != "fp32") {
        throw std::runtime_error("--precision hanya menerima fp16 atau fp32.");
      }
    } else if (argument == "--force") {
      options.force = true;
    } else if (argument == "--onnx-only") {
      options.onnx_only = true;
    } else if (argument == "--simplify") {
      options.simplify = true;
    } else if (argument == "--no-simplify") {
      options.simplify = false;
    } else if (argument == "--no-validate") {
      options.validate = false;
    } else if (argument == "--keep-exporter") {
      options.keep_temporary_exporter = true;
    } else if (argument == "--verbose") {
      options.verbose = true;
    } else {
      throw std::runtime_error("Opsi tidak dikenal: " + argument + ". Gunakan --help.");
    }
  }

  if (options.batch != 1) {
    throw std::runtime_error("Batch harus 1 agar sesuai dengan perception_node.");
  }
  if (options.height != 384 || options.width != 640) {
    throw std::runtime_error("YOLOPv2 V19 mengikuti master: input TensorRT wajib statis [1,3,384,640].");
  }
  return options;
}

std::optional<fs::path> installedModelsDirectoryForSource(const fs::path &models_dir) {
  const std::string source_path = canonicalOrAbsolute(models_dir).string();
  const std::string marker = "/src/" + std::string(kPackageName) + "/models";
  const size_t position = source_path.rfind(marker);
  if (position == std::string::npos) {
    return std::nullopt;
  }

  const fs::path workspace = source_path.substr(0, position);
  const fs::path installed =
    workspace / "install" / kPackageName / "share" / kPackageName / "models";
  if (isRegularDirectory(installed)) {
    return canonicalOrAbsolute(installed);
  }
  return std::nullopt;
}

// Launch ROS mencari engine melalui package share di install space. Pada
// --symlink-install file yang sudah ada ketika build biasanya berupa symlink,
// tetapi file baru yang dibuat setelah build belum tentu langsung muncul. Oleh
// karena itu hasil converter disalin juga ke install share bila diperlukan.
void synchronizeGeneratedFileToInstall(
  const fs::path &source_models_dir,
  const fs::path &generated_file)
{
  const auto installed_models = installedModelsDirectoryForSource(source_models_dir);
  if (!installed_models || !isRegularFile(generated_file)) {
    return;
  }

  const fs::path destination = *installed_models / generated_file.filename();
  std::error_code equivalent_error;
  if (fs::exists(destination) && fs::equivalent(generated_file, destination, equivalent_error) && !equivalent_error) {
    return;
  }

  std::error_code copy_error;
  fs::copy_file(generated_file, destination, fs::copy_options::overwrite_existing, copy_error);
  if (copy_error) {
    std::cerr << "[PERINGATAN] Gagal menyinkronkan " << generated_file.filename()
              << " ke install share: " << copy_error.message() << '\n';
  } else {
    std::cout << "[SYNC] Hasil tersedia untuk launch ROS: " << destination << '\n';
  }
}

void printHeader(const fs::path &models_dir, const std::vector<fs::path> &models, const Options &options) {
  std::cout
    << "============================================================\n"
    << " ASTRA YOLOP GPU - KONVERTER PT -> ONNX -> ENGINE\n"
    << "============================================================\n"
    << "Folder models : " << models_dir << '\n'
    << "Jumlah model  : " << models.size() << '\n'
    << "Input model   : [1, 3, " << options.height << ", " << options.width << "]\n"
    << "ONNX opset    : " << options.opset << '\n'
    << "Precision     : " << options.precision << '\n'
    << "Force rebuild : " << (options.force ? "ya" : "tidak") << '\n'
    << "============================================================\n";
}

} // namespace

int main(int argc, char **argv) {
  fs::path temporary_exporter;
  try {
    const Options options = parseArguments(argc, argv);

    if (!commandExists(options.python)) {
      throw std::runtime_error(
        "Python executable tidak ditemukan: " + options.python +
        "\nGunakan --python /path/ke/python3 bila memakai virtual environment.");
    }

    const fs::path models_dir = resolveModelsDirectory(options);
    const std::vector<fs::path> models = discoverPtModels(models_dir, options);
    printHeader(models_dir, models, options);

    ConversionLock lock(models_dir);
    temporary_exporter = writeTemporaryExporter(models_dir);

    std::optional<fs::path> trtexec;
    std::string trtexec_help;
    if (!options.onnx_only) {
      trtexec = resolveTrtexec(options);
      const auto help_result = runCommandCapture(shellQuote(*trtexec) + " --help");
      trtexec_help = help_result.output;
      std::cout << "trtexec       : " << *trtexec << '\n';
    }

    size_t success_count = 0U;
    for (const fs::path &pt : models) {
      try {
        const fs::path onnx = pt.parent_path() / (pt.stem().string() + ".onnx");
        const fs::path engine = pt.parent_path() / (pt.stem().string() + ".engine");

        std::cout << "\n############################################################\n";
        std::cout << "Memproses model: " << pt.filename() << '\n';
        std::cout << "############################################################\n";

        exportPtToOnnx(temporary_exporter, pt, onnx, options);
        synchronizeGeneratedFileToInstall(models_dir, onnx);
        if (!options.onnx_only) {
          buildOnnxToEngine(temporary_exporter, *trtexec, trtexec_help, onnx, engine, options);
          synchronizeGeneratedFileToInstall(models_dir, engine);
        }
        ++success_count;
      } catch (const std::exception &error) {
        std::cerr << "\n[ERROR] Model " << pt.filename() << " gagal diproses:\n"
                  << error.what() << "\n";
      }
    }

    if (options.keep_temporary_exporter) {
      const fs::path saved = models_dir / "converter_embedded_exporter_debug.py";
      std::error_code copy_error;
      fs::copy_file(temporary_exporter, saved, fs::copy_options::overwrite_existing, copy_error);
      if (!copy_error) {
        std::cout << "Exporter debug disimpan: " << saved << '\n';
      }
    }

    std::error_code remove_error;
    fs::remove(temporary_exporter, remove_error);
    temporary_exporter.clear();

    std::cout << "\n============================================================\n";
    std::cout << "Selesai: " << success_count << "/" << models.size() << " model berhasil.\n";
    std::cout << "============================================================\n";

    return success_count == models.size() ? EXIT_SUCCESS : EXIT_FAILURE;
  } catch (const std::exception &error) {
    if (!temporary_exporter.empty()) {
      std::error_code remove_error;
      fs::remove(temporary_exporter, remove_error);
    }
    std::cerr << "\n[FATAL] " << error.what() << "\n";
    return EXIT_FAILURE;
  }
}
