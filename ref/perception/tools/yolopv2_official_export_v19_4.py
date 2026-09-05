
import argparse
import inspect
import json
import sys
import traceback
import warnings
from pathlib import Path

# PyTorch 2.8 warns that the TorchScript exporter will be superseded in 2.9.
# For this official YOLOPv2 TorchScript artifact we intentionally keep
# dynamo=False for compatibility and suppress only these two known notices.
warnings.filterwarnings(
    "ignore",
    category=DeprecationWarning,
    message=r"You are using the legacy TorchScript-based ONNX export.*",
)
warnings.filterwarnings(
    "ignore",
    category=UserWarning,
    message=r"no signature found for <torch\.ScriptMethod.*",
)

OUTPUT_NAMES = [
    "det_stride_8", "det_stride_16", "det_stride_32",
    "anchor_stride_8", "anchor_stride_16", "anchor_stride_32",
    "drivable", "lane",
]


def log(message):
    print(f"[YOLOPv2 OFFICIAL CONVERTER] {message}", flush=True)


def load_pytorch_model(pt_path, allow_pickle_checkpoint=False):
    import torch
    errors = []
    try:
        model = torch.jit.load(str(pt_path), map_location="cpu")
        log("Model dibaca sebagai TorchScript resmi YOLOPv2.")
        return model
    except Exception as error:
        errors.append("torch.jit.load: " + repr(error))
    if not allow_pickle_checkpoint:
        raise RuntimeError(
            "Model .pt bukan TorchScript YOLOPv2 yang dapat dibaca torch.jit.load. "
            "Runtime robot menolak fallback torch.load/pickle. Untuk konversi manual "
            "checkpoint tepercaya, gunakan --allow-pickle-checkpoint.\n  - "
            + "\n  - ".join(errors)
        )
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


def _rename_graph_tensor(graph, old_name, new_name):
    """Rename one ONNX tensor everywhere without changing graph numerics."""
    if old_name == new_name:
        return
    for node in graph.node:
        for i, value in enumerate(node.input):
            if value == old_name:
                node.input[i] = new_name
        for i, value in enumerate(node.output):
            if value == old_name:
                node.output[i] = new_name
    for collection in (graph.input, graph.output, graph.value_info):
        for value_info in collection:
            if value_info.name == old_name:
                value_info.name = new_name
    for initializer in graph.initializer:
        if initializer.name == old_name:
            initializer.name = new_name
    # Sparse initializers are uncommon here, but keep the renamer complete.
    for sparse in graph.sparse_initializer:
        if sparse.values.name == old_name:
            sparse.values.name = new_name
        if sparse.indices.name == old_name:
            sparse.indices.name = new_name


def normalize_onnx_output_names(path):
    """
    PyTorch legacy ONNX export may ignore output_names for constant outputs.
    YOLOPv2 anchor_grid tensors are constants, so names such as 774/775/776
    can appear even though output_names was supplied. The tuple order remains
    the explicit Wrapper.forward order, therefore normalize names by position.
    This is metadata-only and does not modify tensor values or operators.
    """
    import onnx
    model = onnx.load(str(path))
    onnx.checker.check_model(model)
    actual_names = [o.name for o in model.graph.output]
    if len(actual_names) != len(OUTPUT_NAMES):
        raise RuntimeError(
            f"Jumlah output ONNX berubah: {len(actual_names)} != {len(OUTPUT_NAMES)}; "
            f"outputs={actual_names}"
        )
    if actual_names == OUTPUT_NAMES:
        return

    # Avoid accidental collision with an unrelated internal tensor name.
    all_tensor_names = set()
    for node in model.graph.node:
        all_tensor_names.update(v for v in node.input if v)
        all_tensor_names.update(v for v in node.output if v)
    all_tensor_names.update(v.name for v in model.graph.input)
    all_tensor_names.update(v.name for v in model.graph.output)
    all_tensor_names.update(v.name for v in model.graph.value_info)
    all_tensor_names.update(v.name for v in model.graph.initializer)

    rename_pairs = [(old, new) for old, new in zip(actual_names, OUTPUT_NAMES) if old != new]
    for old, new in rename_pairs:
        if new in all_tensor_names and new not in actual_names:
            raise RuntimeError(
                f"Tidak aman menormalisasi output ONNX {old}->{new}: nama tujuan sudah dipakai internal."
            )

    # Two-phase rename prevents collisions when an old name equals another desired name.
    temporary_pairs = []
    for index, (old, new) in enumerate(rename_pairs):
        temporary = f"__yolopv2_v19_output_{index}__"
        while temporary in all_tensor_names:
            temporary += "_"
        _rename_graph_tensor(model.graph, old, temporary)
        all_tensor_names.discard(old)
        all_tensor_names.add(temporary)
        temporary_pairs.append((temporary, new))
    for temporary, new in temporary_pairs:
        _rename_graph_tensor(model.graph, temporary, new)

    normalized_names = [o.name for o in model.graph.output]
    if normalized_names != OUTPUT_NAMES:
        raise RuntimeError(f"Normalisasi nama output ONNX gagal: {normalized_names}")
    onnx.checker.check_model(model)
    onnx.save(model, str(path))
    log(
        "Nama output ONNX dinormalisasi metadata-only: "
        + "; ".join(f"{old}->{new}" for old, new in rename_pairs)
    )


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


def configure_cuda_ieee_fp32():
    """Matikan TF32 untuk reference FP32 CUDA agar parity lebih konservatif."""
    import torch
    if not torch.cuda.is_available():
        return
    try:
        torch.backends.cuda.matmul.allow_tf32 = False
    except Exception:
        pass
    try:
        torch.backends.cudnn.allow_tf32 = False
    except Exception:
        pass


def save_pt_cpu_reference(model, h, w, prefix):
    """Golden master PT->ONNX: PyTorch CPU FP32, backend sama dengan ORT CPU."""
    import numpy as np
    import torch
    x_cpu = make_parity_input(h, w).to(dtype=torch.float32, device='cpu')
    model_cpu = model.to(device='cpu', dtype=torch.float32).eval()
    with torch.no_grad():
        outputs_cpu = extract_official_outputs(model_cpu(x_cpu), h, w)
    outputs = tuple(t.detach().cpu().float() for t in outputs_cpu)
    x_cpu.numpy().astype(np.float32).tofile(parity_path(prefix, '.input.f32'))
    for name, tensor in zip(OUTPUT_NAMES, outputs):
        tensor.numpy().astype(np.float32).tofile(parity_path(prefix, f'.{name}.f32'))
    metadata = {
        'input_shape': [1, 3, h, w],
        'output_names': OUTPUT_NAMES,
        'output_shapes': {k: list(v) for k, v in expected_shapes(h, w).items()},
        'seed': 20260811,
        'pt_reference_device': 'cpu',
        'pt_reference_dtype': 'float32',
        'purpose': 'PT_CPU_FP32_to_ONNX_CPU_FP32_golden',
    }
    parity_path(prefix, '.meta.json').write_text(json.dumps(metadata, indent=2))
    log('PT->ONNX golden reference: CPU FP32 (backend-matched dengan ORT CPU)')
    return x_cpu, outputs


def save_pt_cuda_runtime_reference(model, h, w, prefix, precision):
    """Reference runtime TensorRT. FP16 mengikuti demo resmi YOLOPv2; FP32 mematikan TF32."""
    import numpy as np
    import torch
    if not torch.cuda.is_available():
        log('CUDA reference tidak tersedia; TensorRT parity akan memakai golden CPU FP32.')
        return False

    device = torch.device('cuda:0')
    x_cpu = make_parity_input(h, w).to(dtype=torch.float32, device='cpu')
    if precision == 'fp16':
        model_ref = model.to(device=device, dtype=torch.float16).eval()
        x_dev = x_cpu.to(device=device, dtype=torch.float16)
        suffix = '.cuda_fp16'
        dtype_name = 'float16'
        log(f'PT TensorRT reference: CUDA FP16 ({torch.cuda.get_device_name(0)}) - mengikuti demo resmi YOLOPv2')
    else:
        configure_cuda_ieee_fp32()
        model_ref = model.to(device=device, dtype=torch.float32).eval()
        x_dev = x_cpu.to(device=device, dtype=torch.float32)
        suffix = '.cuda_fp32'
        dtype_name = 'float32'
        log(f'PT TensorRT reference: CUDA FP32 IEEE/TF32-off ({torch.cuda.get_device_name(0)})')

    with torch.no_grad():
        outputs_dev = extract_official_outputs(model_ref(x_dev), h, w)
    outputs = tuple(t.detach().cpu().float() for t in outputs_dev)
    # TensorRT ONNX input tetap float32. Simpan input asli float32 yang sama.
    x_cpu.numpy().astype(np.float32).tofile(parity_path(prefix, f'{suffix}.input.f32'))
    for name, tensor in zip(OUTPUT_NAMES, outputs):
        tensor.numpy().astype(np.float32).tofile(parity_path(prefix, f'{suffix}.{name}.f32'))
    meta = {
        'input_shape': [1, 3, h, w],
        'output_names': OUTPUT_NAMES,
        'output_shapes': {k: list(v) for k, v in expected_shapes(h, w).items()},
        'seed': 20260811,
        'pt_reference_device': str(device),
        'pt_reference_dtype': dtype_name,
        'precision': precision,
    }
    parity_path(prefix, f'{suffix}.meta.json').write_text(json.dumps(meta, indent=2))
    return True

class ParityMismatch(RuntimeError):
    """Numerical output differs beyond the accepted fidelity limits."""


class ParityToolError(RuntimeError):
    """Parity tool/JSON format failed; rebuilding the engine cannot fix this."""


class ParityContractError(RuntimeError):
    """TensorRT I/O contract differs from the official YOLOPv2 8-output contract."""


def parse_trt_dimensions(raw):
    """Normalize trtexec --exportOutput dimension formats across TensorRT versions.

    TensorRT 10.x may serialize dimensions as e.g. "1x255x48x80" while
    other builds/tools can produce [1, 255, 48, 80], "[1,255,48,80]",
    or small wrapper dictionaries. Reject dynamic/unknown dimensions.
    """
    import re

    if isinstance(raw, dict):
        for key in ("dims", "dimensions", "shape", "d"):
            if key in raw:
                return parse_trt_dimensions(raw[key])
        raise ParityToolError(f"Format dimensions object TensorRT tidak dikenal: {raw!r}")

    if isinstance(raw, (list, tuple)):
        out = []
        for value in raw:
            if isinstance(value, bool):
                raise ParityToolError(f"Dimensi boolean tidak valid: {raw!r}")
            try:
                ivalue = int(value)
            except (TypeError, ValueError) as exc:
                raise ParityToolError(f"Dimensi TensorRT tidak valid: {raw!r}") from exc
            if ivalue < 0:
                raise ParityToolError(f"Dimensi TensorRT masih dinamis/negatif: {raw!r}")
            out.append(ivalue)
        return tuple(out)

    if isinstance(raw, str):
        text = raw.strip()
        if not text:
            raise ParityToolError("String dimensions TensorRT kosong")
        # Accept 1x3x384x640, 1 X 3 X 384 X 640, [1,3,384,640], (1,3,384,640).
        tokens = re.findall(r"-?\d+", text)
        if not tokens:
            raise ParityToolError(f"Tidak dapat membaca dimensions TensorRT: {raw!r}")
        dims = tuple(int(v) for v in tokens)
        if any(v < 0 for v in dims):
            raise ParityToolError(f"Dimensi TensorRT masih dinamis/negatif: {raw!r}")
        return dims

    raise ParityToolError(f"Tipe dimensions TensorRT tidak dikenal: {type(raw).__name__}: {raw!r}")


def compare_arrays(name, expected, actual, precision, is_anchor=False):
    import numpy as np
    expected = np.asarray(expected, dtype=np.float32).reshape(-1)
    actual = np.asarray(actual, dtype=np.float32).reshape(-1)
    if expected.size != actual.size:
        raise ParityContractError(f"Parity {name}: jumlah elemen beda {actual.size} != {expected.size}")
    if not np.isfinite(actual).all():
        raise ParityMismatch(f"Parity {name}: TensorRT mengandung NaN/Inf")
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
        raise ParityMismatch(f"Numerical parity gagal pada output {name}")


def maybe_validate_onnxruntime(onnx_path, x_cpu, pt_cpu_outputs):
    """PT->ONNX parity harus backend-matched: PyTorch CPU FP32 vs ORT CPU FP32."""
    try:
        import numpy as np
        import onnxruntime as ort
    except Exception:
        log('onnxruntime tidak tersedia; PT->ONNX numerical check dilewati. PT->TensorRT parity tetap wajib.')
        return

    available = ort.get_available_providers()
    if 'CPUExecutionProvider' not in available:
        raise RuntimeError(f'ONNX Runtime CPUExecutionProvider tidak tersedia: {available}')
    # Sengaja CPU-only. Jangan bandingkan PyTorch CUDA dengan ORT CPU karena backend floating-point berbeda.
    providers = ['CPUExecutionProvider']
    log('ONNX Runtime export-parity provider: CPUExecutionProvider (sengaja backend-matched dengan PT CPU FP32)')
    session = ort.InferenceSession(str(onnx_path), providers=providers)
    ort_outputs = session.run(OUTPUT_NAMES, {'images': x_cpu.numpy().astype(np.float32)})
    for name, pt_tensor, ort_tensor in zip(OUTPUT_NAMES, pt_cpu_outputs, ort_outputs):
        expected = pt_tensor.detach().cpu().float().numpy()
        compare_arrays(name, expected, ort_tensor, 'fp32', name.startswith('anchor_stride_'))
    log('PT CPU FP32 -> ONNX Runtime CPU FP32 numerical parity: PASS seluruh 8 output.')


def validate_opencv_cpu(model, onnx_path, h, w):
    """Mandatory runtime-matched parity for the OpenCV-DNN CPU backend."""
    try:
        import cv2
        import numpy as np
        import torch
    except Exception as error:
        raise RuntimeError(
            "Validasi CPU memerlukan python3-opencv, numpy, dan torch: " + repr(error)
        ) from error
    x_cpu = make_parity_input(h, w).to(dtype=torch.float32, device='cpu')
    model_cpu = model.to(device='cpu', dtype=torch.float32).eval()
    with torch.no_grad():
        pt_outputs = extract_official_outputs(model_cpu(x_cpu), h, w)
    net = cv2.dnn.readNetFromONNX(str(onnx_path))
    if net.empty():
        raise RuntimeError("OpenCV DNN tidak dapat membaca ONNX hasil ekspor")
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    net.setInput(x_cpu.numpy().astype(np.float32), "images")
    cv_outputs = net.forward(OUTPUT_NAMES)
    if len(cv_outputs) != len(OUTPUT_NAMES):
        raise RuntimeError(f"OpenCV DNN menghasilkan {len(cv_outputs)} output, wajib 8")
    for name, pt_tensor, cv_output in zip(OUTPUT_NAMES, pt_outputs, cv_outputs):
        expected = pt_tensor.detach().cpu().float().numpy()
        compare_arrays(
            name, expected, cv_output, 'fp32', name.startswith('anchor_stride_'))
    log('PT CPU FP32 -> OpenCV-DNN CPU FP32 numerical parity: PASS seluruh 8 output.')

def export_onnx(args):
    import torch
    log(f"Python={sys.executable} PyTorch={torch.__version__}")
    log("ONNX graph export: FP32 static [1,3,384,640]; CUDA dipakai untuk reference parity bila tersedia.")
    log(f"PT={args.pt} ONNX={args.onnx} input=[1,3,{args.height},{args.width}] opset={args.opset}")
    if (args.batch, args.channels, args.height, args.width) != (1, 3, 384, 640):
        raise RuntimeError("YOLOPv2 official geometry wajib [1,3,384,640].")
    torch.set_grad_enabled(False)
    model = load_pytorch_model(
        Path(args.pt), allow_pickle_checkpoint=args.allow_pickle_checkpoint).float().cpu().eval()
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

    normalize_onnx_output_names(Path(args.onnx))
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
            normalize_onnx_output_names(Path(args.onnx))
            validate_onnx_structure(Path(args.onnx), args.height, args.width)
            log("ONNX simplification valid (opsional, bukan default).")
        except Exception as error:
            raise RuntimeError(f"Simplifikasi diminta tetapi gagal: {error}")

    if args.validate_opencv_cpu:
        validate_opencv_cpu(model, Path(args.onnx), args.height, args.width)

    if args.validate:
        # 1) Export graph parity: CPU FP32 <-> CPU FP32, tanpa mencampur backend.
        x_cpu, pt_cpu_outputs = save_pt_cpu_reference(model, args.height, args.width, args.parity_prefix)
        maybe_validate_onnxruntime(Path(args.onnx), x_cpu, pt_cpu_outputs)
        # 2) Runtime references: CUDA FP16 (jalur resmi demo) dan CUDA FP32 TF32-off.
        #    Dipakai kemudian pada PT<->TensorRT parity.
        save_pt_cuda_runtime_reference(model, args.height, args.width, args.parity_prefix, 'fp16')
        # model sudah dipindah/dicast oleh reference FP16; load fresh supaya FP32 tidak mewarisi half.
        model_fp32 = load_pytorch_model(
            Path(args.pt), allow_pickle_checkpoint=args.allow_pickle_checkpoint).float().cpu().eval()
        save_pt_cuda_runtime_reference(model_fp32, args.height, args.width, args.parity_prefix, 'fp32')
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
    # Untuk engine FP16 gunakan reference PT CUDA FP16 (jalur resmi YOLOPv2 demo).
    # Untuk FP32 fallback gunakan PT CUDA FP32 dengan TF32 dimatikan.
    runtime_suffix = '.cuda_fp16' if args.precision == 'fp16' else '.cuda_fp32'
    runtime_meta = parity_path(prefix, f'{runtime_suffix}.meta.json')
    if runtime_meta.is_file():
        ref_suffix = runtime_suffix
        meta_path = runtime_meta
        log(f'TensorRT parity memakai PT runtime reference {runtime_suffix}')
    else:
        ref_suffix = ''
        meta_path = parity_path(prefix, '.meta.json')
        log('TensorRT parity fallback memakai PT CPU FP32 golden reference')
    if not meta_path.is_file():
        raise ParityToolError(f"Metadata parity PT tidak ditemukan: {meta_path}")
    meta = json.loads(meta_path.read_text())
    if meta.get("output_names") != OUTPUT_NAMES:
        raise ParityContractError("Metadata parity output names tidak sesuai kontrak V19.4.")
    records = load_trtexec_json(args.trt_json)
    missing = [name for name in OUTPUT_NAMES if name not in records]
    if missing:
        raise ParityContractError(f"TensorRT exportOutput tidak memiliki output: {missing}")

    for name in OUTPUT_NAMES:
        expected_shape = tuple(meta["output_shapes"][name])
        item = records[name]
        dims_raw = item.get("dimensions", item.get("shape", []))
        dims = parse_trt_dimensions(dims_raw)
        log(f"TensorRT JSON {name}: dimensions={dims_raw!r} -> parsed={dims}")
        if dims != expected_shape:
            raise ParityContractError(f"TensorRT shape {name} salah: {dims} != {expected_shape} (raw={dims_raw!r})")
        values = item.get("values")
        if not isinstance(values, list):
            raise ParityToolError(f"TensorRT JSON {name} tidak memiliki array values.")
        expected = np.fromfile(parity_path(prefix, f"{ref_suffix}.{name}.f32"), dtype=np.float32)
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
    ap.add_argument(
        "--validate-opencv-cpu", action="store_true",
        help="wajibkan parity numerik PT CPU terhadap OpenCV-DNN CPU untuk seluruh 8 output")
    ap.add_argument(
        "--allow-pickle-checkpoint", action="store_true",
        help="opt-in manual untuk checkpoint torch.load tepercaya; runtime robot tidak memakai ini")
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
        try:
            compare_trt(args)
        except ParityMismatch as error:
            log(f"PARITY_NUMERICAL_FAIL: {error}")
            raise SystemExit(42)
        except ParityContractError as error:
            log(f"PARITY_CONTRACT_FAIL: {error}")
            raise SystemExit(44)
        except (ParityToolError, json.JSONDecodeError, KeyError, TypeError, ValueError) as error:
            log(f"PARITY_TOOL_ERROR: {type(error).__name__}: {error}")
            raise SystemExit(43)


if __name__ == "__main__":
    main()
