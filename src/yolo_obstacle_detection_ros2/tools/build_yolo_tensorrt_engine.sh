#!/usr/bin/env bash
set -euo pipefail

# Build and benchmark an FP16 TensorRT engine on the TARGET Jetson.
# Serialized TensorRT engines are deliberately not shipped/cross-copied because
# compatibility depends on the target TensorRT/CUDA/GPU environment.
AGV_WS="${AGV_WS:-/home/otomasi2/forclift}"
ONNX="${1:-${AGV_WS}/models/yolov8n_agv_forklift.onnx}"
ENGINE="${2:-${ONNX%.onnx}.engine}"

if [[ ! -f "$ONNX" ]]; then
  echo "ERROR: ONNX model not found: $ONNX" >&2
  exit 2
fi
TRTEXEC="$(command -v trtexec || true)"
if [[ -z "$TRTEXEC" && -x /usr/src/tensorrt/bin/trtexec ]]; then
  TRTEXEC=/usr/src/tensorrt/bin/trtexec
fi
if [[ -z "$TRTEXEC" ]]; then
  echo "ERROR: trtexec not found. Install/use the TensorRT tools from the active JetPack image." >&2
  exit 3
fi

mkdir -p "$(dirname "$ENGINE")"
HELP="$($TRTEXEC --help 2>&1 || true)"
WORKSPACE_ARG="--workspace=2048"
if grep -q -- '--memPoolSize' <<<"$HELP"; then
  WORKSPACE_ARG="--memPoolSize=workspace:2048"
fi

echo "TensorRT tool : $TRTEXEC"
echo "ONNX          : $ONNX"
echo "Engine        : $ENGINE"
echo "Building FP16 engine on this Jetson..."
"$TRTEXEC" --onnx="$ONNX" --saveEngine="$ENGINE" --fp16 "$WORKSPACE_ARG"

echo
echo "Benchmarking serialized engine..."
"$TRTEXEC" --loadEngine="$ENGINE" --warmUp=1000 --duration=5

sha256sum "$ONNX" "$ENGINE" | tee "${ENGINE}.sha256"
{
  echo "built_at=$(date --iso-8601=seconds)"
  echo "hostname=$(hostname)"
  echo "uname=$(uname -a)"
  echo "trtexec=$TRTEXEC"
  "$TRTEXEC" --version 2>&1 | head -5 || true
} > "${ENGINE}.buildinfo.txt"

echo "PASS: engine built and benchmarked on the target Jetson."
