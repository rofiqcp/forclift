#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export AGV_ROOT="${AGV_ROOT:-$ROOT}"
BUILD_PERCENT=50
MEMORY_PERCENT=50
DO_BUILD=1
WITH_MODEL=0
WITH_BROWSER_QA=0

usage() {
  cat <<'EOF'
Usage: ./install.sh [options]
  --no-build           install dependency + environment saja
  --with-model         download/verifikasi models/yolopv2.pt resmi
  --with-browser-qa    install Playwright Chromium untuk QA ROS Web
  --build-percent N    batas CPU build, 10..100 (default 50)
  --memory-percent N   batas RAM build, 10..90 (default 50)
  --help               tampilkan bantuan
EOF
}

while (($#)); do
  case "$1" in
    --no-build) DO_BUILD=0 ;;
    --with-model) WITH_MODEL=1 ;;
    --with-browser-qa) WITH_BROWSER_QA=1 ;;
    --build-percent) shift; BUILD_PERCENT="${1:-}" ;;
    --memory-percent) shift; MEMORY_PERCENT="${1:-}" ;;
    --help|-h) usage; exit 0 ;;
    *) echo "[ERROR] opsi tidak dikenal: $1" >&2; usage; exit 2 ;;
  esac
  shift
done

if ! [[ "$BUILD_PERCENT" =~ ^[0-9]+$ ]] || ((BUILD_PERCENT < 10 || BUILD_PERCENT > 100)); then
  echo "[ERROR] --build-percent wajib 10..100" >&2
  exit 2
fi
if ! [[ "$MEMORY_PERCENT" =~ ^[0-9]+$ ]] || ((MEMORY_PERCENT < 10 || MEMORY_PERCENT > 90)); then
  echo "[ERROR] --memory-percent wajib 10..90" >&2
  exit 2
fi

log() { printf '\n[AGV] %s\n' "$*"; }
need_sudo() {
  if [[ $EUID -eq 0 ]]; then echo ""; return; fi
  command -v sudo >/dev/null || { echo "[ERROR] sudo tidak tersedia" >&2; exit 2; }
  sudo -v
}

if [[ "$(basename "$ROOT")" != "forclift" ]]; then
  echo "[WARN] nama folder project bukan 'forclift': $ROOT" >&2
fi

need_sudo
SUDO=""
[[ $EUID -ne 0 ]] && SUDO="sudo"

log "Menyiapkan repository/submodule"
cd "$ROOT"
git submodule update --init --recursive

log "Menyiapkan repository apt dasar"
$SUDO apt-get update
$SUDO apt-get install -y --no-install-recommends \
  ca-certificates curl wget gnupg lsb-release software-properties-common \
  build-essential cmake git pkg-config unzip rsync jq

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  log "ROS 2 Humble belum ada; menambahkan repository resmi ROS"
  $SUDO add-apt-repository -y universe
  ROS_KEY=/usr/share/keyrings/ros-archive-keyring.gpg
  curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
    $SUDO tee "$ROS_KEY" >/dev/null
  . /etc/os-release
  CODENAME="${UBUNTU_CODENAME:-${VERSION_CODENAME:-jammy}}"
  ARCH="$(dpkg --print-architecture)"
  echo "deb [arch=$ARCH signed-by=$ROS_KEY] http://packages.ros.org/ros2/ubuntu $CODENAME main" | \
    $SUDO tee /etc/apt/sources.list.d/ros2.list >/dev/null
  $SUDO apt-get update
  $SUDO apt-get install -y ros-humble-desktop ros-dev-tools
fi

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

log "Menginstal dependency sistem AGV"
$SUDO apt-get install -y --no-install-recommends \
  python3-pip python3-venv python3-yaml python3-numpy python3-opencv \
  python3-pil python3-pyproj python3-openpyxl python3-matplotlib \
  python3-rasterio python3-shapely python3-requests python3-serial \
  libopencv-dev libyaml-cpp-dev qtbase5-dev dfu-util libusb-1.0-0-dev

$SUDO apt-get install -y \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-robot-localization \
  ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui \
  ros-humble-vision-msgs

log "Menyiapkan rosdep"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  $SUDO rosdep init
fi
rosdep update
rosdep install --from-paths "$ROOT/src" --ignore-src -r -y --rosdistro humble

log "Memastikan PyTorch CPU kompatibel C++11 ABI"
if ! /usr/bin/python3 - <<'PY'
import torch
fn=getattr(torch, "compiled_with_cxx11_abi", None)
assert callable(fn) and fn(), "PyTorch CXX11_ABI harus 1"
print("torch", torch.__version__, "CXX11_ABI=1")
PY
then
  /usr/bin/python3 -m pip install --user --upgrade \
    torch torchvision --index-url https://download.pytorch.org/whl/cpu
  /usr/bin/python3 - <<'PY'
import torch
assert torch.compiled_with_cxx11_abi(), "PyTorch hasil install belum CXX11_ABI=1"
print("torch", torch.__version__, "CXX11_ABI=1")
PY
fi

log "Memastikan PlatformIO tersedia"
if ! command -v pio >/dev/null 2>&1; then
  /usr/bin/python3 -m pip install --user --upgrade platformio
fi

if getent group dialout >/dev/null; then
  $SUDO usermod -aG dialout "${SUDO_USER:-$USER}" || true
fi

log "Menyiapkan folder runtime"
mkdir -p "$ROOT/models" "$ROOT/data" "$ROOT/calibration" "$ROOT/log"
chmod +x "$ROOT/scripts/agv_env.sh" "$ROOT/scripts/check_portable_paths.py" \
  "$ROOT/models/model.sh" 2>/dev/null || true

log "Menulis managed block AGV ke ~/.bashrc"
/usr/bin/python3 - "$HOME/.bashrc" <<'PY'
from pathlib import Path
import sys
path=Path(sys.argv[1]).expanduser()
start="# >>> FORCLIFT WORKSPACE >>>"
end="# <<< FORCLIFT WORKSPACE <<<"
text=path.read_text() if path.exists() else ""
while start in text and end in text:
    a=text.index(start); b=text.index(end,a)+len(end)
    text=text[:a].rstrip()+"\n"+text[b:].lstrip()
block='''# >>> FORCLIFT WORKSPACE >>>
export AGV_ROOT="${AGV_ROOT:-$HOME/forclift}"
if [ -f "$AGV_ROOT/scripts/agv_env.sh" ]; then
  . "$AGV_ROOT/scripts/agv_env.sh"
fi
# <<< FORCLIFT WORKSPACE <<<'''
path.write_text(text.rstrip()+"\n\n"+block+"\n")
PY

# shellcheck disable=SC1091
source "$ROOT/scripts/agv_env.sh"

if ((WITH_MODEL)); then
  log "Mengunduh/verifikasi YOLOPv2 resmi"
  if [[ -x "$ROOT/models/model.sh" ]]; then
    "$ROOT/models/model.sh"
  else
    echo "[WARN] models/model.sh tidak tersedia; gunakan model yang sudah ada di repository." >&2
  fi
fi

if ((WITH_BROWSER_QA)); then
  log "Menginstal dependency browser QA"
  $SUDO apt-get install -y nodejs npm
  mkdir -p "$ROOT/.playwright"
  npm --prefix "$ROOT/.playwright" install --save-dev @playwright/test
  (cd "$ROOT/.playwright" && npx playwright install chromium)
fi

log "Menjalankan portability/source preflight"
/usr/bin/python3 "$ROOT/scripts/check_portable_paths.py"
for check in "$ROOT/src/navigation/test/project_consistency_self_check.py" "$ROOT/src/navigation/test/web_gui_self_check.py"; do
  [[ -f "$check" ]] && /usr/bin/python3 "$check"
done

if ((DO_BUILD)); then
  cores="$(nproc)"
  allowed_cores=$(( cores * BUILD_PERCENT / 100 ))
  ((allowed_cores < 1)) && allowed_cores=1
  ((allowed_cores > cores)) && allowed_cores="$cores"
  cpu_last=$((allowed_cores - 1))
  cpu_quota=$((allowed_cores * 100))
  mem_kb="$(awk '/MemTotal/{print $2}' /proc/meminfo)"
  mem_bytes=$(( mem_kb * 1024 * MEMORY_PERCENT / 100 ))
  # Translation unit GUI/LibTorch cukup berat; j1 menjaga peak RSS stabil.
  export CMAKE_BUILD_PARALLEL_LEVEL=1
  export MAKEFLAGS="-j1"
  log "Build ROS: CPU <=${BUILD_PERCENT}% (core 0-$cpu_last), RAM <=${MEMORY_PERCENT}%, Release, j1"
  build_cmd=(taskset -c "0-$cpu_last" colcon build --base-paths "$ROOT/src"
    --symlink-install --executor sequential
    --cmake-args -DCMAKE_BUILD_TYPE=Release)
  export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
  export DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-unix:path=$XDG_RUNTIME_DIR/bus}"
  if command -v systemd-run >/dev/null 2>&1 && systemctl --user is-system-running >/dev/null 2>&1; then
    systemd-run --user --scope --quiet \
      -p "CPUQuota=${cpu_quota}%" \
      -p "MemoryMax=$mem_bytes" \
      -p "MemorySwapMax=1200M" \
      "${build_cmd[@]}"
  else
    "${build_cmd[@]}"
  fi
  # shellcheck disable=SC1091
  source "$ROOT/install/setup.bash"
fi

log "Setup selesai"
echo "AGV_ROOT=$AGV_ROOT"
echo "Buka shell baru atau jalankan: source ~/.bashrc"
echo "ROS Web: ros2 launch navigation autonomous.launch.py mode:=web"
