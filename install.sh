#!/usr/bin/env bash
# Agentic Robotics: full native installation, based on the RAISE student setup.
set -euo pipefail
AGR_SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export AGR_WS="${AGR_WS:-$(cd "${AGR_SRC}/../.." && pwd)}"
export AGR_ROS_DISTRO=jazzy
NO_SHELL=0
CHECK_ONLY=0
for arg in "$@"; do
  case "$arg" in
    --no-shell) NO_SHELL=1 ;;
    --check) CHECK_ONLY=1 ;;
    -h|--help)
      cat <<'HELP'
Usage: ./install.sh [--no-shell] [--check]
Install ROS 2 Jazzy, Gazebo Harmonic, all five robot platforms, CPU PyTorch,
YOLO/OpenAI, LeRobot/SmolVLA, PX4 SITL and Micro XRCE-DDS Agent; build and check.
Requires Ubuntu 24.04 amd64, internet and sudo. Run as your normal user.
--no-shell  Do not add the workspace hook to ~/.bashrc.
--check     Check the existing installation without installing anything.
Existing PX4/message checkouts must match the pinned revisions; never reset.
HELP
      exit 0 ;;
    *) echo "Unknown option: $arg" >&2; exit 2 ;;
  esac
done
if (( CHECK_ONLY )); then exec bash "${AGR_SRC}/scripts/check_install.sh"; fi
[[ $EUID -ne 0 ]] || { echo 'Run as a normal user, without sudo.' >&2; exit 1; }
# shellcheck disable=SC1091
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 24.04 && "$(uname -m)" == x86_64 ]] || {
  echo 'Supported platform: Ubuntu 24.04 amd64 + ROS 2 Jazzy.' >&2; exit 1;
}
[[ "${AGR_SRC}" == "${AGR_WS}/src/agentic_robotics" ]] || {
  echo 'Clone this repository at <workspace>/src/agentic_robotics; set AGR_WS if needed.' >&2; exit 1;
}
# A clean subprocess environment avoids picking up Conda or another ROS distro.
if [[ "${AGR_INSTALL_CLEAN:-}" != 1 ]]; then
  exec env -u PYTHONPATH -u LD_LIBRARY_PATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    -u COLCON_PREFIX_PATH -u ROS_DISTRO -u VIRTUAL_ENV -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    AGR_INSTALL_CLEAN=1 bash "$0" "$@"
fi
LOG_DIR="${AGR_WS}/install_logs"
mkdir -p "$LOG_DIR"
exec > >(tee -a "${LOG_DIR}/install-$(date +%Y%m%d-%H%M%S).log") 2>&1
SUDO_KEEPALIVE_PID=''
cleanup() {
  [[ -z "$SUDO_KEEPALIVE_PID" ]] || kill "$SUDO_KEEPALIVE_PID" 2>/dev/null || true
}
trap cleanup EXIT
trap 'echo "Installation failed at line $LINENO. Fix the error above and rerun ./install.sh." >&2' ERR
export DEBIAN_FRONTEND=noninteractive
PX4_DIR="${PX4_DIR:-${HOME}/PX4-Autopilot}"
PX4_REF=03bf4a5e95074c08409297d3b3683a4aa0e19208
MSGS_REF=a4a9864b3c40f0b7e5176e6543872a251b2e3c70
AGENT_DIR="${HOME}/Micro-XRCE-DDS-Agent"
# Check existing sources BEFORE making system changes.
check_checkout() {
  local dest=$1 ref=$2
  if [[ -e "$dest" ]]; then
    if [[ ! -d "$dest/.git" ]] ||
       [[ "$(git -C "$dest" rev-parse HEAD)" != "$(git -C "$dest" rev-parse "$ref^{commit}")" ]] ||
       [[ -n "$(git -C "$dest" status --porcelain --untracked-files=no)" ]]; then
      echo "Existing checkout $dest must be clean and at $ref. It has not been changed." >&2
      return 1
    fi
  fi
}
check_checkout "$PX4_DIR" "$PX4_REF"
check_checkout "${AGR_WS}/src/px4_msgs" "$MSGS_REF"
check_checkout "$AGENT_DIR" v3.0.0
sudo -v
# Keep the single sudo authentication alive during the long ML and PX4 builds.
(while sleep 50; do sudo -n true || exit; done) &
SUDO_KEEPALIVE_PID=$!
sudo apt-get update
sudo apt-get install -y software-properties-common curl ca-certificates locales git python3
sudo locale-gen en_US.UTF-8
export LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
sudo add-apt-repository -y universe
if ! dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null | grep -q 'install ok installed'; then
  # Official ROS apt-source package owns repository keys and their updates.
  ROS_APT_VERSION="$(curl -fsSL --retry 3 --max-time 60 https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | /usr/bin/python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')"
  ROS_APT_DEB="$(mktemp --suffix=.deb)"
  curl -fL --retry 3 --max-time 120 "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_VERSION}/ros2-apt-source_${ROS_APT_VERSION}.noble_all.deb" -o "$ROS_APT_DEB"
  sudo dpkg -i "$ROS_APT_DEB"
  rm -f "$ROS_APT_DEB"
fi
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  ros-jazzy-desktop ros-dev-tools ros-jazzy-ros-gz \
  ros-jazzy-nav2-bringup ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-cv-bridge python3-opencv python3-flask python3-requests \
  python3-colcon-common-extensions python3-rosdep python3-venv python3-pip \
  build-essential astyle bc cmake cppcheck file gdb lcov ninja-build ccache \
  libssl-dev libxml2-dev libxml2-utils libunwind-dev libeigen3-dev \
  libgstreamer-plugins-base1.0-dev libimage-exiftool-perl libopencv-dev \
  cppzmq-dev pkg-config protobuf-compiler python3-dev python3-setuptools \
  python3-wheel rsync shellcheck unzip zip ffmpeg mesa-utils \
  gstreamer1.0-plugins-bad gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly gstreamer1.0-libav
clone_pinned() {
  local url=$1 dest=$2 ref=$3
  if [[ ! -d "$dest" ]]; then
    git clone --no-checkout "$url" "$dest"
    git -C "$dest" checkout --detach "$ref"
  fi
}
clone_pinned https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR" "$PX4_REF"
clone_pinned https://github.com/PX4/px4_msgs.git "${AGR_WS}/src/px4_msgs" "$MSGS_REF"
clone_pinned https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$AGENT_DIR" v3.0.0
git -C "$PX4_DIR" submodule update --init --recursive
# User-site libraries are visible to ROS entrypoints using /usr/bin/python3.
# Keep NumPy 1 and compatible OpenCV together for the apt cv_bridge ABI.
PIP=(/usr/bin/python3 -m pip install --user --break-system-packages --retries 5 --timeout 120)
"${PIP[@]}" 'torch==2.10.0' 'torchvision==0.25.0' --index-url https://download.pytorch.org/whl/cpu
"${PIP[@]}" -r "${AGR_SRC}/scripts/requirements-ai.txt"
# PX4 requires empy 3; isolate it from Jazzy's empy 4 build tooling.
PX4_VENV="${HOME}/raise_venvs/px4"
/usr/bin/python3 -m venv "$PX4_VENV"
"${PX4_VENV}/bin/python3" -m pip install --retries 5 --timeout 120 -r "${PX4_DIR}/Tools/setup/requirements.txt"
VENV="${HOME}/raise_venvs/lerobot"
/usr/bin/python3 -m venv --system-site-packages "$VENV"
"${VENV}/bin/python3" -m pip install --upgrade pip
"${VENV}/bin/python3" -m pip install --retries 5 --timeout 120 \
  'torch==2.10.0' 'torchvision==0.25.0' 'torchcodec==0.10.0' \
  --index-url https://download.pytorch.org/whl/cpu
"${VENV}/bin/python3" -m pip install --retries 5 --timeout 120 'lerobot[smolvla]==0.5.1' fastapi uvicorn
# Cache the VLM backbone used by the Day-2 exercises. If Hugging Face is
# temporarily unavailable, the first model run will retry the same download.
BACKBONE_DIR="${HOME}/.cache/huggingface/hub/models--HuggingFaceTB--SmolVLM2-500M-Video-Instruct"
if [[ ! -d "$BACKBONE_DIR" ]]; then
  "${VENV}/bin/python3" - <<'PY' || echo 'WARNING: SmolVLM2 pre-download failed; first model use will retry.' >&2
from transformers import AutoModelForImageTextToText, AutoProcessor
model = "HuggingFaceTB/SmolVLM2-500M-Video-Instruct"
AutoProcessor.from_pretrained(model)
AutoModelForImageTextToText.from_pretrained(model)
print("SmolVLM2 backbone cached for offline use")
PY
fi
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u
cmake -S "$AGENT_DIR" -B "${AGENT_DIR}/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "${AGENT_DIR}/build" --parallel 2
sudo cmake --install "${AGENT_DIR}/build"
sudo ldconfig
(cd "$PX4_DIR" && PATH="${PX4_VENV}/bin:${PATH}" make -j2 px4_sitl_default)
[[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]] || sudo rosdep init
rosdep update --rosdistro jazzy
rosdep install --from-paths "$AGR_SRC" "${AGR_WS}/src/px4_msgs" --ignore-src --rosdistro jazzy -y
bash "${AGR_SRC}/legged/agr_legged_description/fetch_meshes.sh"
SETUP_ARGS=()
(( NO_SHELL )) && SETUP_ARGS+=(--no-shell)
# The PX4 and ML builds can outlast sudo's credential timeout.
sudo -v
bash "${AGR_SRC}/setup.sh" "${SETUP_ARGS[@]}"
export PX4_DIR
bash "${AGR_SRC}/scripts/check_install.sh"
echo "Full installation complete. Open a new terminal and run agr-sim or agr-sim ground tools:=true."
