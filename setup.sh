#!/usr/bin/env bash
# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
# Agentic Robotics — one-command workspace setup.
#
#   ./setup.sh              install apt deps, build, install commands + shell hook
#   ./setup.sh --no-shell   skip the ~/.bashrc line
#   ./setup.sh --no-deps    skip the apt step (offline, or you manage deps yourself)
#   ./setup.sh --deps-only  install the apt deps and stop
#
# Assumes ROS 2 and PX4 are already installed. If they are not, run ./install.sh first.
set -euo pipefail

AGR_SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_HOME="${HOME:-$(getent passwd "$(id -un)" | cut -d: -f6)}"
export AGR_WS="${AGR_WS:-$(cd "${AGR_SRC}/../.." && pwd)}"
export AGR_ROS_DISTRO="${AGR_ROS_DISTRO:-jazzy}"
DO_SHELL=1
DO_DEPS=1
DEPS_ONLY=0
for arg in "$@"; do
  case "$arg" in
    --no-shell)  DO_SHELL=0 ;;
    --no-deps)   DO_DEPS=0 ;;
    --deps-only) DEPS_ONLY=1 ;;
    *) echo "unknown option: $arg" >&2; exit 2 ;;
  esac
done

echo "[setup] workspace: ${AGR_WS}"
echo "[setup] sources:   ${AGR_SRC}"

if [[ ! -d "/opt/ros/${AGR_ROS_DISTRO}" ]]; then
  echo "✗ ROS 2 ${AGR_ROS_DISTRO} not found at /opt/ros/${AGR_ROS_DISTRO}." >&2
  exit 1
fi

# px4_msgs is a hard dependency of agr_uav_tools and is NOT vendored here — it
# is upstream code that must match the PX4 build, so it is cloned and pinned
# rather than copied. Say which, clearly, instead of failing inside colcon.
if [[ ! -d "${AGR_WS}/src/px4_msgs" ]]; then
  cat >&2 <<EOF
✗ px4_msgs is missing from ${AGR_WS}/src.
  It must match your PX4 build commit exactly — a mismatch does not error, it
  silently stops publishing some uORB topics.
      git clone https://github.com/PX4/px4_msgs.git ${AGR_WS}/src/px4_msgs
      git -C ${AGR_WS}/src/px4_msgs checkout <commit matching your PX4>
EOF
  exit 1
fi

# ── apt dependencies ────────────────────────────────────────────────────────
# Robot descriptions and simulators that the platforms USE but do not vendor.
# Nothing here is copied into the repo, and that is deliberate: these are
# upstream packages maintained by the people who make the robots, and a
# vendored copy is a fork that silently rots. The cost is that a fresh machine
# needs this step before colcon will build.
#
# Which platform needs what:
#   ground  — Husky (clearpath), UR5e, Robotiq, RealSense
#   arm     — the same UR5e + Robotiq + RealSense, bolted to a bench
#   legged  — ros2_control and gz_ros2_control for the 12-joint gait
#   tb3/tb4 — the OFFICIAL TurtleBot simulators, launched as-is. TurtleBot 4
#             also pulls the whole iRobot Create 3 stack (about 70 packages),
#             because a TB4 is a Create 3 with a mast on it.
APT_DEPS=(
  # shared / ground + arm
  "ros-${AGR_ROS_DISTRO}-ur-description"
  "ros-${AGR_ROS_DISTRO}-robotiq-description"
  "ros-${AGR_ROS_DISTRO}-realsense2-description"
  "ros-${AGR_ROS_DISTRO}-clearpath-platform-description"
  # legged
  "ros-${AGR_ROS_DISTRO}-ros2-control"
  "ros-${AGR_ROS_DISTRO}-ros2-controllers"
  "ros-${AGR_ROS_DISTRO}-gz-ros2-control"
  # gazebo bridge, used by every platform
  "ros-${AGR_ROS_DISTRO}-ros-gz-sim"
  "ros-${AGR_ROS_DISTRO}-ros-gz-bridge"
  "ros-${AGR_ROS_DISTRO}-ros-gz-image"
  # TurtleBot 3 — official simulator
  "ros-${AGR_ROS_DISTRO}-turtlebot3"
  "ros-${AGR_ROS_DISTRO}-turtlebot3-simulations"
  "ros-${AGR_ROS_DISTRO}-turtlebot3-teleop"
  # TurtleBot 4 — official simulator (pulls irobot_create_*)
  "ros-${AGR_ROS_DISTRO}-turtlebot4-simulator"
  "ros-${AGR_ROS_DISTRO}-turtlebot4-description"
  "ros-${AGR_ROS_DISTRO}-turtlebot4-navigation"
  "ros-${AGR_ROS_DISTRO}-turtlebot4-viz"
)

install_deps() {
  local missing=()
  for pkg in "${APT_DEPS[@]}"; do
    dpkg -s "$pkg" >/dev/null 2>&1 || missing+=("$pkg")
  done
  if (( ${#missing[@]} == 0 )); then
    echo "[setup] all ${#APT_DEPS[@]} apt dependencies already installed"
    return 0
  fi
  echo "[setup] installing ${#missing[@]} missing apt package(s):"
  printf '           %s\n' "${missing[@]}"
  # --no-install-recommends keeps this to the simulation packages rather than
  # dragging in every desktop tool they suggest.
  sudo apt-get install -y --no-install-recommends "${missing[@]}"
}

if (( DO_DEPS == 1 )); then
  install_deps
else
  echo "[setup] skipping apt dependencies (--no-deps)"
fi
(( DEPS_ONLY == 1 )) && { echo "[setup] --deps-only: stopping here."; exit 0; }

echo "[setup] building…"
"${AGR_SRC}/agr-build"

for cmd in agr-sim agr-stop agr-build; do
  chmod +x "${AGR_SRC}/${cmd}"
  sudo ln -sf "${AGR_SRC}/${cmd}" "/usr/local/bin/${cmd}"
  echo "[setup] installed /usr/local/bin/${cmd}"
done

if (( DO_SHELL == 1 )); then
  printf -v LINE 'source %q' "${AGR_SRC}/agr_aliases.sh"
  if ! grep -qF "${LINE}" "${RUN_HOME}/.bashrc" 2>/dev/null; then
    { echo ""; echo "# Agentic Robotics"; echo "${LINE}"; } >> "${RUN_HOME}/.bashrc"
    echo "[setup] added alias block to ~/.bashrc"
  else
    echo "[setup] ~/.bashrc already sources agr_aliases.sh"
  fi
fi

cat <<EOF

✓ Setup complete.

Open a NEW shell, then:
    agr-sim                       one x500 with the Gazebo GUI
    agr-sim headless:=true        no GUI
    agr-sim airframe:=rc_cessna   fixed-wing
    agr-sim --multi count:=3      three vehicles
    agr-sim ground tools:=true    RaiseBot in the greenhouse
    agr-sim legged                Unitree Go2
    agr-sim arm tools:=true       bench UR5e + Robotiq
    agr-sim turtlebot             TurtleBot 3 (official simulator)
    agr-sim turtlebot model:=tb4_standard    TurtleBot 4 (official simulator)
    agr-stop                      stop everything
    agr_help                      all commands

Note: agr-sim and agr-build strip anaconda from the environment internally,
because ROS 2 ${AGR_ROS_DISTRO} needs the system python3.12 and anaconda's
python3.11 cannot see ROS's modules. Your interactive shell is left alone.
EOF
