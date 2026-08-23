#!/usr/bin/env bash
# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
# Agentic Robotics — one-command workspace setup.
#
#   ./setup.sh            build + install agr-sim/agr-stop/agr-build + shell hook
#   ./setup.sh --no-shell skip the ~/.bashrc line
#
# Assumes ROS 2 and PX4 are already installed. If they are not, run the full
# environment installer first (see README).
set -euo pipefail

AGR_SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_HOME="${HOME:-$(getent passwd "$(id -un)" | cut -d: -f6)}"
AGR_WS="${AGR_WS:-$(cd "${AGR_SRC}/../.." && pwd)}"
AGR_ROS_DISTRO="${AGR_ROS_DISTRO:-jazzy}"
DO_SHELL=1
[[ "${1:-}" == "--no-shell" ]] && DO_SHELL=0

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

echo "[setup] building…"
"${AGR_SRC}/agr-build"

for cmd in agr-sim agr-stop agr-build; do
  chmod +x "${AGR_SRC}/${cmd}"
  sudo ln -sf "${AGR_SRC}/${cmd}" "/usr/local/bin/${cmd}"
  echo "[setup] installed /usr/local/bin/${cmd}"
done

if (( DO_SHELL == 1 )); then
  LINE="source ${AGR_SRC}/agr_aliases.sh"
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
    agr-stop                      stop everything
    agr_help                      all commands

Note: agr-sim and agr-build strip anaconda from the environment internally,
because ROS 2 ${AGR_ROS_DISTRO} needs the system python3.12 and anaconda's
python3.11 cannot see ROS's modules. Your interactive shell is left alone.
EOF
