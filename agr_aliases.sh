# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
# Agentic Robotics — shell integration. Sourced from ~/.bashrc by setup.sh:
#
#     source ~/ros2_ws/src/agentic_robotics/agr_aliases.sh
#
# Namespaced `agr_*` so it never collides with the AgriBot alias block.

_AGR_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
export AGR_SRC="${_AGR_DIR}"
export AGR_WS="${AGR_WS:-$(cd "${_AGR_DIR}/../.." && pwd)}"
export AGR_ROS_DISTRO="${AGR_ROS_DISTRO:-jazzy}"

# ── anaconda warning (once, at shell start) ─────────────────────────────────
# We do NOT strip anaconda from your interactive shell — you use it for other
# work. agr-sim / agr-build strip it internally instead. But if you run a bare
# `colcon build` yourself it WILL fail with "No module named 'em'", so say so
# rather than let it look like a broken workspace.
if command -v python3 >/dev/null 2>&1 && python3 -c 'import sys; sys.exit(0 if "conda" in sys.prefix else 1)' 2>/dev/null; then
  echo "ℹ  anaconda python is active ($(python3 -V 2>&1)). ROS 2 needs system python3.12."
  echo "   Use 'agr-build' instead of 'colcon build' — it handles this for you."
fi

# Overlay the workspace if built. ROS's setup.bash trips `set -u`, so preserve
# the caller's nounset state exactly rather than forcing it on or off.
if [ -f "${AGR_WS}/install/setup.bash" ]; then
  case "$-" in *u*) _agr_u=1 ;; *) _agr_u=0 ;; esac
  set +u
  . "${AGR_WS}/install/setup.bash"
  [ "${_agr_u}" = "1" ] && set -u
  unset _agr_u
fi

# ── build / run ─────────────────────────────────────────────────────────────
# agr-sim, agr-stop, agr-build are real scripts on /usr/local/bin — no aliases.
alias agr_ws='cd "${AGR_WS}"'
alias agr_src='cd "${AGR_SRC}"'

# ── inspect ─────────────────────────────────────────────────────────────────
alias agr_airframes='ros2 run agr_uav_tools list_airframes'
alias agr_monitor='ros2 run agr_uav_tools vehicle_monitor'
alias agr_check='ros2 run agr_uav_labs 01_check_bridge'
alias agr_topics='ros2 topic list | grep -E "^/(px4_[0-9]+/)?fmu" || echo "no /fmu topics — start agr-sim first"'

# PX4 version-suffixes some topics (vehicle_status_v4) and not others
# (vehicle_global_position), and the suffix moves between releases. Resolve by
# prefix at call time so nothing here rots.
_agr_topic() {
  ros2 topic list 2>/dev/null | grep -E "^/(${2:+${2}/})?fmu/out/$1(_v[0-9]+)?$" | head -1
}
agr_echo() {
  local base="${1:?usage: agr_echo <topic-basename> [namespace]}" ns="${2:-}"
  local t; t="$(_agr_topic "$base" "$ns")"
  [ -n "$t" ] || { echo "✗ no /fmu/out/${base} — is agr-sim running?"; return 1; }
  # Timeout, because a topic can be advertised and still never publish:
  # vehicle_local_position only starts once the EKF has a valid estimate.
  timeout "${AGR_ECHO_TIMEOUT:-10}" ros2 topic echo "$t" --once ||
    { echo "✗ ${t} advertised but silent — estimator likely has no fix yet."; return 1; }
}
alias agr_status='agr_echo vehicle_status'
alias agr_pos='agr_echo vehicle_local_position'
alias agr_batt='agr_echo battery_status'

# ── ground platform (AgriBot stack, unmodified) ──────────────────────────
alias agr_ground='agr-sim ground'
alias agr_ground_lite='agr-sim ground world:=greenhouse_2026_lite.sdf'
alias agr_drive='ros2 run agribot_teleop teleop_keyboard'
alias agr_cam='ros2 run agribot_teleop camera_view'
alias agr_ground_topics='ros2 topic list | grep -E "cmd_vel|scan|camera|joint_states|odom"'

# ── Day-2 VLA labs ──────────────────────────────────────────────────────────
# These import LeRobot, which pulls numpy 2.x — incompatible with the apt
# cv_bridge the rest of the stack uses. So they run under their own venv rather
# than the system python. The venv is created with system-site-packages, so it
# still sees rclpy and the agribot_* packages.
export AGR_LEROBOT_PY="${AGR_LEROBOT_PY:-$HOME/raise_venvs/lerobot/bin/python3}"
export AGR_D2="${AGR_SRC}/ground/agribot_labs/day2"

# The reference policy is PUBLIC on Hugging Face — no token, no manual download.
# api_clients/vla_client/factory.py resolves in this order:
#   VLA_LOCAL_CKPT  →  ~/raise_checkpoints/smolvla_C_ref  →  this HF repo
# so a student with none of the above still gets a working brain on first run
# (~900 MB into the HF cache, then local forever).
export AGR_VLA_HF_REF="scalexi/smolvla-raise2026-ripeness-ref"

_agr_vla() {
  if [ ! -x "${AGR_LEROBOT_PY}" ]; then
    echo "✗ LeRobot venv not found at ${AGR_LEROBOT_PY}"
    echo "  Set AGR_LEROBOT_PY, or create it — see ground/agribot_labs/day2/HOW_TO_TRAIN_AND_USE.md"
    return 1
  fi
  "${AGR_LEROBOT_PY}" "$@"
}
alias agr_vla_one='_agr_vla "${AGR_D2}/day2_02_vla_executor/starter/vla_one_step.py"'
alias agr_vla='_agr_vla "${AGR_D2}/day2_02_vla_executor/starter/vla_executor.py"'
alias agr_finetune='_agr_vla "${AGR_D2}/day2_02_vla_executor/starter/finetune_smolvla.py"'
alias agr_record='_agr_vla "${AGR_D2}/day2_01_teleoperation_and_data/starter/03_record.py"'
alias agr_replay='_agr_vla "${AGR_D2}/day2_01_teleoperation_and_data/starter/06_replay_episode.py"'
# Force the public HF brain even when a local checkpoint exists — this is the
# path a student on a fresh machine takes, so it is the one worth testing.
alias agr_vla_hf='VLA_LOCAL_CKPT="${AGR_VLA_HF_REF}" _agr_vla "${AGR_D2}/day2_02_vla_executor/starter/vla_one_step.py"'

agr_help() {
  cat <<'HELP'
Agentic Robotics — UAV track

  UAV        agr-sim                       one x500, Gazebo GUI
             agr-sim headless:=true        no GUI
             agr-sim world:=agr_city       urban world
             agr-sim world:=agr_defense    secured installation
             agr-sim airframe:=rc_cessna   fixed-wing (cannot hover)
             agr-sim uav --multi count:=3  three vehicles

  GROUND     agr-sim ground                Husky in the greenhouse
             agr_ground_lite               lighter world (CPU-only machines)
             agr_drive                     keyboard teleop
             agr_cam                       camera view

  stop       agr-stop                      SIGINT then SIGKILL, either stack

  build      agr-build                     colcon with the correct python
             agr-build --packages-select agr_uav_tools

  inspect    agr_airframes                 the airframe registry
             agr_check                     lab 01: is the bridge alive?
             agr_topics                    /fmu topics
             agr_status / agr_pos / agr_batt
             agr_echo <name> [namespace]   any /fmu/out topic

  VLA (day2) agr_vla_one                    one inference step
             agr_vla                        full executor
             agr_vla_hf                     force the public HF brain
             agr_record / agr_replay        teleop data collection
             agr_finetune                   fine-tune SmolVLA
             (these run under the LeRobot venv, not system python)

  navigate   agr_ws / agr_src
HELP
}
