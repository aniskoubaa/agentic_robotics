# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
# Agentic Robotics — shell integration. Sourced from ~/.bashrc by setup.sh:
#
#     source ~/ros2_ws/src/agentic_robotics/agr_aliases.sh
#
# Namespaced `agr_*` so it never collides with the RaiseBot alias block.

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
alias agr_check='ros2 run agr_uav_demos diagnose'
alias agr_check_bridge='ros2 run agr_uav_labs 01_check_bridge'
alias agr_fly='ros2 run agr_uav_teleop teleop_keyboard'
alias agr_uav_cam='ros2 run agr_uav_teleop camera_view'
alias agr_uav_demo='ros2 run agr_uav_demos demo_flight'
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

# ── ground platform (RaiseBot) ─────────────────────────────────────────────
# tools:=true starts the six service servers. Without them every service call
# blocks FOREVER, which is what a "hanging" lab almost always turns out to be.
alias agr_ground='agr-sim ground tools:=true'
alias agr_ground_lite='agr-sim ground tools:=true world:=greenhouse_2026_lite.sdf'
alias agr_drive='ros2 run raisebot_teleop teleop_keyboard'
alias agr_cam='ros2 run raisebot_teleop camera_view'
alias agr_ground_check='ros2 run raisebot_demos diagnose'
alias agr_ground_demo='ros2 run raisebot_demos demo_greenhouse'
alias agr_ground_topics='ros2 topic list | grep -E "cmd_vel|scan|camera|joint_states|odom"'

# ── Day-2 VLA labs ──────────────────────────────────────────────────────────
# These import LeRobot, which pulls numpy 2.x — incompatible with the apt
# cv_bridge the rest of the stack uses. So they run under their own venv rather
# than the system python. The venv is created with system-site-packages, so it
# still sees rclpy and the raisebot_* packages.
export AGR_LEROBOT_PY="${AGR_LEROBOT_PY:-$HOME/raise_venvs/lerobot/bin/python3}"
export AGR_D2="${AGR_SRC}/ground/raisebot_labs/day2"

# The reference policy is PUBLIC on Hugging Face — no token, no manual download.
# api_clients/vla_client/factory.py resolves in this order:
#   VLA_LOCAL_CKPT  →  ~/raise_checkpoints/smolvla_C_ref  →  this HF repo
# so a student with none of the above still gets a working brain on first run
# (~900 MB into the HF cache, then local forever).
export AGR_VLA_HF_REF="scalexi/smolvla-raise2026-ripeness-ref"

_agr_vla() {
  if [ ! -x "${AGR_LEROBOT_PY}" ]; then
    echo "✗ LeRobot venv not found at ${AGR_LEROBOT_PY}"
    echo "  Set AGR_LEROBOT_PY, or create it — see ground/raisebot_labs/day2/HOW_TO_TRAIN_AND_USE.md"
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

# ── legged platform (Unitree Go2) ───────────────────────────────────────────
alias agr_legged='agr-sim legged'
alias agr_walk='ros2 run agr_legged_teleop teleop_keyboard'
alias agr_legged_cam='ros2 run agr_legged_teleop camera_view'
alias agr_legged_check='ros2 run agr_legged_demos diagnose'
alias agr_legged_demo='ros2 run agr_legged_demos demo_walkabout'
# Pose commands need the gait controller stopped — it publishes to the same
# position controller at 100 Hz and two publishers thrash the robot over.
alias agr_stand='ros2 run agr_legged_examples 04_change_pose --ros-args -p pose:=stand'
alias agr_crouch='ros2 run agr_legged_examples 04_change_pose --ros-args -p pose:=crouch'
alias agr_tuck='ros2 run agr_legged_examples 04_change_pose --ros-args -p pose:=tuck'
alias agr_legged_joints='ros2 run agr_legged_examples 01_read_joints'

agr_help() {
  cat <<'HELP'
Agentic Robotics — three platforms, the same three layers on each.

  Every platform has:  teleop (drive it) · examples (learn it) · demos (show
  it / diagnose it).  When something is wrong, run the diagnose first.

  START A ROBOT
    agr-sim                            UAV: one x500, Gazebo GUI
    agr-sim airframe:=x500_mono_cam    ...with a camera
    agr-sim airframe:=rc_cessna        ...fixed-wing (cannot hover)
    agr-sim world:=agr_city            ...urban world
    agr-sim uav --multi count:=3       ...three vehicles
    agr_ground                         RaiseBot Husky + greenhouse + tools
    agr_ground_lite                    ...lighter world (CPU-only machines)
    agr_legged                         Unitree Go2 on the inspection site
    agr-stop                           stop whichever is running
    add headless:=true to any of them for no GUI

  IS IT WORKING?
    agr_check                          UAV    — 9 checks
    agr_ground_check                   ground — 10 checks
    agr_legged_check                   legged — 9 checks
    ros2 run agr_uav_demos diagnose --ros-args -p active:=true   (really arms it)

  DRIVE IT BY HAND
    agr_fly                            UAV keyboard flight (t=takeoff, l=land)
    agr_drive                          Husky keyboard
    agr_walk                           Go2 keyboard (q/e strafe — no wheels can)
    agr_uav_cam / agr_cam / agr_legged_cam        live camera window

  SHOW IT OFF
    agr_uav_demo                       arm, climb, orbit, land
    agr_ground_demo                    narrated greenhouse inspection
    agr_legged_demo                    walk, strafe, turn on the spot

  LEARN IT — six scripts per platform, one idea each
    ros2 run agr_uav_examples 01_read_telemetry     ... 06_list_airframes
    ros2 run raisebot_examples 01_drive             ... 06_navigate
    ros2 run agr_legged_examples 01_read_joints     ... 06_walk_a_square
    (tab-completion after the package name lists all six)

  POSES (legged)   agr_stand / agr_crouch / agr_tuck
                   needs the gait stopped:  agr-sim legged gait:=false

  BUILD            agr-build                    colcon with the right python
                   agr-build --packages-select agr_uav_tools

  INSPECT (UAV)    agr_airframes                the airframe registry
                   agr_monitor                  live vehicle state
                   agr_topics                   /fmu topics
                   agr_status / agr_pos / agr_batt
                   agr_echo <name> [namespace]  any /fmu/out topic

  VLA (day2)       agr_vla_one                  one inference step
                   agr_vla                      full executor
                   agr_vla_hf                   force the public HF brain
                   agr_record / agr_replay      teleop data collection
                   agr_finetune                 fine-tune SmolVLA
                   (these run under the LeRobot venv, not system python)

  NAVIGATE         agr_ws / agr_src
HELP
}
