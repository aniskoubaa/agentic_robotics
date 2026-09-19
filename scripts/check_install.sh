#!/usr/bin/env bash
# Read-only readiness checks; does not launch robots or download models.
set -euo pipefail
AGR_SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export AGR_WS="${AGR_WS:-$(cd "${AGR_SRC}/../.." && pwd)}"
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH
failed=0
check() {
  local label=$1; shift
  if "$@"; then echo "PASS: $label"; else echo "FAIL: $label" >&2; failed=$((failed + 1)); fi
}
if [[ -f /opt/ros/jazzy/setup.bash && -f "${AGR_WS}/install/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  # shellcheck disable=SC1091
  source "${AGR_WS}/install/setup.bash"
  set -u
else
  echo 'FAIL: ROS 2 Jazzy or workspace setup.bash missing' >&2
  exit 1
fi
check 'ROS, OpenCV image conversion, AI imports and tensor operations' /usr/bin/python3 - <<'PY'
import numpy as np
import rclpy, openai, ultralytics, torch, torchvision, flask
from cv_bridge import CvBridge
assert int(np.__version__.split('.')[0]) < 2, np.__version__
bridge = CvBridge()
a = np.zeros((4, 4, 3), dtype=np.uint8)
assert np.array_equal(a, bridge.imgmsg_to_cv2(bridge.cv2_to_imgmsg(a, 'bgr8'), 'bgr8'))
assert (torch.ones(2) + 1).tolist() == [2, 2]
print('PyTorch:', torch.__version__)
PY
check 'LeRobot dataset and SmolVLA imports' "${HOME}/raise_venvs/lerobot/bin/python3" - <<'PY'
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
import torch, fastapi, uvicorn
print('LeRobot PyTorch:', torch.__version__, 'CUDA available:', torch.cuda.is_available())
PY
check 'PX4 SITL binary' test -x "${PX4_DIR:-${HOME}/PX4-Autopilot}/build/px4_sitl_default/bin/px4"
check 'PX4 pinned revision' test "$(git -C "${PX4_DIR:-${HOME}/PX4-Autopilot}" rev-parse HEAD 2>/dev/null || true)" = 03bf4a5e95074c08409297d3b3683a4aa0e19208
check 'px4_msgs pinned revision' test "$(git -C "${AGR_WS}/src/px4_msgs" rev-parse HEAD 2>/dev/null || true)" = a4a9864b3c40f0b7e5176e6543872a251b2e3c70
check 'Micro XRCE-DDS Agent' command -v MicroXRCEAgent
check 'Micro XRCE-DDS Agent v3.0.0 source' test "$(git -C "${HOME}/Micro-XRCE-DDS-Agent" describe --tags --exact-match 2>/dev/null || true)" = v3.0.0
check 'Gazebo Harmonic' gz sim --versions
check 'all package.xml system dependencies' rosdep check --from-paths "$AGR_SRC" --ignore-src --rosdistro jazzy
for pkg in px4_msgs agr_core agr_uav_bringup raisebot_bringup agr_legged_bringup agr_arm_bringup agr_tb_bringup; do
  check "$pkg" ros2 pkg prefix "$pkg"
done
for cmd in agr-sim agr-stop agr-build; do
  check "$cmd command link" test "$(readlink -f "/usr/local/bin/$cmd" 2>/dev/null || true)" = "${AGR_SRC}/$cmd"
done
if (( failed )); then echo "$failed readiness check(s) failed." >&2; exit 1; fi
echo 'Readiness checks passed. Run each platform diagnose command after launching its simulator.'
