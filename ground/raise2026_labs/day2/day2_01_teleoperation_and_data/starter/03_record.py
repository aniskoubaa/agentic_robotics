#!/usr/bin/env python3
# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""
RAISE 2026 — Lab 2.1, Script 03 — record demonstrations in LeRobot format.

WHAT  While you teleoperate (Script 01 in another terminal), this captures the
      wrist image + joint state + the command you sent, ~30 times a second, and
      writes them as a LeRobot episode. Do this ~30-50 times and you have the
      dataset your VLA will be fine-tuned on.

WHY   This IS the morning's deliverable. Everything the model learns comes from
      here. The single Day-2 lesson — "what you record in the morning is the
      brain you command in the afternoon" — starts at this script.

LEARN A behavioral-cloning training example is a tuple at each timestep:
        observation.images.wrist : 224x224x3 RGB  (what the robot SEES)
        observation.state        : 6 joints + gripper  (where the robot IS)
        action                   : 6 joints + gripper  (what you COMMANDED)
      The model learns the mapping  (image, instruction, state) -> action.
      We read `state` from /joint_states and `action` from the /..._joint/cmd
      topics your teleop publishes — so action is the REAL command, not a guess.

      Targets the LeRobot v3.0 dataset API (create / add_frame / save_episode /
      finalize). Install:  pip install "lerobot[smolvla]"   (numpy<2 same resolve)

Workflow (two terminals):
    Terminal 1:  ros2 run raise2026_labs 01_teleop.py --task A
    Terminal 2:  ros2 run raise2026_labs 03_record.py --task A --team team07 \
                     --hf-user raiseschool --episodes 30
    Then follow the prompts: ENTER to start an episode, ENTER to stop, keep/discard.
    (or:  03_d3 ...)
"""

import argparse
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64

try:
    import _d2paths                    # installed next to the ros2-run scripts
except ImportError:                    # running straight from the source tree
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    import _d2paths
_d2paths.bootstrap(lerobot=True)
DAY2 = _d2paths.DAY2
from vla_client.base import UR5E_JOINTS, GRIPPER_KNUCKLE, GRIPPER_OPEN   # noqa: E402
from vla_client.ros_image import imgmsg_to_rgb, resize_rgb   # noqa: E402  (no cv_bridge — numpy-2 safe)
from task_pack import load_task                                          # noqa: E402

IMAGE_TOPIC = '/wrist_camera/image_raw'
STATE_TOPIC = '/joint_states'
STATE_JOINTS = UR5E_JOINTS + [GRIPPER_KNUCKLE]   # 7-vector order for state + action
IMG_SIZE = 224
FPS = 30


class Recorder(Node):
    """Caches the latest image/state/command and, when recording, buffers frames."""

    def __init__(self):
        super().__init__('raise_recorder')
        self.latest_img = None                      # (224,224,3) uint8
        self.latest_state = {}                      # joint -> measured position
        self.latest_cmd = {j: None for j in STATE_JOINTS}   # joint -> last command

        self.create_subscription(Image, IMAGE_TOPIC, self._on_image, 10)
        self.create_subscription(JointState, STATE_TOPIC, self._on_state, 10)
        # Listen to the SAME command topics teleop publishes, so `action` is the
        # true commanded target rather than a reconstruction.
        for j in STATE_JOINTS:
            self.create_subscription(Float64, f'/{j}/cmd',
                                     lambda m, jn=j: self.latest_cmd.__setitem__(jn, m.data), 10)

        self.recording = False
        self.frames = []                            # buffered frames for the current episode
        self.create_timer(1.0 / FPS, self._tick)    # capture at a fixed 30 Hz

    def _on_image(self, msg: Image):
        # Pure-numpy conversion + resize (no cv_bridge/cv2 — works under the
        # lerobot venv's numpy 2 AND the system's numpy 1).
        self.latest_img = resize_rgb(imgmsg_to_rgb(msg), IMG_SIZE)

    def _on_state(self, msg: JointState):
        self.latest_state = dict(zip(msg.name, msg.position))

    def _state_vec(self):
        if not all(j in self.latest_state for j in STATE_JOINTS):
            return None
        return np.array([self.latest_state[j] for j in STATE_JOINTS], dtype=np.float32)

    def _action_vec(self, state_vec):
        # action = last command on each /cmd topic; if a joint was never
        # commanded yet, hold its current measured position (no jump).
        return np.array(
            [self.latest_cmd[j] if self.latest_cmd[j] is not None else state_vec[i]
             for i, j in enumerate(STATE_JOINTS)],
            dtype=np.float32,
        )

    def _tick(self):
        """Called at 30 Hz. When recording, append one synced, validated frame."""
        if not self.recording:
            return
        state = self._state_vec()
        if state is None or self.latest_img is None:
            return                                  # not all streams ready — skip
        action = self._action_vec(state)
        if np.isnan(state).any() or np.isnan(action).any():
            return                                  # never record a NaN frame
        self.frames.append({
            'observation.images.wrist': self.latest_img.copy(),
            'observation.state': state,
            'action': action,
        })


def make_features():
    """The LeRobot feature schema — must match the keys we add per frame."""
    return {
        'observation.images.wrist': {
            'dtype': 'image', 'shape': (IMG_SIZE, IMG_SIZE, 3),
            'names': ['height', 'width', 'channel'],
        },
        'observation.state': {'dtype': 'float32', 'shape': (len(STATE_JOINTS),), 'names': STATE_JOINTS},
        'action': {'dtype': 'float32', 'shape': (len(STATE_JOINTS),), 'names': STATE_JOINTS},
    }


def main():
    ap = argparse.ArgumentParser(description='RAISE 2026 LeRobot demo recorder')
    ap.add_argument('--task', default='A', help='A | C | B')
    ap.add_argument('--team', default='team00', help='your team id (names the dataset)')
    ap.add_argument('--hf-user', default='raiseschool', help='HF/username prefix for the dataset')
    ap.add_argument('--episodes', type=int, default=None, help='target episode count (default = pack recommendation)')
    ap.add_argument('--root', default=str(_d2paths.DATASETS_DIR), help='local dataset root')
    args = ap.parse_args()

    pack = load_task(args.task)
    target_eps = args.episodes or pack.recommended_episodes
    repo_id = pack.dataset_repo_id(args.hf_user, args.team)
    root = Path(args.root) / repo_id.replace('/', '__')

    # Lazy import: lerobot is multi-GB and only needed to actually record.
    try:
        try:                                    # LeRobot moved this module between releases
            from lerobot.datasets.lerobot_dataset import LeRobotDataset
        except ImportError:
            from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
    except ImportError:
        print('✗ lerobot not installed. Run:  pip install "lerobot[smolvla]"  (numpy<2 same resolve)')
        sys.exit(1)

    print(f'Task {pack.id}: {pack.name}')
    print(f'Dataset repo_id : {repo_id}')
    print(f'Local root      : {root}')
    print(f'Target episodes : {target_eps}\n')
    print('Instructions to cover (vary them across episodes):')
    for s in pack.instructions:
        print(f'   • {s}')
    print()

    dataset = LeRobotDataset.create(
        repo_id=repo_id, fps=FPS, features=make_features(),
        robot_type='ur5e', root=str(root),
    )

    rclpy.init()
    node = Recorder()
    # Spin ROS in the background so the 30 Hz timer keeps firing while the main
    # thread blocks on keyboard prompts.
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    ep = 0
    instr_cycle = pack.instructions
    try:
        while ep < target_eps and rclpy.ok():
            instruction = instr_cycle[ep % len(instr_cycle)]
            input(f'\n[episode {ep + 1}/{target_eps}]  instruction: "{instruction}"\n'
                  f'  Drive to the START pose, then press ENTER to RECORD...')
            node.frames = []
            node.recording = True
            input('  ● RECORDING — perform the demo, then press ENTER to STOP...')
            node.recording = False
            n = len(node.frames)
            time.sleep(0.1)   # let the timer settle

            choice = input(f'  captured {n} frames ({n / FPS:.1f}s). keep? [Y/n/q]: ').strip().lower()
            if choice == 'q':
                break
            if choice == 'n' or n < FPS:   # discard short/bad takes (<1 s)
                print('  discarded.')
                continue

            for fr in node.frames:
                # lerobot >= 0.5: the task string is a FIELD of the frame.
                dataset.add_frame({**fr, 'task': instruction})
            dataset.save_episode()
            ep += 1
            print(f'  ✓ saved episode {ep}.')
    except KeyboardInterrupt:
        pass
    finally:
        # v3.0: finalize writes dataset-level stats/metadata once recording ends.
        try:
            dataset.finalize()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
        print(f'\nDone. {ep} episodes in {root}. Next: 04_upload.py to ship them.')


if __name__ == '__main__':
    main()
