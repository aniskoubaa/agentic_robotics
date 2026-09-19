#!/usr/bin/env python3
# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""
AGR Arm — grasp server: decides when the gripper is holding a block, and
makes that true in the physics.

WHY IT EXISTS
    Gazebo Harmonic with DART will not hold a 50 mm cube between two rubber
    pads. Measured on this machine: close the Robotiq on a block and the
    knuckle runs straight past the block's width while the block squirts out
    sideways and ends up 5 cm away. That is not a tuning problem you can win.

    So a grasp here is a real KINEMATIC CONSTRAINT rather than friction. Each
    block has a gz DetachableJoint declared against the arm's wrist link (see
    agr_arm_gazebo.urdf.xacro); this node welds and breaks it. While a block
    is welded the arm carries its mass, the fingers still close around it,
    and opening the gripper still drops it — everything downstream behaves
    exactly as it would with a real gripper on real hardware.

WHAT TRIGGERS IT
    The COMMANDED knuckle angle on /<knuckle>/cmd, not the measured one.
    Command is intent, and intent is the right trigger: it arrives before the
    fingers do, so the block is welded BEFORE the pads reach it and therefore
    cannot be knocked away by its own grasp. Waiting for the fingers to
    actually close would be too late every time.

FRAMES
    The base is bolted to the world, so TF's `world` IS the Gazebo world
    frame and there is no robot pose to track — a real simplification over
    the mobile manipulator this arm is otherwise identical to. The grasp
    point is TF(world -> ur5e_tool0) shifted TCP_OFFSET along the tool's +z.
    Block positions come from `gz topic -e .../dynamic_pose/info`, because
    the ros_gz Pose_V bridge drops entity names (see gz_utils).

THE STARTUP DANCE
    DetachableJoint has no "start detached" setting: it welds on load. So
    every block is stuck to the wrist from the moment the robot spawns, and
    the first thing this node does is break all of them and put the blocks
    back where the world file says they belong. That is why it reads the SDF.

USE
    ros2 run agr_arm_tools grasp_server
    ros2 topic echo /grasp/state              # the model being held, or 'none'
    ros2 topic pub --once /grasp/reset std_msgs/String "{data: ''}"
"""

import math
import os
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from tf2_ros import Buffer, TransformListener

from agr_arm_tools import gz_utils
from agr_arm_tools.arm import GRIPPER_CLOSED, KNUCKLE
from agr_arm_tools.kinematics import TCP_OFFSET

# Grab on the COMMAND, let go on the MEASUREMENT. The asymmetry is not an
# accident and it is the difference between placing a block and firing it
# across the bench.
#
# GRABBING on the command means the weld forms BEFORE the pads arrive, so a
# block cannot be knocked out of the way by the very grasp that is closing on
# it. Waiting for the fingers to actually reach it is too late every time.
#
# RELEASING on the command would break the weld while the fingers are still
# clamped. They are not resting against the block, they are pressing: stalled
# 0.3 rad short of their target with a 200 p_gain behind them. Cut the weld at
# that moment and the stored pinch force launches the block — measured, it
# came out 15 cm from where it was placed. Waiting for the MEASURED opening
# means the pads have physically let go before the constraint does, and the
# block simply drops.
CLOSE_AT = GRIPPER_CLOSED * 0.5     # rad, commanded
RELEASE_AT = 0.15                   # rad, MEASURED — pads are clear by here


class GraspServer(Node):
    def __init__(self):
        super().__init__('grasp_server')

        self.declare_parameter('world', gz_utils.DEFAULT_WORLD)
        self.declare_parameter('tool_frame', 'ur5e_tool0')
        self.declare_parameter('base_frame', 'world')
        self.declare_parameter('grasp_offset', TCP_OFFSET)
        # 0.07 m: half a block plus the few millimetres of tool error, and
        # comfortably less than the 0.18 m between neighbouring blocks, so
        # closing over one can never pick up the next.
        self.declare_parameter('attach_radius', 0.07)
        self.declare_parameter('object_prefix', 'block_')
        self.declare_parameter('rate_hz', 20.0)

        self.world = self.get_parameter('world').value
        self.tool_frame = self.get_parameter('tool_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.grasp_offset = float(self.get_parameter('grasp_offset').value)
        self.attach_radius = float(self.get_parameter('attach_radius').value)
        self.prefix = self.get_parameter('object_prefix').value
        rate_hz = float(self.get_parameter('rate_hz').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.home_poses = self._read_world_layout()
        self.objects = dict(self.home_poses)     # name -> last known world xyz
        self.attached = None
        self.knuckle_cmd = 0.0
        self.knuckle_measured = 0.0
        self.closing = False
        self._resetting = False

        self._break_everything()

        self.create_subscription(Float64, f'/{KNUCKLE}/cmd', self._on_knuckle, 10)
        self.create_subscription(JointState, '/joint_states', self._on_joints, 10)
        self.create_subscription(String, '/grasp/reset', self._on_reset, 10)
        self.state_pub = self.create_publisher(String, '/grasp/state', 10)
        self.create_timer(1.0 / rate_hz, self._tick)
        self.create_timer(1.0, self._sync)

        self.get_logger().info(
            f'grasp_server ready (world={self.world}, radius={self.attach_radius} m, '
            f'graspable: {", ".join(sorted(self.objects)) or "nothing found"})')

    # ── Startup ─────────────────────────────────────────────────────────────
    def _read_world_layout(self) -> dict:
        """Declared starting position of every graspable model in the world."""
        try:
            share = get_package_share_directory('agr_arm_worlds')
        except Exception:
            self.get_logger().warn('agr_arm_worlds not found; cannot restore layout')
            return {}
        path = os.path.join(share, 'worlds', f'{self.world}.sdf')
        poses = gz_utils.parse_world_model_poses(path)
        found = {n: p for n, p in poses.items()
                 if not self.prefix or n.startswith(self.prefix)}
        if not found:
            self.get_logger().warn(f'no {self.prefix}* models declared in {path}')
        return found

    def _break_everything(self, attempts: int = 4) -> bool:
        """Detach every block and put it back where the world file wants it.

        Both halves are needed. The detach is because DetachableJoint welds on
        load; the pose restore is because by the time this node is up, the arm
        has already carried the welded blocks wherever it went.

        The SETTLE and the RETRY are needed too, and that is the part that is
        easy to get wrong. Detaching is asynchronous — the plugin acts on the
        message in its next PreUpdate — so a set_pose issued straight
        afterwards is still fighting a live weld and the block snaps back.
        The first version of this restored exactly one of three blocks, the
        last one, purely because it happened to be far enough behind the
        detach. So: break every weld, let Gazebo step, then place, then check
        the world and try again for whatever did not land.
        """
        if not self.home_poses:
            return True
        for name in self.home_poses:
            gz_utils.publish_empty(f'/grasp/{name}/detach')
        for _ in range(attempts):
            time.sleep(0.5)
            for name, home in self.home_poses.items():
                gz_utils.set_model_pose(name, *home, world=self.world)
            time.sleep(0.4)
            live = gz_utils.get_world_poses(self.world)
            stuck = [n for n, home in self.home_poses.items()
                     if n in live and math.dist(live[n][0], home) > 0.02]
            if not stuck:
                self.get_logger().info(
                    f'detached and reset {len(self.home_poses)} block(s) to their '
                    f'declared positions')
                return True
            for name in stuck:
                gz_utils.publish_empty(f'/grasp/{name}/detach')
        self.get_logger().warn(
            f'could not reset {stuck} — they may still be welded to the wrist')
        return False

    # ── Inputs ──────────────────────────────────────────────────────────────
    def _on_knuckle(self, msg: Float64):
        self.knuckle_cmd = msg.data

    def _on_joints(self, msg: JointState):
        if KNUCKLE in msg.name:
            self.knuckle_measured = msg.position[list(msg.name).index(KNUCKLE)]

    def _on_reset(self, msg: String):
        """Put the scene back: drop whatever is held, blocks to their marks.

        Guarded against re-entry. A reset takes a couple of seconds — detach,
        let Gazebo step, place, verify — and it runs inside this callback, so
        a second message arriving meanwhile would otherwise queue up and fire
        AFTER the caller had moved on. That is not hypothetical: a test that
        published reset five times to "make sure" had the fifth one land four
        seconds into the first pick and quietly put the block back on the
        bench while the arm carried on with an empty gripper.
        """
        if self._resetting:
            self.get_logger().info('reset already in progress — ignoring')
            return
        self._resetting = True
        try:
            self.attached = None
            self.closing = False
            self._break_everything()
            self.objects = dict(self.home_poses)
        finally:
            self._resetting = False

    # ── Slow sync with Gazebo truth (shells out; keep off the hot path) ─────
    def _sync(self):
        poses = gz_utils.get_world_poses(self.world)
        if not poses:
            return
        for name in list(self.objects):
            if name in poses:
                self.objects[name] = poses[name][0]

    # ── Geometry ────────────────────────────────────────────────────────────
    def grasp_point(self):
        """World position of the point between the pads, or None."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame, self.tool_frame, rclpy.time.Time())
        except Exception:
            return None
        p, q = t.transform.translation, t.transform.rotation
        off = gz_utils.rotate_vec((q.x, q.y, q.z, q.w),
                                  (0.0, 0.0, self.grasp_offset))
        return (p.x + off[0], p.y + off[1], p.z + off[2])

    # ── The state machine ───────────────────────────────────────────────────
    def _tick(self):
        if not self.closing and self.knuckle_cmd > CLOSE_AT:
            self.closing = True
            self._try_attach()
        elif self.closing and self.knuckle_cmd < CLOSE_AT \
                and self.knuckle_measured < RELEASE_AT:
            self.closing = False
            self._release()
        self.state_pub.publish(String(data=self.attached or 'none'))

    def _try_attach(self):
        tip = self.grasp_point()
        if tip is None:
            self.get_logger().warn('gripper closed but TF has no tool pose yet')
            return
        best, best_d = None, self.attach_radius
        for name, pos in self.objects.items():
            d = math.dist(tip, pos)
            if d < best_d:
                best, best_d = name, d
        if best is None:
            nearest = min((math.dist(tip, p) for p in self.objects.values()),
                          default=float('inf'))
            self.get_logger().info(
                'gripper closed on nothing' +
                (f' — nearest block is {nearest * 100:.1f} cm away, '
                 f'needs to be under {self.attach_radius * 100:.0f} cm'
                 if math.isfinite(nearest) else ' — no blocks known'))
            return
        gz_utils.publish_empty(f'/grasp/{best}/attach')
        self.attached = best
        self.get_logger().info(f'holding "{best}" ({best_d * 100:.1f} cm from the pads)')

    def _release(self):
        if self.attached is None:
            return
        gz_utils.publish_empty(f'/grasp/{self.attached}/detach')
        self.get_logger().info(f'released "{self.attached}"')
        self.attached = None


def main():
    rclpy.init()
    node = GraspServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
