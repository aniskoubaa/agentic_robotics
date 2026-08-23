#!/usr/bin/env python3
# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""
RAISE 2026 — grasp server: a deterministic "fake grasp" for the sim.

WHY  Gazebo Harmonic + DART can't reliably grasp with friction, and the Robotiq
     has no grasp plugin — so picking is physically impossible out of the box,
     by hand OR by script. This node adds a deterministic grasp: when the
     gripper closes near a registered tomato, the tomato is "stuck" to the
     gripper (teleported to follow the tool each tick); when the gripper opens,
     it is released. Clean, repeatable, no physics tuning.

     It triggers off the GRIPPER JOINT STATE (/joint_states), not a service
     call — so anyone who closes the gripper (the auto-demonstrator, a teleop
     human, or the trained VLA policy) grasps identically.

HOW (frames — verified live in the headless sim):
     - base_link → gripper_mount_link comes from TF (robot_state_publisher +
       /joint_states): exact kinematics, updated live.
     - world → base_link comes from Gazebo's dynamic_pose/info, read via the
       `gz topic` CLI (the ros_gz Pose_V→TFMessage bridge DROPS entity names,
       so it can't be used). Top-level models are world-frame there. Refreshed
       on a slow timer — fine because the base is still while grasping.
     - tool point = world_T_base ⊗ (base_T_gripper + grasp_offset along the
       gripper's local Z). Registered tomatoes are top-level models: their
       world pose is read once at registration, then tracked as we move them.

USE  Run alongside the sim, during BOTH recording and execution:
         ros2 run raise2026_tools grasp_server          # alias: grasp_d3
     Register a graspable model (the demonstrator does this after spawning):
         ros2 topic pub --once /grasp/register std_msgs/String "{data: tomato_red_0}"
     Watch what's held:  ros2 topic echo /grasp/state
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String

from tf2_ros import Buffer, TransformListener

from raise2026_tools import gz_utils

KNUCKLE = 'gripper_robotiq_85_left_knuckle_joint'   # the gripper's driving joint
# We trigger on the COMMANDED knuckle value (/.../cmd), not the measured joint
# state: with a tomato between the fingers the knuckle physically STALLS on
# contact (verified live — it never reaches the old 0.30 threshold), but the
# command still says "close". Command = intent; that's the grasp signal.
CLOSE_AT = 0.25     # commanded > this (rising) => try to grasp  (open=0.0, closed=0.5)
OPEN_AT = 0.10      # commanded < this (falling) => release      (hysteresis band)


_rotate_vec = gz_utils.rotate_vec   # shared quaternion-rotate helper


class GraspServer(Node):
    def __init__(self):
        super().__init__('grasp_server')

        # ── Tunables (override with --ros-args -p name:=value) ──────────────
        self.declare_parameter('world', gz_utils.DEFAULT_WORLD)
        self.declare_parameter('robot_model', 'raise2026_robot')  # gz model name
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('gripper_link', 'gripper_mount_link')
        self.declare_parameter('attach_radius', 0.15)   # m, tool-to-fruit to grab
        self.declare_parameter('grasp_offset', 0.13)    # m along tool +z to the finger gap
        self.declare_parameter('follow_hz', 15.0)
        self.world = self.get_parameter('world').value
        self.robot_model = self.get_parameter('robot_model').value
        self.base_frame = self.get_parameter('base_frame').value
        self.gripper_link = self.get_parameter('gripper_link').value
        self.attach_radius = float(self.get_parameter('attach_radius').value)
        self.grasp_offset = float(self.get_parameter('grasp_offset').value)
        follow_hz = float(self.get_parameter('follow_hz').value)

        # TF gives base_link -> gripper live; gz gives world -> base slowly.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.world_T_base = None       # ((x,y,z),(qx,qy,qz,qw)) of base in world

        self.fruit_pos = {}            # registered model -> last known world (x,y,z)
        self.pending = set()           # registered, world pose not yet resolved
        self.attached = None           # currently held model name, or None
        self.knuckle = 0.0
        self.closing = False           # latched gripper-closed state (hysteresis)

        # Listen to the knuckle COMMAND topic (see CLOSE_AT note above).
        self.create_subscription(Float64, f'/{KNUCKLE}/cmd', self._on_knuckle_cmd, 10)
        self.create_subscription(String, '/grasp/register', self._on_register, 10)
        self.state_pub = self.create_publisher(String, '/grasp/state', 10)
        self.create_timer(1.0 / follow_hz, self._tick)
        self.create_timer(2.0, self._slow_sync)     # world_T_base + pending fruits
        self._slow_sync()

        self.get_logger().info(
            f'grasp_server ready (world={self.world}, gripper={self.gripper_link}, '
            f'attach_radius={self.attach_radius} m). Register models on /grasp/register.')

    # ── Inputs ──────────────────────────────────────────────────────────────
    def _on_register(self, msg: String):
        name = msg.data.strip()
        if name:
            # ALWAYS re-resolve, even if we knew this name: episodes re-spawn
            # same-named tomatoes at new positions, and a stale position makes
            # the grasp attach the wrong fruit (bug found in the live dry-run).
            self.pending.add(name)
            self.get_logger().info(f'registered graspable: {name} (resolving pose...)')

    def _on_knuckle_cmd(self, msg: Float64):
        self.knuckle = msg.data

    # ── Slow sync: read world poses from gz (subprocess — keep off hot path) ─
    def _slow_sync(self):
        poses = gz_utils.get_world_poses(self.world)
        if not poses:
            return
        if self.robot_model in poses:
            self.world_T_base = poses[self.robot_model]
        for name in list(self.pending):
            if name in poses:
                self.fruit_pos[name] = poses[name][0]
                self.pending.discard(name)
                self.get_logger().info(
                    f'{name} at {tuple(round(v, 2) for v in self.fruit_pos[name])}')
        # Keep FREE fruits' positions in sync with gz truth (they can be pushed
        # by contacts). Never overwrite the attached one — we drive it ourselves.
        for name in self.fruit_pos:
            if name != self.attached and name in poses:
                self.fruit_pos[name] = poses[name][0]
        # If the fruit we are holding was REMOVED from the world (scene
        # cleanup, sim restart), let go — otherwise the follow tick spams the
        # sim with pose updates for a non-existent entity forever. (The
        # set_pose service happily ACCEPTS requests for missing entities, so
        # this world-truth check is the only reliable signal.)
        if self.attached is not None and self.attached not in poses:
            self.get_logger().warn(
                f'"{self.attached}" vanished from the world — dropping it')
            self.fruit_pos.pop(self.attached, None)
            self.attached = None
        # Drop registrations for fruits that no longer exist at all.
        for name in [n for n in self.fruit_pos if n not in poses]:
            self.fruit_pos.pop(name, None)

    # ── Geometry ─────────────────────────────────────────────────────────────
    def _tool_point(self):
        """World position of the grasp point (between the fingers), or None."""
        if self.world_T_base is None:
            return None
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame, self.gripper_link, rclpy.time.Time())
        except Exception:
            return None
        p, q = t.transform.translation, t.transform.rotation
        # grasp point in the BASE frame: gripper origin + offset along its local Z
        off = _rotate_vec((q.x, q.y, q.z, q.w), (0.0, 0.0, self.grasp_offset))
        p_base = (p.x + off[0], p.y + off[1], p.z + off[2])
        # then into the WORLD frame via the robot model's world pose
        (bx, by, bz), bq = self.world_T_base
        w = _rotate_vec(bq, p_base)
        return (bx + w[0], by + w[1], bz + w[2])

    @staticmethod
    def _dist(a, b):
        return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))

    # ── The grasp state machine, ticked at follow_hz ─────────────────────────
    def _tick(self):
        if not self.closing and self.knuckle > CLOSE_AT:
            self.closing = True
            self._try_attach()
        elif self.closing and self.knuckle < OPEN_AT:
            self.closing = False
            self._release()

        # while holding, glue the fruit to the tool (teleport it each tick)
        if self.attached is not None:
            tip = self._tool_point()
            if tip is not None:
                if gz_utils.set_model_pose(self.attached, *tip, world=self.world):
                    self.fruit_pos[self.attached] = tip
                    self._follow_misses = 0
                else:
                    # The fruit no longer exists (scene cleaned up, sim
                    # restarted, …). Let go instead of spamming the sim with
                    # "Unable to update the pose" errors forever.
                    self._follow_misses = getattr(self, '_follow_misses', 0) + 1
                    if self._follow_misses >= 10:
                        self.get_logger().warn(
                            f'"{self.attached}" vanished from the world — dropping it')
                        self.fruit_pos.pop(self.attached, None)
                        self.attached = None
                        self._follow_misses = 0

        self.state_pub.publish(String(data=self.attached or 'none'))

    def _try_attach(self):
        tip = self._tool_point()
        if tip is None:
            self.get_logger().warn('no tool pose yet (TF or world sync missing)')
            return
        best, best_d = None, self.attach_radius
        for name, pos in self.fruit_pos.items():
            d = self._dist(tip, pos)
            if d < best_d:
                best, best_d = name, d
        if best is not None:
            self.attached = best
            self.get_logger().info(f'grasped "{best}" ({best_d * 100:.1f} cm from tool)')
        else:
            self.get_logger().info('gripper closed but no fruit in range — empty grasp')

    def _release(self):
        if self.attached is not None:
            self.get_logger().info(f'released "{self.attached}"')
        self.attached = None


def main():
    rclpy.init()
    node = GraspServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
