#!/usr/bin/env python3
# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""Trot gait for the Go2 — turns /cmd_vel into 12 joint angles.

    ros2 run agr_legged_bringup gait               # started by the launch file
    ros2 run agr_legged_teleop teleop_keyboard     # ...drive it by hand

WHAT  A quadruped has no /cmd_vel. Wheels take a velocity directly; legs only
      take joint angles, so SOMETHING has to turn "go forward at 0.2 m/s" into
      twelve numbers, a hundred times a second. That something is a gait
      generator, and this is the smallest one that actually walks.

LEARN - TROT = the two DIAGONAL pairs alternate. FL+RR push while FR+RL swing,
        then they swap. Two feet are always down and they straddle the centre
        of mass, so the robot is statically sensible at every instant. That is
        why trot, not bound or pronk, is what every quadruped demo uses.
      - The gait is written in FOOT SPACE, not joint space. You say where the
        foot should be; inverse kinematics converts that to hip/thigh/calf
        angles. Writing a gait directly in joint angles is how people spend a
        week and end up with a robot that twitches.
      - A STANCE foot is standing still IN THE WORLD. It only looks like it is
        moving backward because you are watching from the body. That single
        sentence is the whole of legged locomotion: to move the body forward,
        push the planted feet backward through the body frame.
      - The HIP joints are what make this a robot and not a tank. Give each
        foot a sideways component and the Go2 can turn on the spot, or strafe
        sideways without turning at all — neither of which the wheeled RaiseBot
        can do. Leave the hips at 0 and the robot can only skid-steer, which
        works but scrubs the feet and turns at a fraction of the commanded
        rate. That was measured, not assumed: see docs/.

The gait is OPEN LOOP — no IMU feedback, no balance controller, no foothold
planning. On flat ground that is enough and the code stays readable. It is
also why MAX_VX is low: above roughly 0.3 m/s the robot out-runs its own
stability and lands on its back. Raising the limit does not make it faster, it
makes it fall over. Real speed needs state estimation and a balance
controller, and pretending otherwise would teach the wrong lesson.
"""
from __future__ import annotations

import math
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# ─── Robot geometry — must match go2.urdf.xacro ────────────────────────────
L1 = 0.213              # thigh length (m)
L2 = 0.213              # calf length (m)
HIP_X = 0.1934          # hip joint, forward of body centre (m)
HIP_Y = 0.0465          # hip joint, out from body centreline (m)
THIGH_Y = 0.0955        # thigh joint, further out from the hip joint (m)

# Controller joint order, exactly as in go2_controllers.yaml. Get this wrong
# and the robot drives the correct angles into the wrong joints — no error,
# just a robot that folds up.
LEGS = ('FL', 'FR', 'RL', 'RR')
JOINTS = [f'{leg}_{part}_joint' for leg in LEGS
          for part in ('hip', 'thigh', 'calf')]

# Where each hip joint sits, and which way the leg splays out.
# sign_x: +1 front, -1 rear.   sign_y: +1 left, -1 right.
GEOM = {leg: (+1 if leg[0] == 'F' else -1, +1 if leg[1] == 'L' else -1)
        for leg in LEGS}

# Diagonal trot pairs: FL and RR move together, half a cycle from FR and RL.
PHASE_OFFSET = {'FL': 0.0, 'RR': 0.0, 'FR': 0.5, 'RL': 0.5}

# ─── Gait tunables — every one is also a ROS parameter, see __init__ ───────
CYCLE_S      = 0.4      # one full trot cycle (s)
STAND_HEIGHT = 0.28     # trunk above the feet (m). Must be < L1 + L2.
SWING_HEIGHT = 0.05     # how high a swinging foot lifts (m)
RATE_HZ      = 100      # command rate. Below ~50 the legs visibly step.

# MEASURED limits, not guesses. At cycle 0.4 s, commanding more than 0.25 m/s
# does not go faster - it falls over. Measured in agr_inspection on flat
# ground, trunk height treated as "upright" above 0.22 m:
#
#     commanded   achieved    outcome
#       0.20       0.17 m/s   upright
#       0.25       0.18 m/s   upright
#       0.35       0.22 m/s   ON ITS BACK
#       0.50       0.18 m/s   ON ITS BACK
#
# The achieved speed barely rises across that range because the extra stride
# goes into pitching the body, not into travel. That is the signature of an
# open-loop gait at its limit, and it is why the clamp is here rather than a
# comment telling you to be careful.
MAX_VX = 0.25           # m/s   forward / backward  (achieves ~0.18 fwd, ~0.06 back)
MAX_VY = 0.15           # m/s   sideways strafe     (achieves ~0.11)
MAX_WZ = 1.0            # rad/s yaw                 (achieves ~90% of command)

IDLE_STOP_S = 0.5       # no /cmd_vel for this long → stand still
DEADBAND    = 0.02      # below this, do not step at all — a gait asked for
                        # 1 mm/s just shuffles in place and looks broken


def planar_ik(x: float, z: float) -> tuple[float, float]:
    """Foot at (x forward, z up so z<0), in the leg's own plane → thigh, calf.

    Two-link arm. The knee always bends the same way (calf angle negative)
    because the Go2's calf limit is [-2.72, -0.84] rad: the leg physically
    cannot straighten, or bend the other way.
    """
    r = min(math.hypot(x, z), L1 + L2 - 1e-4)   # never ask for a straight leg
    # Law of cosines at the knee. Clamped because floating point WILL hand you
    # 1.0000000002 near full extension, and acos() then raises.
    cos_knee = (r * r - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    calf = -math.acos(max(-1.0, min(1.0, cos_knee)))
    # Angle from straight-down to the foot, minus the thigh's own contribution.
    phi = math.atan2(-x, -z)
    alpha = math.atan2(L2 * math.sin(calf), L1 + L2 * math.cos(calf))
    return phi - alpha, calf


def leg_ik(x: float, y: float, z: float, y_off: float) -> tuple[float, float, float]:
    """Foot at (x, y, z) relative to the HIP JOINT → hip, thigh, calf angles.

    Three joints, three numbers — but they are not independent. The hip
    rotates the whole leg about the forward axis, so it controls how far out
    sideways the foot goes; only then do thigh and calf work in the plane the
    hip just chose. Solve the hip first, then the plane.

    `y_off` is the fixed sideways offset from hip joint to thigh joint
    (+0.0955 on the left legs, -0.0955 on the right). It is the reason a hip
    angle of 0 is not the same as a foot directly below the hip joint.
    """
    # Everything below the hip lies at a fixed distance from the hip axis, so
    # in the y-z plane the leg is a rigid rod of length rho pinned at the hip.
    rho = math.hypot(y, z)
    # The rod's own length is hypot(y_off, z_leg); invert that for z_leg.
    z_leg = -math.sqrt(max(0.0, rho * rho - y_off * y_off))
    if z_leg == 0.0:            # asked for a foot at or inside the hip axis
        z_leg = -1e-4           # degenerate; keep the atan2 below meaningful
    hip = math.atan2(z, y) - math.atan2(z_leg, y_off)
    # Normalise into (-pi, pi]: the subtraction above can leave +/-2pi on it,
    # which is the same pose but far outside the joint limit.
    hip = math.atan2(math.sin(hip), math.cos(hip))
    thigh, calf = planar_ik(x, z_leg)
    return hip, thigh, calf


class Gait(Node):
    def __init__(self) -> None:
        super().__init__('gait_controller')
        # Every gait constant is a ROS parameter, because tuning a gait is a
        # purely empirical exercise: change one number, watch the robot, change
        # it back. Needing a rebuild between tries makes that loop so slow that
        # people stop doing it and settle for a bad gait.
        #     ros2 param set /gait_controller cycle_s 0.6
        for name, default in (('cycle_s', CYCLE_S),
                              ('stand_height', STAND_HEIGHT),
                              ('swing_height', SWING_HEIGHT),
                              ('max_vx', MAX_VX),
                              ('max_vy', MAX_VY),
                              ('max_wz', MAX_WZ)):
            self.declare_parameter(name, default)

        self._pub = self.create_publisher(
            Float64MultiArray, '/joint_group_position_controller/commands', 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self._cmd = (0.0, 0.0, 0.0)     # vx, vy, wz
        self._phase = 0.0
        self._last_cmd_ns = 0
        self.create_timer(1.0 / RATE_HZ, self._tick)
        self.get_logger().info(
            f'trot gait ready — cycle {self._p("cycle_s"):.2f}s, stand height '
            f'{self._p("stand_height"):.2f}m, max {self._p("max_vx"):.2f} m/s. '
            f'Publish /cmd_vel to walk.')

    def _p(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _on_cmd(self, msg: Twist) -> None:
        def clamp(v, lim):
            return max(-lim, min(lim, v))
        self._cmd = (clamp(msg.linear.x, self._p('max_vx')),
                     clamp(msg.linear.y, self._p('max_vy')),
                     clamp(msg.angular.z, self._p('max_wz')))
        self._last_cmd_ns = self.get_clock().now().nanoseconds

    def _tick(self) -> None:
        # Treat a silent publisher as "stop". Teleop nodes get Ctrl-C'd
        # mid-stride; without this the robot trots away on its own.
        stale = (self.get_clock().now().nanoseconds - self._last_cmd_ns
                 > IDLE_STOP_S * 1e9)
        vx, vy, wz = (0.0, 0.0, 0.0) if stale else self._cmd
        stand_h = self._p('stand_height')

        if max(abs(vx), abs(vy), abs(wz)) < DEADBAND:
            # Hold a still, level stance and reset the clock, so the next
            # command starts at the beginning of a cycle rather than mid-swing
            # with one foot already in the air.
            self._phase = 0.0
            self._publish({leg: self._solve(leg, 0.0, 0.0, stand_h)
                           for leg in LEGS})
            return

        cycle_s = self._p('cycle_s')
        swing_h = self._p('swing_height')
        self._phase = (self._phase + 1.0 / (RATE_HZ * cycle_s)) % 1.0

        # Stance lasts half a cycle, so a stride of S gives a body speed of
        # S / (cycle/2). Invert that for the stride each foot needs.
        t_stance = 0.5 * cycle_s

        angles = {}
        for leg in LEGS:
            sx, sy = GEOM[leg]
            # Where this foot stands, relative to the body centre.
            px, py = sx * HIP_X, sy * (HIP_Y + THIGH_Y)
            # Body-frame velocity the foot must track while planted. A point
            # fixed in the WORLD drifts through the body frame at -(v + w x p),
            # and a stance foot is exactly such a point. This cross-product
            # term is the whole of turning: it gives the outer feet a longer
            # stride AND a sideways component, which is what the hip joint is
            # there to deliver.
            dx = vx - wz * py
            dy = vy + wz * px
            stride_x, stride_y = dx * t_stance, dy * t_stance

            phase = (self._phase + PHASE_OFFSET[leg]) % 1.0
            if phase < 0.5:                       # STANCE: slide back, planted
                s = phase / 0.5
                fx, fy, lift = stride_x * (0.5 - s), stride_y * (0.5 - s), 0.0
            else:                                 # SWING: lift and reach ahead
                s = (phase - 0.5) / 0.5
                # Half-sine: starts and ends at zero height, so the foot leaves
                # and meets the ground smoothly instead of stubbing it.
                fx, fy = stride_x * (s - 0.5), stride_y * (s - 0.5)
                lift = swing_h * math.sin(math.pi * s)
            angles[leg] = self._solve(leg, fx, fy, stand_h - lift)
        self._publish(angles)

    def _solve(self, leg: str, fx: float, fy: float, height: float) -> tuple:
        """Foot offset (fx, fy) from its nominal stance, at `height` below."""
        y_off = GEOM[leg][1] * THIGH_Y
        return leg_ik(fx, y_off + fy, -height, y_off)

    def _publish(self, angles: dict) -> None:
        msg = Float64MultiArray()
        msg.data = [v for leg in LEGS for v in angles[leg]]
        self._pub.publish(msg)


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = Gait()
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
    return 0


if __name__ == '__main__':
    sys.exit(main())
