#!/usr/bin/env python3
"""Bridge pathFollower /cmd_vel to Unitree Go2 Sport Move API."""

import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from unitree_api.msg import Request
from unitree_go.msg import WirelessController

from go2_cmd_vel_bridge.sport_client import SportClient

# Go2 wireless controller key bits (see unitree_sdk2 advanced_gamepad.hpp)
KEY_L2 = 1 << 5
KEY_R2 = 1 << 4
KEY_L2_R2 = KEY_L2 | KEY_R2


def l2_r2_pressed(keys):
    return (keys & KEY_L2_R2) == KEY_L2_R2


def edge_pressed(now, prev):
    return now and not prev


def clamp(value, limit):
    limit = abs(limit)
    return max(-limit, min(limit, value))


def sticks_active(lx, ly, rx, deadzone):
    return (abs(lx) > deadzone or abs(ly) > deadzone or abs(rx) > deadzone)


def sticks_to_velocity(lx, ly, rx, max_linear_x, max_linear_y, max_angular_z):
    """Unitree stick convention: ly=forward, lx=lateral, rx=yaw."""
    return (
        clamp(ly * max_linear_x, max_linear_x),
        clamp(lx * max_linear_y, max_linear_y),
        clamp(rx * max_angular_z, max_angular_z),
    )


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('go2_cmd_vel_bridge')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('sport_request_topic', '/api/sport/request')
        self.declare_parameter('wireless_controller_topic', '/wirelesscontroller')
        self.declare_parameter('max_linear_x', 0.5)
        self.declare_parameter('max_linear_y', 0.3)
        self.declare_parameter('max_angular_z', 1.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('stick_timeout', 0.5)
        self.declare_parameter('stick_deadzone', 0.1)
        self.declare_parameter('stream_rate', 10.0)
        self.declare_parameter('zero_threshold', 0.01)

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        sport_request_topic = self.get_parameter('sport_request_topic').value
        wireless_topic = self.get_parameter('wireless_controller_topic').value
        self._max_linear_x = float(self.get_parameter('max_linear_x').value)
        self._max_linear_y = float(self.get_parameter('max_linear_y').value)
        self._max_angular_z = float(self.get_parameter('max_angular_z').value)
        self._cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)
        self._stick_timeout = float(self.get_parameter('stick_timeout').value)
        self._stick_deadzone = float(self.get_parameter('stick_deadzone').value)
        stream_rate = float(self.get_parameter('stream_rate').value)
        self._zero_threshold = float(self.get_parameter('zero_threshold').value)

        self._sport_pub = self.create_publisher(Request, sport_request_topic, 10)
        self._sport = SportClient(self._sport_pub)

        self._autonomy_mode = False
        self._last_keys = 0
        self._last_cmd_time = time.monotonic()
        self._last_stick_time = 0.0
        self._active = False
        self._last_vx = 0.0
        self._last_vy = 0.0
        self._last_wz = 0.0
        self._lx = 0.0
        self._ly = 0.0
        self._rx = 0.0
        self._stick_override = False

        self.create_subscription(
            TwistStamped, cmd_vel_topic, self._cmd_vel_callback, 10)
        self.create_subscription(
            WirelessController, wireless_topic, self._wireless_callback, 10)
        self.create_timer(1.0 / stream_rate, self._stream_callback)

        # Default regular gait; further gait changes stay on the Unitree remote.
        self._sport.classic_walk(True)

        self.get_logger().info(
            'Remote control by default (ClassicWalk). Press L2+R2 to toggle '
            'autonomy. In autonomy, stick overrides nav. Listening on %s, '
            'Sport on %s' % (cmd_vel_topic, sport_request_topic))

    def _enter_autonomy_mode(self):
        self._sport.switch_joystick(False)
        self._last_cmd_time = time.monotonic()
        self._stick_override = False
        self.get_logger().info(
            'Autonomy ENABLED (nav /cmd_vel; Unitree stick overrides when active)')

    def _exit_autonomy_mode(self):
        self._active = False
        self._stick_override = False
        self._last_vx = 0.0
        self._last_vy = 0.0
        self._last_wz = 0.0
        self._sport.move(0.0, 0.0, 0.0)
        self._sport.switch_joystick(True)
        self.get_logger().info('Remote control RESTORED (autonomy disabled)')

    def _toggle_autonomy_mode(self):
        self._autonomy_mode = not self._autonomy_mode
        if self._autonomy_mode:
            self._enter_autonomy_mode()
        else:
            self._exit_autonomy_mode()

    def _wireless_callback(self, msg: WirelessController):
        keys = msg.keys
        if edge_pressed(l2_r2_pressed(keys), l2_r2_pressed(self._last_keys)):
            self._toggle_autonomy_mode()
        self._last_keys = keys

        self._lx = float(msg.lx)
        self._ly = float(msg.ly)
        self._rx = float(msg.rx)
        self._last_stick_time = time.monotonic()

    def _is_zero(self, vx, vy, wz):
        return (abs(vx) < self._zero_threshold and
                abs(vy) < self._zero_threshold and
                abs(wz) < self._zero_threshold)

    def _cmd_vel_callback(self, msg: TwistStamped):
        if not self._autonomy_mode:
            return

        self._last_cmd_time = time.monotonic()
        self._last_vx = clamp(msg.twist.linear.x, self._max_linear_x)
        self._last_vy = clamp(msg.twist.linear.y, self._max_linear_y)
        self._last_wz = clamp(msg.twist.angular.z, self._max_angular_z)
        self._active = not self._is_zero(self._last_vx, self._last_vy, self._last_wz)

    def _stick_override_active(self, now):
        if (now - self._last_stick_time) > self._stick_timeout:
            return False
        return sticks_active(
            self._lx, self._ly, self._rx, self._stick_deadzone)

    def _stream_callback(self):
        if not self._autonomy_mode:
            return

        now = time.monotonic()

        if self._stick_override_active(now):
            if not self._stick_override:
                self.get_logger().info('Stick override ON')
                self._stick_override = True
            vx, vy, wz = sticks_to_velocity(
                self._lx, self._ly, self._rx,
                self._max_linear_x, self._max_linear_y, self._max_angular_z)
            self._sport.move(vx, vy, wz)
            return

        if self._stick_override:
            self.get_logger().info('Stick override OFF, resume nav')
            self._stick_override = False

        timed_out = (now - self._last_cmd_time) > self._cmd_vel_timeout

        if not self._active or timed_out:
            if self._active and timed_out:
                self.get_logger().warn(
                    'cmd_vel timeout (%.2fs), holding zero velocity' %
                    (now - self._last_cmd_time))
            self._active = False
            self._last_vx = 0.0
            self._last_vy = 0.0
            self._last_wz = 0.0

        # Keep publishing Move at a fixed rate. Use Move(0,0,0) instead of
        # StopMove so the sport controller stays in velocity mode.
        self._sport.move(self._last_vx, self._last_vy, self._last_wz)

    def shutdown(self):
        if self._autonomy_mode:
            self._sport.move(0.0, 0.0, 0.0)
        self._sport.switch_joystick(True)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
