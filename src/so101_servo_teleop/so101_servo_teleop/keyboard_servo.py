#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType  # ← Jazzy service
import threading
import sys
import tty
import termios

KEY_BINDINGS = {
    'w': ('linear',  0,  1.0),
    's': ('linear',  0, -1.0),
    'a': ('linear',  1,  1.0),
    'd': ('linear',  1, -1.0),
    'q': ('linear',  2,  1.0),
    'e': ('linear',  2, -1.0),
    'u': ('angular', 0,  1.0),
    'j': ('angular', 0, -1.0),
    'i': ('angular', 1,  1.0),
    'k': ('angular', 1, -1.0),
    'o': ('angular', 2,  1.0),
    'l': ('angular', 2, -1.0),
}

LINEAR_SPEED  = 0.03
ANGULAR_SPEED = 0.1
PUBLISH_HZ    = 20

class CartesianKeyboardServo(Node):
    def __init__(self):
        super().__init__('cartesian_keyboard_servo')

        self._lock  = threading.Lock()
        self._twist = [0.0] * 6
        self._last_key_time = 0.0

        self.pub = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            10
        )

        self.create_timer(1.0 / PUBLISH_HZ, self._publish_cb)

        # ── Jazzy: switch to TWIST (Cartesian) mode ──────────
        self._switch_command_type()

        self._print_help()

    def _switch_command_type(self):
        cli = self.create_client(ServoCommandType, '/servo_node/switch_command_type')
        self.get_logger().info('Waiting for /servo_node/switch_command_type ...')

        if not cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('switch_command_type service not found!')
            return

        req = ServoCommandType.Request()
        req.command_type = ServoCommandType.Request.TWIST  # = 1, Cartesian mode

        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() and future.result().success:
            self.get_logger().info('Switched to TWIST (Cartesian) mode ✓')
        else:
            self.get_logger().warn('switch_command_type failed — check servo status')

    def _publish_cb(self):
        import time
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base'  # must match robot_link_command_frame in servo.yaml

        with self._lock:
            if time.time() - self._last_key_time > 0.15:
                self._twist = [0.0] * 6
            msg.twist.linear.x  = self._twist[0]
            msg.twist.linear.y  = self._twist[1]
            msg.twist.linear.z  = self._twist[2]
            msg.twist.angular.x = self._twist[3]
            msg.twist.angular.y = self._twist[4]
            msg.twist.angular.z = self._twist[5]

        self.pub.publish(msg)

    def _keyboard_thread(self):
        import time
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while rclpy.ok():
                key = sys.stdin.read(1)
                if key == '\x03':   # CTRL+C
                    break
                key = key.lower()
                with self._lock:
                    if key in KEY_BINDINGS:
                        kind, axis, direction = KEY_BINDINGS[key]
                        self._twist = [0.0] * 6
                        if kind == 'linear':
                            self._twist[axis] = direction * LINEAR_SPEED
                        else:
                            self._twist[3 + axis] = direction * ANGULAR_SPEED
                        self._last_key_time = time.time()
                    else:
                        self._twist = [0.0] * 6
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def _print_help(self):
        print("""
  ╔═══════════════════════════════════════════════╗
  ║    SO101 Cartesian Keyboard Control           ║
  ║    EE: gripper  │  Frame: base_link           ║
  ╠═══════════════════════════════════════════════╣
  ║  TRANSLATION          ROTATION                ║
  ║  W / S  →  ±X fwd     U / J  →  ±Roll        ║
  ║  A / D  →  ±Y left    I / K  →  ±Pitch       ║
  ║  Q / E  →  ±Z up      O / L  →  ±Yaw         ║
  ╠═══════════════════════════════════════════════╣
  ║  Hold key = move  │  Release = stop           ║
  ║  CTRL+C  = quit                               ║
  ╚═══════════════════════════════════════════════╝
        """)

    def run(self):
        kb = threading.Thread(target=self._keyboard_thread, daemon=True)
        kb.start()
        rclpy.spin(self)


def main():
    rclpy.init()
    node = CartesianKeyboardServo()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
