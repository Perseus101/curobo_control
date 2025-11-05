#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class JoyCmdVelNode(Node):
    VEL = 0.04
    ROT_VEL = 0.5
    DEADZONE = 0.01

    def __init__(self) -> None:
        super().__init__("joy_cmd_vel_node")
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.subscription = self.create_subscription(
            Joy,
            "/joy",
            self.joy_callback,
            10,
        )
        self._axes = [0.0, 0.0, 0.0, 0.0]
        self._publish_timer = self.create_timer(0.2, self._publish_cmd_vel)

    def joy_callback(self, msg: Joy) -> None:
        self._axes[0] = self.VEL * float(msg.axes[0])
        self._axes[1] = self.VEL * float(msg.axes[1])
        self._axes[2] = self.VEL * float(msg.axes[3])
        self._axes[3] = self.ROT_VEL * (float(msg.buttons[-3]) - float(msg.buttons[-2]))

    def _publish_cmd_vel(self) -> None:
        if all([abs(ax) < self.DEADZONE for ax in self._axes]):
            return

        twist = Twist()
        twist.linear.x, twist.linear.y, twist.linear.z = self._axes[:3]
        twist.angular.x = self._axes[3]

        self.publisher_.publish(twist)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JoyCmdVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
