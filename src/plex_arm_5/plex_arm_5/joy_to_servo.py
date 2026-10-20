import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy


class JoyToServo(Node):
    def __init__(self):
        super().__init__('joy_to_servo')
        self.deadzone = 0.1
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.publisher = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10)

    def axis(self, msg, index):
        if index >= len(msg.axes):
            return 0.0
        value = float(msg.axes[index])
        if abs(value) < self.deadzone:
            return 0.0
        return max(-1.0, min(1.0, value))

    def joy_callback(self, msg):
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'base_link'
        command.twist.linear.x = self.axis(msg, 1)
        command.twist.linear.y = -self.axis(msg, 0)
        z_index = 4 if len(msg.axes) > 4 else 3
        command.twist.linear.z = self.axis(msg, z_index)
        self.publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = JoyToServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
