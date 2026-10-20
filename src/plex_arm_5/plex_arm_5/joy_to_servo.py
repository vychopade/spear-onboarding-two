import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy


class JoyToServo(Node):
    def __init__(self):
        super().__init__('joy_to_servo')
        self.deadzone = 0.1
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.linear_z = 0.0
        self.last_joy_time = None
        self.started = self.get_clock().now()
        self.rest = None
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.publisher = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.create_timer(0.05, self.publish_command)

    def axis(self, msg, index):
        if index >= len(msg.axes):
            return 0.0
        value = float(msg.axes[index])
        if abs(value) < self.deadzone:
            return 0.0
        return max(-1.0, min(1.0, value))

    def axes_signature(self, msg):
        values = []
        for index in range(6):
            if index < len(msg.axes):
                values.append(round(float(msg.axes[index]), 2))
            else:
                values.append(0.0)
        return tuple(values)

    def joy_callback(self, msg):
        now = self.get_clock().now()
        age = (now - self.started).nanoseconds * 1e-9
        signature = self.axes_signature(msg)
        if age < 1.0:
            if self.rest is None:
                self.rest = signature
            elif signature != self.rest:
                self.rest = None

        if self.rest is not None and signature == self.rest:
            return
        self.linear_x = self.axis(msg, 1)
        self.linear_y = -self.axis(msg, 0)
        z_index = 4 if len(msg.axes) > 4 else 3
        self.linear_z = self.axis(msg, z_index)
        self.last_joy_time = now

    def publish_command(self):
        if self.last_joy_time is None:
            return
        age = (self.get_clock().now() - self.last_joy_time).nanoseconds * 1e-9
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'base_link'
        if age > 0.15:
            command.twist.linear.x = 0.0
            command.twist.linear.y = 0.0
            command.twist.linear.z = 0.0
        else:
            command.twist.linear.x = self.linear_x
            command.twist.linear.y = self.linear_y
            command.twist.linear.z = self.linear_z
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
