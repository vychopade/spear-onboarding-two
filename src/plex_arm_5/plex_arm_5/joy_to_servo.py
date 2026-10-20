import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy


class JoyToServo(Node):
    def __init__(self):
        super().__init__('joy_to_servo')
        self.deadzone = 0.2
        self.centers = [0.0, 0.0, 0.0, 0.0]
        self.rest = None
        self.rest_since = None
        self.ready = False
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.linear_z = 0.0
        self.last_joy_time = None
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.publisher = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.create_timer(0.02, self.publish_command)

    def raw_axis(self, msg, index):
        if index >= len(msg.axes):
            return 0.0
        return max(-1.0, min(1.0, float(msg.axes[index])))

    def shaped(self, reading, index):
        value = reading[index] - self.centers[index]
        if abs(value) < self.deadzone:
            return 0.0
        sign = 1.0 if value > 0.0 else -1.0
        scaled = (abs(value) - self.deadzone) / (1.0 - self.deadzone)
        return max(-1.0, min(1.0, sign * scaled))

    def joy_callback(self, msg):
        now = self.get_clock().now()
        reading = [self.raw_axis(msg, index) for index in range(4)]
        if not self.ready:
            moved = self.rest is None or any(
                abs(reading[index] - self.rest[index]) > 0.1
                for index in (0, 1, 3))
            if moved:
                self.rest = reading
                self.rest_since = now
            elif (now - self.rest_since).nanoseconds * 1e-9 > 0.5:
                if all(abs(reading[index]) <= 0.5 for index in (0, 1, 3)):
                    self.centers = list(reading)
                    self.ready = True
                    self.get_logger().info('joystick centered')
                else:
                    self.rest_since = now
            self.linear_x = 0.0
            self.linear_y = 0.0
            self.linear_z = 0.0
        else:
            # joy_node: stick up is positive, stick left is positive.
            self.linear_x = self.shaped(reading, 1)
            self.linear_y = -self.shaped(reading, 0)
            self.linear_z = self.shaped(reading, 3)
        self.last_joy_time = now

    def publish_command(self):
        if self.last_joy_time is None:
            return
        age = (self.get_clock().now() - self.last_joy_time).nanoseconds * 1e-9
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'base_link'
        if age > 0.15 or not self.ready:
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
