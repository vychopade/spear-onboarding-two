import rclpy
from rclpy.node import Node

from control_msgs.msg import JointJog
from sensor_msgs.msg import Joy

# joy_node: stick up is positive, stick left is positive.
# Same joint map as the previous gamepad teleop: sticks jog the first four
# joints, L/R jog joint_5, and ZL/ZR jog joint_6.
LEFT_X = 0
LEFT_Y = 1
RIGHT_X = 2
RIGHT_Y = 3
BUTTON_L = 5
BUTTON_R = 6
BUTTON_ZL = 7
BUTTON_ZR = 8

JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

COMMAND_TOPIC = '/servo_node/delta_joint_cmds'
QUIET_TIMEOUT = 0.15


class JoyToServo(Node):
    def __init__(self):
        super().__init__('joy_to_servo')
        self.deadzone = 0.2
        self.last_joy_time = None
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.publisher = self.create_publisher(JointJog, COMMAND_TOPIC, 10)
        # Watchdog only. Live commands are published from the joy callback.
        self.create_timer(0.02, self.publish_halt_if_quiet)

    def raw_axis(self, msg, index):
        if index >= len(msg.axes):
            return 0.0
        return max(-1.0, min(1.0, float(msg.axes[index])))

    def shaped(self, value):
        """Deadzone around zero, then rescale the rest of the stick to [-1, 1]."""
        if abs(value) < self.deadzone:
            return 0.0
        sign = 1.0 if value > 0.0 else -1.0
        scaled = (abs(value) - self.deadzone) / (1.0 - self.deadzone)
        return max(-1.0, min(1.0, sign * scaled))

    def pressed(self, msg, index):
        if index >= len(msg.buttons):
            return False
        return int(msg.buttons[index]) != 0

    def joint_velocities(self, msg):
        velocities = [0.0] * len(JOINTS)
        velocities[0] = -self.shaped(self.raw_axis(msg, LEFT_X))
        velocities[1] = self.shaped(self.raw_axis(msg, LEFT_Y))
        velocities[2] = -self.shaped(self.raw_axis(msg, RIGHT_X))
        velocities[3] = self.shaped(self.raw_axis(msg, RIGHT_Y))
        if self.pressed(msg, BUTTON_L):
            velocities[4] -= 1.0
        if self.pressed(msg, BUTTON_R):
            velocities[4] += 1.0
        if self.pressed(msg, BUTTON_ZL):
            velocities[5] -= 1.0
        if self.pressed(msg, BUTTON_ZR):
            velocities[5] += 1.0
        return [max(-1.0, min(1.0, value)) for value in velocities]

    def publish_velocities(self, velocities):
        message = JointJog()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        message.joint_names = list(JOINTS)
        message.velocities = [float(value) for value in velocities]
        self.publisher.publish(message)

    def joy_callback(self, msg):
        self.last_joy_time = self.get_clock().now()
        self.publish_velocities(self.joint_velocities(msg))

    def publish_halt_if_quiet(self):
        if self.last_joy_time is None:
            return
        age = (self.get_clock().now() - self.last_joy_time).nanoseconds * 1e-9
        if age > QUIET_TIMEOUT:
            self.publish_velocities([0.0] * len(JOINTS))


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
