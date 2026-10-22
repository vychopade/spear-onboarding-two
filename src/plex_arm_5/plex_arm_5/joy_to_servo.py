import rclpy
from rclpy.node import Node

from control_msgs.msg import JointJog
from sensor_msgs.msg import Joy

# Servo can move the arm two ways. A Twist asks it to run inverse kinematics
# and chase the end effector in 3D. This arm's spawn orientation makes that
# IK singular, so the gamepad sends a JointJog and Servo moves each joint
# itself. Velocities are unitless, in [-1, 1]; servo_params.yaml scales them.

# joy_node: stick up is positive, stick left is positive.
LEFT_X = 0
LEFT_Y = 1
RIGHT_X = 2
RIGHT_Y = 3
BUTTON_L = 5
BUTTON_R = 6
BUTTON_ZL = 7
BUTTON_ZR = 8

JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

# Left stick jogs joints 1 and 2, right stick jogs joints 3 and 4.
# L/R jog joint 5, ZL/ZR jog joint 6.
COMMAND_TOPIC = '/servo_node/delta_joint_cmds'
QUIET_TIMEOUT = 0.15


class JoyToServo(Node):
    def __init__(self):
        super().__init__('joy_to_servo')
        self.deadzone = 0.2
        self.last_joy_time = None
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.publisher = self.create_publisher(JointJog, COMMAND_TOPIC, 10)
        # Live commands come from the Joy callback. This only sends zeros
        # after the pad goes quiet, so the arm stops even if a message is lost.
        self.create_timer(0.02, self.publish_halt_if_quiet)

    def raw_axis(self, msg, index):
        if index >= len(msg.axes):
            return 0.0
        return max(-1.0, min(1.0, float(msg.axes[index])))

    def shaped(self, value):
        # Deadzone around zero, then stretch the rest of the travel to [-1, 1].
        if abs(value) < self.deadzone:
            return 0.0
        sign = 1.0 if value > 0.0 else -1.0
        scaled = (abs(value) - self.deadzone) / (1.0 - self.deadzone)
        return max(-1.0, min(1.0, sign * scaled))

    def pressed(self, msg, index):
        if index >= len(msg.buttons):
            return False
        return int(msg.buttons[index]) != 0

    def publish_command(self, joy):
        # One JointJog per Joy message: create it, map the pad onto it, publish.
        command = JointJog()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'base_link'
        command.joint_names = list(JOINTS)
        command.velocities = [
            -self.shaped(self.raw_axis(joy, LEFT_X)),
            self.shaped(self.raw_axis(joy, LEFT_Y)),
            -self.shaped(self.raw_axis(joy, RIGHT_X)),
            self.shaped(self.raw_axis(joy, RIGHT_Y)),
            0.0,
            0.0,
        ]
        if self.pressed(joy, BUTTON_L):
            command.velocities[4] -= 1.0
        if self.pressed(joy, BUTTON_R):
            command.velocities[4] += 1.0
        if self.pressed(joy, BUTTON_ZL):
            command.velocities[5] -= 1.0
        if self.pressed(joy, BUTTON_ZR):
            command.velocities[5] += 1.0
        command.velocities = [
            max(-1.0, min(1.0, value)) for value in command.velocities
        ]
        self.publisher.publish(command)

    def joy_callback(self, msg):
        self.last_joy_time = self.get_clock().now()
        self.publish_command(msg)

    def publish_halt_if_quiet(self):
        if self.last_joy_time is None:
            return
        age = (self.get_clock().now() - self.last_joy_time).nanoseconds * 1e-9
        if age > QUIET_TIMEOUT:
            self.publish_command(Joy())


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
