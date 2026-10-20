import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState, Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Linux joydev order for a Switch Pro Controller using hid-nintendo.
# Left stick up and right stick up are negative on axes 1 and 3.
LEFT_X = 0
LEFT_Y = 1
RIGHT_X = 2
RIGHT_Y = 3
BUTTON_L = 5
BUTTON_R = 6
BUTTON_ZL = 7
BUTTON_ZR = 8

JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']


class ArmTeleop(Node):
    def __init__(self):
        super().__init__('arm_teleop')
        self.declare_parameter('speed', 0.6)
        self.deadzone = 0.15
        self.axes = []
        self.buttons = []
        self.last_joy = None
        self.armed = False
        self.measured = None
        self.goal = None
        self.moving = False
        self.create_subscription(Joy, '/joy', self.on_joy, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.publisher = self.create_publisher(
            JointTrajectory, '/plex_arm_controller/joint_trajectory', 10)
        self.create_timer(0.05, self.tick)

    def on_joy(self, msg):
        self.axes = list(msg.axes)
        self.buttons = list(msg.buttons)
        self.last_joy = self.get_clock().now()
        if not self.armed and self.neutral():
            self.armed = True

    def on_joints(self, msg):
        positions = {}
        for name, position in zip(msg.name, msg.position):
            positions[name] = float(position)
        if all(name in positions for name in JOINTS):
            self.measured = [positions[name] for name in JOINTS]

    def raw_axis(self, index):
        if index >= len(self.axes):
            return 0.0
        return float(self.axes[index])

    def axis(self, index):
        value = self.raw_axis(index)
        if abs(value) < self.deadzone:
            return 0.0
        return max(-1.0, min(1.0, value))

    def pressed(self, index):
        if index >= len(self.buttons):
            return False
        return int(self.buttons[index]) != 0

    def neutral(self):
        for index in (LEFT_X, LEFT_Y, RIGHT_X, RIGHT_Y):
            if abs(self.raw_axis(index)) >= self.deadzone:
                return False
        for index in (BUTTON_L, BUTTON_R, BUTTON_ZL, BUTTON_ZR):
            if self.pressed(index):
                return False
        return True

    def rates(self):
        speed = float(self.get_parameter('speed').value)
        joint_rates = [0.0] * len(JOINTS)
        joint_rates[0] = self.axis(LEFT_X) * speed
        joint_rates[1] = -self.axis(LEFT_Y) * speed
        joint_rates[2] = self.axis(RIGHT_X) * speed
        joint_rates[3] = -self.axis(RIGHT_Y) * speed
        if self.pressed(BUTTON_L):
            joint_rates[4] -= speed
        if self.pressed(BUTTON_R):
            joint_rates[4] += speed
        if self.pressed(BUTTON_ZL):
            joint_rates[5] -= speed
        if self.pressed(BUTTON_ZR):
            joint_rates[5] += speed
        return joint_rates

    def publish_positions(self, positions):
        message = JointTrajectory()
        message.header.stamp = self.get_clock().now().to_msg()
        message.joint_names = list(JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        point.time_from_start.nanosec = 100000000
        message.points = [point]
        self.publisher.publish(message)

    def hold(self):
        if self.moving and self.goal is not None:
            self.publish_positions(self.goal)
        self.moving = False
        self.goal = None

    def tick(self):
        if self.measured is None or not self.armed or self.last_joy is None:
            return
        age = (self.get_clock().now() - self.last_joy).nanoseconds * 1e-9
        if age > 0.25:
            self.hold()
            return
        joint_rates = self.rates()
        if all(abs(rate) < 1e-4 for rate in joint_rates):
            self.hold()
            return
        if self.goal is None:
            self.goal = list(self.measured)
        for index, rate in enumerate(joint_rates):
            self.goal[index] += rate * 0.05
        self.publish_positions(self.goal)
        self.moving = True


def main(args=None):
    rclpy.init(args=args)
    node = ArmTeleop()
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
