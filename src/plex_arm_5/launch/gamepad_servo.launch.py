import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    package_share = get_package_share_directory('plex_arm_5')

    rviz_config = os.path.join(package_share, 'config', 'moveit.rviz')
    ros2_controllers_path = os.path.join(package_share, 'config', 'ros2_controllers.yaml')

    moveit_config = MoveItConfigsBuilder(
        'plex_arm_5', package_name='plex_arm_5').to_moveit_configs()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description],
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[moveit_config.robot_description, ros2_controllers_path],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'plex_arm_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'move_group.launch.py')
        )
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    joy_linux = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_linux_node',
        output='screen',
        respawn=True,
        respawn_delay=1.0,
        parameters=[{
            'dev': LaunchConfiguration('dev'),
            'dev_name': 'Nintendo Switch Pro Controller',
            'dev_ff': '',
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
    )

    arm_teleop = Node(
        package='plex_arm_5',
        executable='arm_teleop',
        name='arm_teleop',
        output='screen',
        parameters=[{
            'speed': ParameterValue(LaunchConfiguration('speed'), value_type=float),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('dev', default_value='/dev/input/js0'),
        DeclareLaunchArgument('speed', default_value='0.6'),
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        move_group,
        rviz_node,
        joy_linux,
        arm_teleop,
    ])
