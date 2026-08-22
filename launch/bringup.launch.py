import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('four_wheel_chassis')

    can_interface = LaunchConfiguration('can_interface')

    robot_description = Command(
        ['xacro ', os.path.join(pkg_share, 'urdf', 'ylhb_chassis.urdf.xacro')]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'can_interface',
            default_value='can0',
            description='CAN 接口名（USB-CAN 通常是 can0）'),

        # 发布机器人模型和 base_footprint/base_link/imu_link/laser_link 静态 TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # 底盘控制：/cmd_vel -> CAN -> /odom + TF
        Node(
            package='four_wheel_chassis',
            executable='chassis_control_node',
            parameters=[
                os.path.join(pkg_share, 'config', 'chassis.yaml'),
                {'can_interface': can_interface},
            ],
            output='screen',
        ),
    ])
