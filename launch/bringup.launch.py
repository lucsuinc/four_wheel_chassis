import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('four_wheel_chassis')

    can_interface = LaunchConfiguration('can_interface')
    imu_port = LaunchConfiguration('imu_port')
    imu_frame_id = LaunchConfiguration('imu_frame_id')
    lidar_port = LaunchConfiguration('lidar_port')

    # 机器人模型（base_footprint / base_link / imu_link / laser_link 的静态 TF）
    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(pkg_share, 'urdf', 'ylhb_chassis.urdf.xacro')]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'can_interface',
            default_value='can0',
            description='CAN 接口名（USB-CAN 通常是 can0）'),
        DeclareLaunchArgument(
            'imu_port',
            default_value='/dev/robot_imu',
            description='N300Pro IMU 串口（先运行 scripts/bind_usb.sh）'),
        DeclareLaunchArgument(
            'imu_frame_id',
            default_value='imu_link',
            description='IMU 输出坐标系，需和 URDF 一致'),
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/robot_lidar',
            description='RPLIDAR 串口（先运行 scripts/bind_usb.sh）'),

        # 1. 机器人模型 + 静态关节 TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # 2. 底盘控制：/cmd_vel -> CAN -> /odom
        #    publish_tf=false：odom -> base_footprint 交给 EKF 统一发布，
        #    避免和 EKF 抢 TF。
        Node(
            package='four_wheel_chassis',
            executable='chassis_control_node',
            parameters=[
                os.path.join(pkg_share, 'config', 'chassis.yaml'),
                {'can_interface': can_interface},
                {'publish_tf': False},
            ],
            output='screen',
        ),

        # 3. N300Pro IMU（HiPNUC 协议）：/imu/data
        Node(
            package='four_wheel_chassis',
            executable='imu_driver',
            parameters=[
                os.path.join(pkg_share, 'config', 'imu.yaml'),
                {'serial_port': imu_port},
                {'frame_id': imu_frame_id},
            ],
            output='screen',
        ),

        # 4. RPLIDAR：/scan
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': lidar_port,
                'serial_baudrate': 115200,
                'frame_id': 'laser_link',
                'inverted': False,
                'angle_compensate': True,
            }],
            output='screen',
        ),

        # 5. robot_localization EKF：融合 /odom + /imu/data
        #    发布 odom -> base_footprint TF，供 slam_toolbox / Nav2 使用
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml')],
            output='screen',
        ),
    ])
