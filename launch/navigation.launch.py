import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('four_wheel_chassis')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    # 一条命令启动全部：底盘 + IMU + 雷达 + EKF + Nav2（含 AMCL 定位）
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'bringup.launch.py')))

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'map': map_file,
            'params_file': params_file,
            # 行为树路径由 nav2_bringup 默认提供，覆盖我们 YAML 里的空值
            'default_bt_xml_filename': os.path.join(
                nav2_bringup_share,
                'behavior_trees',
                'navigate_to_pose_w_recovery_and_rerouting.xml'),
        }.items())

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(os.path.expanduser('~'), 'ros2_ws', 'maps', 'my_map.yaml'),
            description='建图完成后保存的地图 yaml 路径'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
            description='Nav2 参数文件'),

        bringup,
        nav2,
    ])
