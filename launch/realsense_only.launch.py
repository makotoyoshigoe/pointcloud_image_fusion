from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # パッケージ共有ディレクトリ取得
    fusion_pkg = get_package_share_directory('pointcloud_image_fusion')
    rs_pkg = get_package_share_directory('realsense2_camera')

    # RViz設定ファイルパス
    rviz_config = os.path.join(fusion_pkg, 'rviz', 'realsense_only.rviz')

    return LaunchDescription([
        # Group: RealSenseデバイス起動
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rs_pkg, 'launch', 'rs_launch.py')
                ),
                launch_arguments={
                    'depth.enable': 'true',
                    'color.enable': 'true',
                    'enable_depth': 'true', 
                    'pointcloud.enable': 'true',
                    'pointcloud.ordered_pc': 'false', 
                }.items()
            )
        ]),

        # Group: lc_fusionノード
        GroupAction([
            Node(
                package='pointcloud_image_fusion',
                executable='lc_fusion',
                name='lc_fusion_node',
                output='screen',
                arguments=['--pc-type', 'xyz', '--ros-args', '--log-level', 'warn'],
                remappings=[
                    ('/livox/lidar', '/camera/camera/depth/color/points')
                ],
                parameters=[{
                    'remove_outrange': True, 
                    'leaf_size': 0.025
                }],
                additional_env={
                    'RCUTILS_LOGGING_BUFFERED_STREAM': '1'
                }
            )
        ]),

        # Group: RViz2
        GroupAction([
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config, '--ros-args', '--log-level', 'warn'],
            )
        ])
    ])
