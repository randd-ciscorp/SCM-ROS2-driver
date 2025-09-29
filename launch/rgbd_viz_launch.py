from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    shrd_pkg_prefix = get_package_share_directory('cis_scm')

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    shrd_pkg_prefix + '/launch' + '/rgbd_launch.py'
                )
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', shrd_pkg_prefix + '/config/rviz/scm_rgbd.rviz'],
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='map_to_camera_base',
                arguments=[
                    '--z',
                    '1.0',
                    '--frame-id',
                    'map',
                    '--child-frame-id',
                    'camera_base',
                ],
            ),
        ]
    )
