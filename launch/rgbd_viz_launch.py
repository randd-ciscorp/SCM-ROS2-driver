from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory('cis_scm') + '/launch' + '/rgbd_launch.py'
            )
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            #arguments=['-d', rviz_config]
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_camera_base",
            arguments=['--z', '1.0', '--frame-id', 'map', '--child-frame-id', 'camera_base']
        )
    ])