from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='cis_scm',
                namespace='cis_scm',
                executable='rgbd_node',
                name='rgbd_node',
                parameters=[{'camera_params': 'package://cis_scm/cam_param.yaml'}],
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='camera_base_to_camera',
                arguments=[
                    '--z',
                    '0.03',
                    '--frame-id',
                    'camera_base',
                    '--child-frame-id',
                    'camera',
                ],
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='camera_to_cam_depth',
                arguments=[
                    '--x',
                    '0.001',
                    '--yaw',
                    '-1.57',
                    '--roll',
                    '-1.57',
                    '--frame-id',
                    'camera',
                    '--child-frame-id',
                    'cam_depth',
                ],
            ),
        ]
    )
