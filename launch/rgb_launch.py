import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('cis_scm'), 'config', 'params.yaml'
    )

    return LaunchDescription([
        Node(
            package="cis_scm",
            namespace="cis_scm",
            executable="rgb_node",
            name="rgb_node",
            parameters=[{"camera_params": "package://cis_scm/cam_param.yaml"}, config]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_base_to_camera",
            arguments=['--z', '0.03', '--frame-id', 'camera_base', '--child-frame-id' ,'camera']
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_to_cam_depth",
            arguments=['--x', '0.001', '--yaw', '-1.57', '--roll', '-1.57', '--frame-id', 'camera', '--child-frame-id', 'cam_depth']
        )
    ])
