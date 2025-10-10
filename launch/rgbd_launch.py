# Copyright 2025 CIS Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
