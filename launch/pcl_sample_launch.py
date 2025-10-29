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
                    shrd_pkg_prefix + '/launch' + '/tof_launch.py'
                )
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', shrd_pkg_prefix + '/config/rviz/scm_tof.rviz'],
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
            Node(
                package='cis_scm',
                executable='pcl_sample_node',
                name='pcl_sample',
                parameters=[
                    {'cloud_topic': '/cis_scm/depth/points'},
                    {'world_frame': 'map'},
                    {'camera_frame': 'camera_link'},
                    {'voxel_leaf_size': 0.02},  # mm
                    {'x_filter_min': -2.5},  # mm
                    {'x_filter_max': 2.5},  # mm
                    {'y_filter_min': -2.5},  # mm
                    {'y_filter_max': 2.5},  # mm
                    {'z_filter_min': -2.5},  # mm
                    {'z_filter_max': 2.5},  # mm
                    {'plane_max_iterations': 50},
                    {'plane_distance_threshold': 0.05},
                ],
            ),
        ]
    )
