# Copyright (c) 2023 Takumi Asada

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'ros2_topic_sample'
    rviz_file_name = "topic_view.rviz"

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(package_name), "rviz", rviz_file_name]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    diagnostic_node = Node(
        package=package_name,
        executable='diagnostic_pub',
        output="screen",
    )

    float32_node = Node(
        package=package_name,
        executable='float32_pub',
        output="screen",
    )

    illuminance_node = Node(
        package=package_name,
        executable='illuminance_pub',
        output="screen",
    )

    polygon_node = Node(
        package=package_name,
        executable='polygon_pub',
        output="screen",
    )

    posestamped_node = Node(
        package=package_name,
        executable='posestamped_pub',
        output="screen",
    )

    range_node = Node(
        package=package_name,
        executable='range_pub',
        output="screen",
    )

    string_node = Node(
        package=package_name,
        executable='string_pub',
        output="screen",
    )

    twiststamped_node = Node(
        package=package_name,
        executable='twiststamped_pub',
        output="screen",
    )

    nodes = [
        rviz_node,
        diagnostic_node,
        float32_node,
        illuminance_node,
        polygon_node,
        posestamped_node,
        range_node,
        string_node,
        twiststamped_node,
    ]

    return LaunchDescription(nodes)
