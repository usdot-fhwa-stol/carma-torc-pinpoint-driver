# Copyright (C) 2024 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from carma_ros2_utils.launch.get_current_namespace import GetCurrentNamespace

import os

"""
This file is can be used to launch the pinpoint driver.
"""


def generate_launch_description():

    # Declare the log_level launch argument
    log_level = LaunchConfiguration("log_level")
    declare_log_level_arg = DeclareLaunchArgument(
        name="log_level", default_value="WARN"
    )

    pinpoint_param_file = os.path.join(
        get_package_share_directory("pinpoint"), "config/parameters.yaml"
    )

    # Args for driver
    address = LaunchConfiguration("address")
    declare_address = DeclareLaunchArgument(
        name="address",
        default_value="192.168.88.29",
        description="Pinpoint device ip address",
    )

    port = LaunchConfiguration("port")
    declare_port = DeclareLaunchArgument(
        name="port",
        default_value="9501",
        description="The localization port the pinpoint is sending data to",
    )

    odom_frame = LaunchConfiguration("odom_frame")
    declare_odom_frame = DeclareLaunchArgument(
        name="odom_frame",
        default_value="odom",
        description="The name of the local planning frame",
    )

    base_link_frame = LaunchConfiguration("base_link_frame")
    declare_base_link_frame = DeclareLaunchArgument(
        name="base_link_frame",
        default_value="base_link",
        description="The name of the robot base frame",
    )

    sensor_frame = LaunchConfiguration("sensor_frame")
    declare_sensor_frame = DeclareLaunchArgument(
        name="sensor_frame",
        default_value="pinpoint",
        description="The name of the pinpoint device frame",
    )

    remap_ns = LaunchConfiguration("remap_ns")
    declare_remap_ns = DeclareLaunchArgument(name="remap_ns", default_value="/")

    # Launch node(s) in a carma container to allow logging to be configured
    container = ComposableNodeContainer(
        package="carma_ros2_utils",
        name="pinpoint_container",
        namespace=remap_ns,
        executable="carma_component_container_mt",
        composable_node_descriptions=[
            # Launch the core node(s)
            ComposableNode(
                package="pinpoint",
                plugin="pinpoint::PinPointApplication",
                name="pinpoint",
                extra_arguments=[
                    {"use_intra_process_comms": True},
                    {"--log-level": log_level},
                ],
                parameters=[
                    pinpoint_param_file,
                    {'address' : address},
                    {'port' : port},
                    {'odom_frame' : odom_frame},
                    {'base_link_frame' : base_link_frame},
                    {'sensor_frame' : sensor_frame},
                ],
            ),
        ],
    )

    return LaunchDescription(
        [
            declare_log_level_arg,
            declare_address,
            declare_port,
            declare_odom_frame,
            declare_base_link_frame,
            declare_sensor_frame,
            declare_remap_ns,
            container,
        ]
    )
