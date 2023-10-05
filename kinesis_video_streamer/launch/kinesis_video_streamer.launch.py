# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License").
# You may not use this file except in compliance with the License.
# A copy of the License is located at
#
#  http://aws.amazon.com/apache2.0
#
# or in the "license" file accompanying this file. This file is distributed
# on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
# express or implied. See the License for the specific language governing
# permissions and limitations under the License.

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.actions import OpaqueFunction
from launch.actions import LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    node_name = LaunchConfiguration("node_name")
    config_file_path = LaunchConfiguration("config_file_path")
    log4cplus_config_file_path = LaunchConfiguration("log4cplus_config_file_path")

    if context.perform_substitution(config_file_path) == "":
        config_file_path = PathJoinSubstitution(
            [FindPackageShare("kinesis_video_streamer"), "config", "sample.yaml"]
        )
    if context.perform_substitution(log4cplus_config_file_path) == "":
        log4cplus_config_file_path = PathJoinSubstitution(
            [FindPackageShare("kinesis_video_streamer"), "config", "log4cplus.ini"]
        )

    node = Node(
        package="kinesis_video_streamer",
        executable="kinesis_video_streamer",
        name=node_name,
        parameters=[
            config_file_path,
            {"kinesis_video": {"log4cplus_config": log4cplus_config_file_path}},
        ],
    )

    output_log_actions = []
    output_log_actions.append(LogInfo(msg=[config_file_path]))
    output_log_actions.append(LogInfo(msg=[log4cplus_config_file_path]))
    return output_log_actions + [node]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "node_name",
            default_value="kinesis_color_video_streamer",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "config_file_path",
            default_value="",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "log4cplus_config_file_path",
            default_value="",
        )
    )
    
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
