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
  config_filename = LaunchConfiguration("config_filename")

  config_filepath = PathJoinSubstitution(
      [FindPackageShare("kinesis_video_streamer"), "config", config_filename]
  )
  logger_config_path = PathJoinSubstitution(
      [FindPackageShare("kinesis_video_streamer"), "config", "kvs_log_configuration"]
  )

  streamer_node = Node(
    package="kinesis_video_streamer",
    executable="kinesis_video_streamer",
    name=node_name,
    parameters=[config_filepath,
                {'kinesis_video': {
                  'log4cplus_config': logger_config_path
                }}]
  )

  output_log_actions = [LogInfo(msg=config_filepath)]
  return output_log_actions + [streamer_node]


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
          "config_filename",
          default_value="color.yaml",
      )
  )

  return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
