#!/usr/bin/env python3
#
# Copyright 2023 REMAKE.AI
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

import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_nodes(context: LaunchContext, robot_model, map, use_sim_time, slam):
    robot_model_str = context.perform_substitution(robot_model)
    map_path_str = context.perform_substitution(map)
    use_sim_time_str = context.perform_substitution(use_sim_time)
    slam_str = context.perform_substitution(slam)
    description_package_path = get_package_share_path(robot_model_str)

    rviz_config_path = os.path.join(
        description_package_path,
        'rviz',
        'navigation.rviz')

    nav_config_path = os.path.join(
        description_package_path,
        'config',
        'navigation.yaml')

    print('Rviz2 config : {}'.format(rviz_config_path))
    print('Nav2  config : {}'.format(nav_config_path))
    print('Map          : {}'.format(map_path_str))

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_path('nav2_bringup'), 'launch'),
                '/bringup_launch.py'
            ]),
            launch_arguments={
                'map': map_path_str,
                'use_sim_time': use_sim_time_str,
                'slam': slam_str,
                'params_file': nav_config_path}.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': use_sim_time_str.lower() == 'true'}],
        )
    ]

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            name='robot_model',
            default_value='makerspet_snoopy',
            description='Robot description package name'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
                get_package_share_path('kaiaai_gazebo'),
                'map',
                'living_room.yaml'),
            description='Full path to an existing map file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'slam',
            default_value='False',
            choices=['True', 'False'],
            description='Navigate while creating a new map'
        ),
        OpaqueFunction(function=make_nodes, args=[
            LaunchConfiguration('robot_model'),
            LaunchConfiguration('map'),
            LaunchConfiguration('use_sim_time'),
            LaunchConfiguration('slam'),
        ]),
    ])
