#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    
    turtle_ns = LaunchConfiguration('', 
        default='')
    
    logger_ns = LaunchConfiguration('',
        default='')

    evader_ns = LaunchConfiguration('',
        default='')

    turtle_node = Node(
        package='unmanned_systems_ros2_pkg',
        namespace=turtle_ns,
        executable='pn.py'
        )
    
    logger_node = Node(
        package='unmanned_systems_ros2_pkg',
        namespace=logger_ns,
        executable='logger.py'
        )

    evader_node = Node(
        package='unmanned_systems_ros2_pkg',
        namespace=evader_ns,
        executable='turtlebot_Astar.py'
        )


    launch_description = LaunchDescription(
        [
        logger_node,
        turtle_node,
        evader_node
        ]
    )

    return launch_description
