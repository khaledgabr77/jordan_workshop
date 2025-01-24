#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    cpp_publisher_node = Node(
        package="py_pub_sub_pkg",
        executable="py_param",
        name="my_publisher_param",
        parameters=[{"published_text": "Parameter_from_launch"}]
    )


    ld.add_action(cpp_publisher_node)
    return ld