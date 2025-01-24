#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    cpp_publisher_node = Node(
        package="cpp_pub_sub_pkg",
        executable="publisher_hello_ros",
        name="my_publisher_cpp",
        remappings=[
            ("chatter", "my_topic")
        ]
    )

    cpp_subscriber_node = Node(
        package="cpp_pub_sub_pkg",
        executable="subscriber_hello_ros",
        name="my_subscriber_cpp",
        remappings=[
            ("chatter", "another_topic")
            ]
    )

    py_publisher_node = Node(
        package="py_pub_sub_pkg",
        executable="publisher_hello_ros",
        name="my_publisher_py",
        remappings=[
            ("chatter", "another_topic")
            ]
    
    )

    py_subscriber_node = Node(
        package="cpp_pub_sub_pkg",
        executable="subscriber_hello_ros",
        name="my_subscriber_py",
        remappings=[
            ("chatter", "my_topic")
        ]
    )

    ld.add_action(cpp_publisher_node)
    ld.add_action(cpp_subscriber_node)
    ld.add_action(py_publisher_node)
    ld.add_action(py_subscriber_node)
    return ld