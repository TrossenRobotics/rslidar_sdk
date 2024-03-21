# Copyright 2024 Trossen Robotics - All Rights Reserved
#
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential

"""
This launch file does the following.

- Launches the Safety Features with the following nodes:
    - collision_monitor
    - collision_detector
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import (
    Node,
    ComposableNodeContainer,
)
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    rslidar_config_file = LaunchConfiguration('rslidar_config')
    lidar_container_name = LaunchConfiguration('lidar_container_name')

    lifecycle_nodes = ['trossen_rslidar_node']

    load_composable_nodes = ComposableNodeContainer(
        name=lidar_container_name,
        namespace='',
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package='rslidar_sdk',
                plugin='robosense::lidar::PointCloudLFNode',
                name='trossen_rslidar_node',
                parameters=[rslidar_config_file],
                remappings=[],
            ),
            # ComposableNode(
            #     package='rviz2',
            #     node_namespace='rviz2',
            #     node_name='rviz2',
            #     node_executable='rviz2',
            #     arguments=['-d',LaunchConfiguration('rviz_config')],
            #     condition=IfCondition(LaunchConfiguration('use_rviz'))
            # )
        ],
        output="screen",
    )

    lifecycle_manager_lidar_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_lidar',
        parameters=[
            {'autostart': True},
            {'node_names': lifecycle_nodes},
        ],
        remappings=[],
    )

    # rviz_node = Node(
    #         package='rviz2',
    #         node_namespace='rviz2',
    #         node_name='rviz2',
    #         node_executable='rviz2',
    #         arguments=['-d',LaunchConfiguration('rviz_config')],
    #     )

    return [
        load_composable_nodes,
        lifecycle_manager_lidar_node,
        # rviz_node,
    ]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'rslidar_config',
            default_value=os.path.join(
                get_package_share_directory('rslidar_sdk'),
                'config',
                'config_ros2.yaml',
            ),
            description='Full path to the ROS 2 parameters file to use for all launched nodes',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'lidar_container_name',
            default_value='rslidar_container',
            description='the name of container that will contain lidar nodes',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
                get_package_share_directory('rslidar_sdk'),
                'rviz',
                'rviz2.rviz'
            ),
            description='path to RViz2 config file',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            choices=('true', 'false'),
            description='use RViz for visualization if true, disable if false.',
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
