import os
import sys

from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
import launch.actions
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='ros_mstar',
            node_executable='m_star_planner',
            node_name='m_star_planner',
            output='screen',
            arguments=['0.5', '128', '/r1/global_costmap', 'get_multi_robot_plan', '0.25', 'l2', '1', 'inf']),
    ])
