dimport argparse
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescriptor
from launch.exit_handler import restart_exit_handler
from launch.launcher import DefaultLauncher
from launch.output_handler import ConsoleOutput
from ros2run.api import get_executable_path


def launch(launch_descriptor, argv):
    parser = argparse.ArgumentParser(description='launch mstar planner')
    parser.add_argument(
        '--map',
        help='path to map (will be passed to map_server)')
    args = parser.parse_args(argv)

    ld = launch_descriptor

    package = 'ros_mstar'
    ld.add_process(
        cmd=[get_executable_path(package_name=package, executable_name='m_star_planner.py'), 
        '0.5', '128', '<costmap_topic>', 'mstar_service', '0.25',
        'l2', '1', 'inf'],
        name='m_star_planner',
        exit_handler=restart_exit_handler,
    )

    return ld

def main(argv=sys.argv[1:]):
    launcher = DefaultLauncher()
    launch_descriptor = launch(LaunchDescriptor(), argv)
    launcher.add_launch_descriptor(launch_descriptor)
    rc = launcher.launch()
    return rc


if __name__ == '__main__':
    sys.exit(main())