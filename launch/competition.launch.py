"""Launch the installed student simulation without workspace-relative paths."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = {
        'mode': ('demo', 'demo: complete reference mission; student: external cmd_vel and payload services'),
        'gui': ('true', 'Open the Gazebo GUI'),
        'cameras': ('false', 'Enable four cameras and ROS image bridges; requires graphics'),
        'rate': ('1.0', 'Requested real-time factor; 0 runs as fast as possible'),
        'output': ('', 'New run directory; empty creates a timestamped ~/.ros run'),
        'max_sim_time': ('0', 'Stop after this many simulated seconds; 0 disables the limit'),
        'exit_on_finish': ('false', 'Stop the simulator when result.json is written'),
    }
    declarations = [DeclareLaunchArgument(k, default_value=v[0], description=v[1])
                    for k, v in arguments.items()]
    argv = []
    for key in arguments:
        argv.extend(['--' + key.replace('_', '-'), LaunchConfiguration(key)])
    return LaunchDescription(declarations + [Node(
        package='d1_tello_competition', executable='simulator.py',
        arguments=argv, output='screen')])
