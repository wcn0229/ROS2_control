import os
from ament_index_python.packages import get_package_share_directory
import launch.actions
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition 
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    #  parameters
    param_file = os.path.join(get_package_share_directory('ds_node'), 
        'param',
        'ds_talker_params.yaml')

    nav_tcp = LaunchConfiguration('nav_tcp', default='False')
    nav_udp = LaunchConfiguration('nav_udp', default='False')
    return LaunchDescription([
        
        # Set env var to print messages to stdout immediately
        # requirement #1
        launch.actions.SetEnvironmentVariable(
            'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),
        
        # [node_namespace] and [node_executable] deprecated after distro Dashing
        Node(
            package='ds_node',
            namespace='ds_ns',
            executable='ds_node_talker',
            name='ds_node_talker', # useless
            output='screen', # requirement #2
            parameters=[param_file]
        ),

        Node(
            package='ds_node',
            namespace='ds_ns',
            executable='ds_binary_tcp',
            name='ds_binary_tcp',
            condition=IfCondition(nav_tcp),
            output='screen',
            parameters=[param_file]
        ),

        Node(
            package='ds_node',
            namespace='ds_ns',
            executable='ds_binary_udp',
            name='ds_binary_udp',
            condition=IfCondition(nav_udp),
            output='screen',
            parameters=[param_file]
        )
    ])
