#!/usr/bin/env python3
"""
Launch file for running ds_node tests
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for testing."""
    
    # Declare launch arguments
    test_type_arg = DeclareLaunchArgument(
        'test_type',
        default_value='all',
        description='Type of tests to run: all, unit, integration, convert, nmea'
    )
    
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Run tests with verbose output'
    )
    
    # Get package share directory
    package_share = FindPackageShare('ds_node')
    
    # Test executables
    test_convert = ExecuteProcess(
        cmd=[
            PathJoinSubstitution([package_share, 'test', 'test_ds_convert']),
            '--gtest_output=xml:results/test_ds_convert.xml'
        ],
        output='screen' if LaunchConfiguration('verbose') == 'true' else 'own',
        condition=lambda context: LaunchConfiguration('test_type').perform(context) in ['all', 'unit', 'convert']
    )
    
    test_nmea = ExecuteProcess(
        cmd=[
            PathJoinSubstitution([package_share, 'test', 'test_ds_nmea']),
            '--gtest_output=xml:results/test_ds_nmea.xml'
        ],
        output='screen' if LaunchConfiguration('verbose') == 'true' else 'own',
        condition=lambda context: LaunchConfiguration('test_type').perform(context) in ['all', 'unit', 'nmea']
    )
    
    test_integration = ExecuteProcess(
        cmd=[
            PathJoinSubstitution([package_share, 'test', 'test_integration']),
            '--gtest_output=xml:results/test_integration.xml'
        ],
        output='screen' if LaunchConfiguration('verbose') == 'true' else 'own',
        condition=lambda context: LaunchConfiguration('test_type').perform(context) in ['all', 'integration']
    )
    
    # Test node for message testing
    test_node = Node(
        package='ds_node',
        executable='ds_node',
        name='test_ds_node',
        output='screen',
        parameters=[{
            'test_mode': True,
            'publish_rate': 10.0,
            'coordinate_frame': 'test_frame'
        }],
        condition=lambda context: LaunchConfiguration('test_type').perform(context) in ['all', 'integration']
    )
    
    # Create results directory
    create_results_dir = ExecuteProcess(
        cmd=['mkdir', '-p', 'results'],
        output='own'
    )
    
    return LaunchDescription([
        test_type_arg,
        verbose_arg,
        create_results_dir,
        test_convert,
        test_nmea,
        test_integration,
        test_node
    ])