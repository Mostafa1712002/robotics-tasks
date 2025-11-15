#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Paths
    task1_dir = os.path.join(os.path.expanduser('~'), 'www', 'ros-robtics-tasks', 'tasks', 'task1')
    world_file = os.path.join(task1_dir, 'worlds', 'custom_world.sdf')
    bridge_config = os.path.join(task1_dir, 'config', 'bridge.yaml')

    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Set Gazebo resource path
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(os.path.expanduser('~'), '.gz', 'models')
    )

    # Launch Gazebo with custom world
    try:
        pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
        gz_sim = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': f'-r {world_file}'
            }.items()
        )
    except:
        # Fallback if ros_gz_sim not found
        print("WARNING: ros_gz_sim not found. You may need to launch Gazebo manually:")
        print(f"  gz sim -r {world_file}")
        gz_sim = None

    # Spawn TurtleBot3 (using built-in TurtleBot3 model from Gazebo Fuel)
    spawn_turtlebot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'turtlebot3_waffle_pi',
            '-topic', '/robot_description',
            '-x', '4.5',
            '-y', '4.5',
            '-z', '0.01',
            '-Y', '-2.356',  # -135 degrees (facing toward center)
        ],
        output='screen'
    )

    # ROS-Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p', f'config_file:={bridge_config}',
            '-p', f'use_sim_time:={use_sim_time}'
        ],
        output='screen'
    )

    # Robot State Publisher (for TF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ''  # Will be populated by spawn
        }]
    )

    # Create launch description
    ld = [
        gz_resource_path,
    ]

    if gz_sim:
        ld.append(gz_sim)

    ld.extend([
        spawn_turtlebot,
        bridge,
        robot_state_publisher,
    ])

    return LaunchDescription(ld)
