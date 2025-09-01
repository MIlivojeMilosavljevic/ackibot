#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    ThisLaunchFileDir,
    PathJoinSubstitution
)
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

import mybot_utils.uname
from mybot_utils.utils import show
from mybot_utils.usb_mapper import USB_Mapper

def parse_bool(s):
    sl = s.lower()
    t = sl in ['true', '1', 'yes', 'y']
    f = sl in ['false', '0', 'no', 'n']
    if t:
        return True
    if f:
        return False
    raise TypeError(f'Cannot parse {s} to bool!')

def launch_setup(context, *args, **kwargs):
    en_teleop = LaunchConfiguration('en_teleop', default='true')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    joypad = LaunchConfiguration('joypad', default='sony')


    #assert(mybot_utils.uname.is_raspberrypi())
    um = USB_Mapper()
    show(um.table)

    arduino_port = um.get_exactly_1_dev_of_class('Arduino')
    show(arduino_port)


    params_fn = os.path.join(
        get_package_share_directory('mybot_bringup'),
        'param',
        'mybot.yaml'
    )



    return [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'en_teleop',
            default_value=en_teleop,
            description='launch teleop'
        ),
        DeclareLaunchArgument(
            'joypad',
            default_value='sony'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                ThisLaunchFileDir(),
                '/state_publisher.launch.py'
            ]),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('mybot_teleop'),
                    'launch',
                    'teleop.launch.py'
                )
            ]),
            launch_arguments={
                'machine': 'sbc',
                'joypad': joypad
            }.items(),
            condition=IfCondition(en_teleop),
        ),
        Node(
            package="twist_stamper",
            executable="twist_stamper",
            name='twist_stamper',
            output='screen',
            remappings=[
                ('cmd_vel_in', '/cmd_vel_nav_unstamped'),
                ('cmd_vel_out', '/cmd_vel_nav_stamped'),
            ],
        ),
        Node(
            package="twist_mux",
            executable="twist_mux",
            name='twist_mux',
            output='screen',
            #arguments=['--ros-args', '--log-level', 'DEBUG'],
            parameters=[
                os.path.join(
                    get_package_share_directory('mybot_bringup'),
                    'config',
                    'twist_mux_topics.yaml'
                ),
                #{'use_stamped': True}, # This kills my twist_mux
            ],
            remappings=[
                ('cmd_vel_out', '/cmd_vel_node'),
            ],
        ),

        Node(
            package='mybot_node',
            executable='fw_node',
            parameters=[params_fn],
            arguments=['-i', arduino_port],
            output='screen',
            remappings=[
                ('cmd_vel', '/cmd_vel_node'),
            ],
        ),
    ]

def generate_launch_description():
	ld = LaunchDescription([
		OpaqueFunction(function = launch_setup)
	])

	return ld