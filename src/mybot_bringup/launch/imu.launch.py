
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import (
    LaunchConfiguration,
)
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)


from mybot_utils.utils import show

def launch_setup(context, *args, **kwargs):
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB2')
    
    return [
        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='USB device for IMU'
        ),

        Node(
            package = 'bno055',
            executable = 'bno055',
            respawn = True,
            parameters = [
                os.path.join(
                    get_package_share_directory('bno055'),
                    'config',
                    'bno055_params.yaml'
                ),
                {
                    # Override port
                    'uart_port': serial_port,
                }
            ],
            remappings = [
                ('/bno055/imu', '/imu')
            ],
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[
                os.path.join(
                    get_package_share_directory('mybot_bringup'), 
                    'config/ekf.yaml'
                ),
            ],
            #TODO To ekf.yaml file
            remappings = [
                ('/odometry/filtered', '/odom')
            ],
        ),
    ]

def generate_launch_description():
	ld = LaunchDescription([
		OpaqueFunction(function = launch_setup)
	])

	return ld