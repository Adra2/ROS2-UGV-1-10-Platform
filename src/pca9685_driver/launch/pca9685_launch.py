from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='vehicle_interface',
            executable='vehicle_interface',
            name='vehicle_interface',
            output='screen'
        ),

        Node(
            package='pca9685_driver',
            executable='pca9685_node',
            name='pca9685_node',
            output='screen',
            parameters=[{
                'frequency': 50,
                'steering_channel': 0,
                'throttle_channel': 1
            }]
        )
    ])