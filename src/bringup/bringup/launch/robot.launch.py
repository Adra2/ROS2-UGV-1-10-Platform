"""
robot.launch.py  —  Full robot bringup
Usage:
  ros2 launch bringup robot.launch.py
  ros2 launch bringup robot.launch.py hardware_only:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    cfg_dir = os.path.join(
        get_package_share_directory('bringup'), 'config'
    )

    # ── args ──────────────────────────────────────────────────────────────────
    hardware_only = DeclareLaunchArgument(
        'hardware_only', default_value='false',
        description='Launch only hardware nodes (no perception/sensors)'
    )

    # ── hardware nodes ────────────────────────────────────────────────────────
    servo_node = Node(
        package    = 'vehicle_interface',
        executable = 'servo_node',
        name       = 'servo_node',
        parameters = [os.path.join(cfg_dir, 'vehicle_interface.yaml')],
        output     = 'screen',
        emulate_tty = True,
    )

    esc_node = Node(
        package    = 'vehicle_interface',
        executable = 'esc_node',
        name       = 'esc_node',
        parameters = [os.path.join(cfg_dir, 'vehicle_interface.yaml')],
        output     = 'screen',
        emulate_tty = True,
    )

    # ── sensor nodes ─────────────────────────────────────────────────────────
    gps_node = Node(
        package    = 'sensors',
        executable = 'gps_node',
        name       = 'gps_node',
        parameters = [os.path.join(cfg_dir, 'sensors.yaml')],
        output     = 'screen',
        condition  = UnlessCondition(LaunchConfiguration('hardware_only')),
    )

    rc_receiver_node = Node(
        package    = 'sensors',
        executable = 'rc_receiver_node',
        name       = 'rc_receiver_node',
        parameters = [os.path.join(cfg_dir, 'sensors.yaml')],
        output     = 'screen',
        condition  = UnlessCondition(LaunchConfiguration('hardware_only')),
    )

    # ── perception nodes ──────────────────────────────────────────────────────
    camera_node = Node(
        package    = 'perception',
        executable = 'camera_node',
        name       = 'camera_node',
        parameters = [os.path.join(cfg_dir, 'perception.yaml')],
        output     = 'screen',
        condition  = UnlessCondition(LaunchConfiguration('hardware_only')),
    )

    return LaunchDescription([
        hardware_only,
        servo_node,
        esc_node,
        gps_node,
        rc_receiver_node,
        camera_node,
    ])
