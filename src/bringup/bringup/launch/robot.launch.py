"""
robot.launch.py  —  Main robot bringup
Package:   bringup

Usage:
  ros2 launch bringup robot.launch.py                    # full launch
  ros2 launch bringup robot.launch.py hardware_only:=true  # servo+ESC only
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    cfg_dir = os.path.join(
        get_package_share_directory('bringup'), 'config'
    )

    # ── launch args ───────────────────────────────────────────────────────────
    hardware_only = DeclareLaunchArgument(
        'hardware_only', default_value='false',
        description='Start only hardware nodes (vehicle_interface + pca9685)'
    )

    # ── hardware layer ────────────────────────────────────────────────────────

    # Converts normalised Float32 → µs Int16, deadman watchdog, ESC arming
    vehicle_interface_node = Node(
        package    = 'vehicle_interface',
        executable = 'vehicle_interface',       # entry_point in vehicle_interface/setup.py
        name       = 'vehicle_interface',
        parameters = [os.path.join(cfg_dir, 'vehicle_interface.yaml')],
        output     = 'screen',
        emulate_tty = True,
    )

    # Low-level PCA9685 I²C driver (adafruit_pca9685 / Blinka)
    pca9685_node = Node(
        package    = 'pca9685_driver',
        executable = 'pca9685_node',            # entry_point in pca9685_driver/setup.py
        name       = 'pca9685_node',
        parameters = [os.path.join(cfg_dir, 'vehicle_interface.yaml')],
        output     = 'screen',
        emulate_tty = True,
    )

    # ── sensor layer ──────────────────────────────────────────────────────────

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

    # ── perception layer ──────────────────────────────────────────────────────

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
        vehicle_interface_node,
        pca9685_node,
        gps_node,
        rc_receiver_node,
        camera_node,
    ])
