#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Launch the minimal DG competition loop.

This launch file starts the first practical autonomous flow:

  P -> QR -> wait for QR signal -> return P -> stop

Nav2 itself, chassis, lidar, EKF and camera should already be running.
The QR detector can be started here after the camera is available.
The HDMI display node is optional because it needs a valid X11 DISPLAY.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import os


def generate_launch_description():
    """Generate the minimal competition launch description."""

    dg_pkg_share = get_package_share_directory('dg_pkg')
    default_params_file = os.path.join(
        dg_pkg_share,
        'config',
        'competition_params.yaml',
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='比赛自动巡航参数 yaml 文件完整路径',
    )
    declare_start_qrcode = DeclareLaunchArgument(
        'start_qrcode',
        default_value='true',
        description='是否启动二维码识别节点 qrcode_detector',
    )
    declare_start_display = DeclareLaunchArgument(
        'start_display',
        default_value='false',
        description='是否启动 7 寸屏显示节点 qr_result_display',
    )
    declare_debounce_frames = DeclareLaunchArgument(
        'debounce_frames',
        default_value='2',
        description='二维码连续识别去抖帧数',
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='nav_cruise 是否启动后立即执行任务',
    )
    declare_publish_initial_pose = DeclareLaunchArgument(
        'publish_initial_pose',
        default_value='true',
        description='是否由 nav_cruise 自动发布 /initialpose',
    )
    declare_qr_wait_timeout_sec = DeclareLaunchArgument(
        'qr_wait_timeout_sec',
        default_value='60.0',
        description='到达 QR 点后等待二维码方向信号的超时时间，0 表示不超时',
    )

    qrcode_detector = Node(
        package='dg_pkg',
        executable='qrcode_detector',
        name='qrcode_detector',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_qrcode')),
        parameters=[{
            'debounce_frames': ParameterValue(
                LaunchConfiguration('debounce_frames'),
                value_type=int,
            ),
        }],
    )

    qr_result_display = Node(
        package='dg_pkg',
        executable='qr_result_display',
        name='qr_result_display',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_display')),
    )

    nav_cruise = Node(
        package='dg_pkg',
        executable='nav_cruise',
        name='nav_cruise',
        output='screen',
        parameters=[{
            'params_file': LaunchConfiguration('params_file'),
            'autostart': ParameterValue(
                LaunchConfiguration('autostart'),
                value_type=bool,
            ),
            'publish_initial_pose': ParameterValue(
                LaunchConfiguration('publish_initial_pose'),
                value_type=bool,
            ),
            'qr_wait_timeout_sec': ParameterValue(
                LaunchConfiguration('qr_wait_timeout_sec'),
                value_type=float,
            ),
        }],
    )

    return LaunchDescription([
        declare_params_file,
        declare_start_qrcode,
        declare_start_display,
        declare_debounce_frames,
        declare_autostart,
        declare_publish_initial_pose,
        declare_qr_wait_timeout_sec,
        qrcode_detector,
        qr_result_display,
        nav_cruise,
    ])
