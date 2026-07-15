"""
Navigation2 导航栈一键启动文件 — 适配 OriginCar 阿克曼底盘。

启动机制:
  通过 IncludeLaunchDescription 引入官方 nav2_bringup/bringup_launch.py，
  传入自定义 params_file。运行时自动生成临时参数文件，将动态获取的
  两棵阿克曼安全 BT 的绝对路径注入 bt_navigator 的 Humble 参数，
  确保 cmd_vel 控制链重映射由官方 bringup 内部正确处理:
    controller_server → /cmd_vel_nav → velocity_smoother → /cmd_vel → 底盘

不启动的节点（由外部 launch 分别负责）:
  底盘 (origincar_base)、EKF、N10 雷达 (lslidar_driver)、
  base_link→laser 静态 TF

启动参数:
  map          — 地图 yaml 文件路径（默认 /root/maps/competition_map.yaml）
  params_file  — Nav2 参数 yaml 文件路径（默认使用 dg_pkg/config/nav2_params.yaml）
                 用户覆盖后仍会自动注入安全 BT 路径
  use_sim_time — 是否使用仿真时间（默认 false，由官方 bringup 统一下发）
  autostart    — lifecycle 自动启动（默认 true）

规格来源: FRAMEWORK.md §3.10
维护者: Claude Code（M10 阿克曼安全补丁 R3）
"""

import os
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include_nav2_with_safe_bt(
        context, nav_to_pose_bt_path, nav_through_poses_bt_path):
    """注入两个导航器的安全 BT 路径并 Include 官方 Nav2 bringup。"""

    selected_params_file = LaunchConfiguration('params_file').perform(context)
    if not os.path.isfile(selected_params_file):
        raise RuntimeError(
            f'Nav2 params_file does not exist: {selected_params_file}'
        )

    with open(selected_params_file, 'r', encoding='utf-8-sig') as file:
        params = yaml.safe_load(file)

    if not isinstance(params, dict):
        raise RuntimeError(
            f'Nav2 params_file must contain a YAML mapping: '
            f'{selected_params_file}'
        )

    bt_params = params.setdefault(
        'bt_navigator', {}
    ).setdefault('ros__parameters', {})
    bt_params['default_nav_to_pose_bt_xml'] = nav_to_pose_bt_path
    bt_params['default_nav_through_poses_bt_xml'] = nav_through_poses_bt_path
    bt_params.pop('default_bt_xml_filename', None)

    runtime_params_file = os.path.join(
        tempfile.gettempdir(),
        f'dg_nav2_params_{os.getpid()}.yaml',
    )
    with open(runtime_params_file, 'w', encoding='utf-8') as file:
        yaml.safe_dump(
            params,
            file,
            default_flow_style=False,
            allow_unicode=True,
            sort_keys=False,
        )

    nav2_bringup_launch = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('nav2_bringup'),
            'launch',
            'bringup_launch.py',
        )
    )

    return [
        IncludeLaunchDescription(
            nav2_bringup_launch,
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'params_file': runtime_params_file,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': LaunchConfiguration('autostart'),
                'use_composition': 'False',
            }.items(),
        )
    ]


def generate_launch_description():
    """生成 Nav2 导航栈 LaunchDescription。"""

    # ============================================================
    # 路径计算
    # ============================================================
    dg_pkg_share = get_package_share_directory('dg_pkg')

    # 阿克曼安全 BT XML 绝对路径（动态获取，不硬编码小车路径）
    nav_to_pose_bt_path = os.path.join(
        dg_pkg_share, 'behavior_trees',
        'navigate_to_pose_ackermann_safe.xml'
    )
    nav_through_poses_bt_path = os.path.join(
        dg_pkg_share, 'behavior_trees',
        'navigate_through_poses_ackermann_safe.xml'
    )

    # 基础参数文件（dg_pkg 安装目录内）
    base_params_file = os.path.join(dg_pkg_share, 'config', 'nav2_params.yaml')

    # ============================================================
    # Launch 参数（与官方 bringup 一致的四个接口）
    # ============================================================
    declare_map = DeclareLaunchArgument(
        'map',
        default_value='/root/maps/competition_map.yaml',
        description='SLAM 地图 yaml 文件完整路径'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=base_params_file,
        description='Nav2 参数 yaml 文件完整路径'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否启用仿真时间（由官方 bringup 统一下发到各节点）'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='是否自动触发 lifecycle transition'
    )

    nav2_bringup = OpaqueFunction(
        function=include_nav2_with_safe_bt,
        kwargs={
            'nav_to_pose_bt_path': nav_to_pose_bt_path,
            'nav_through_poses_bt_path': nav_through_poses_bt_path,
        },
    )

    return LaunchDescription([
        declare_map,
        declare_params_file,
        declare_use_sim_time,
        declare_autostart,
        nav2_bringup,
    ])
