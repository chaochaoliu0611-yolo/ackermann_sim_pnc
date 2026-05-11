#!/usr/bin/env python3
"""
SAYE Nav2 Bringup (自定义插件版)
─────────────────────────────────
使用 Nav2 官方 bringup_launch.py, 加载自定义规划控制插件。
控制器切换: 修改 nav2_params_saye.yaml 中 controller_server.FollowPath.plugin

使用方式:
  终端 1: ros2 launch saye_bringup saye_spawn.launch.py rviz:=false
  终端 2: ros2 launch saye_nav2_plugins saye_nav_bringup.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_saye_bringup     = get_package_share_directory('saye_bringup')
    pkg_saye_nav2_plugins = get_package_share_directory('saye_nav2_plugins')
    pkg_nav2_bringup     = get_package_share_directory('nav2_bringup')

    map_file     = os.path.join(pkg_saye_bringup, 'maps', 'map.yaml')
    params_file  = os.path.join(pkg_saye_nav2_plugins, 'config', 'nav2_params_saye.yaml')
    rviz_config  = os.path.join(pkg_saye_bringup, 'rviz', 'navigation.rviz')

    # Nav2 官方 bringup (包含 localization + navigation 全部生命周期管理)
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map':              map_file,
            'params_file':      params_file,
            'use_sim_time':     'True',
            'use_composition':  'False',
            'autostart':        'True',
        }.items()
    )

    # RViz
    from launch_ros.actions import Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # odom→saye TF: Gazebo PosePublisher 桥接时间戳异常，自行发布可靠的 TF
    odom_to_tf = Node(
        package='saye_nav2_plugins',
        executable='odom_to_tf.py',
        name='odom_to_tf',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    ld = LaunchDescription()
    ld.add_action(nav2_cmd)
    ld.add_action(odom_to_tf)
    ld.add_action(rviz_node)
    return ld
