#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    raiz_compartida = get_package_share_path('robot_description')
    URDF_INDICE = str(raiz_compartida / 'urdf/indice.urdf')
    URDF_PULGAR = str(raiz_compartida / 'urdf/pulgar.urdf')
    RVIZ_CFG    = str(raiz_compartida / 'rviz/urdf.rviz')

    #
    arg_gui = DeclareLaunchArgument(
        name='gui', default_value='false', choices=['true', 'false'],
        description='Activar/desactivar joint_state_publisher_gui'
    )
    arg_modelo_indice = DeclareLaunchArgument(
        name='modelo_indice', default_value=URDF_INDICE,
        description='Ruta al URDF/Xacro del índice'
    )
    arg_modelo_pulgar = DeclareLaunchArgument(
        name='modelo_pulgar', default_value=URDF_PULGAR,
        description='Ruta al URDF/Xacro del pulgar'
    )
    arg_config_rviz = DeclareLaunchArgument(
        name='config_rviz', default_value=RVIZ_CFG,
        description='Ruta al archivo de configuración de RViz'
    )

  
    desc_indice = ParameterValue(
        Command(['xacro ', LaunchConfiguration('modelo_indice')]), value_type=str
    )
    desc_pulgar = ParameterValue(
        Command(['xacro ', LaunchConfiguration('modelo_pulgar')]), value_type=str
    )

    
    rsp_indice = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        namespace='index', name='rsp_indice',
        parameters=[{'robot_description': desc_indice},
                    {'frame_prefix': 'idx_'}],
        remappings=[('joint_states', '/index/joint_states')]
    )
    jsp_indice_cli = Node(
        package='joint_state_publisher', executable='joint_state_publisher',
        namespace='index', name='jsp_indice_cli',
        condition=UnlessCondition(LaunchConfiguration('gui')),
        remappings=[('joint_states', 'joint_states')]
    )
    jsp_indice_gui = Node(
        package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
        namespace='index', name='jsp_indice_gui',
        condition=IfCondition(LaunchConfiguration('gui')),
        remappings=[('joint_states', 'joint_states')]
    )

   
    rsp_pulgar = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        namespace='thumb', name='rsp_pulgar',
        parameters=[{'robot_description': desc_pulgar},
                    {'frame_prefix': 'th_'}],
        remappings=[('joint_states', '/thumb/joint_states')]
    )
    jsp_pulgar_cli = Node(
        package='joint_state_publisher', executable='joint_state_publisher',
        namespace='thumb', name='jsp_pulgar_cli',
        condition=UnlessCondition(LaunchConfiguration('gui')),
        remappings=[('joint_states', 'joint_states')]
    )
    jsp_pulgar_gui = Node(
        package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
        namespace='thumb', name='jsp_pulgar_gui',
        condition=IfCondition(LaunchConfiguration('gui')),
        remappings=[('joint_states', 'joint_states')]
    )

    
    tf_indice = Node(
        package='tf2_ros', executable='static_transform_publisher', name='tf_indice',
        arguments=['0', '0', '0', '0', '0', '0', 'common_frame_T', 'idx_link0_passive']
    )
    tf_pulgar = Node(
        package='tf2_ros', executable='static_transform_publisher', name='tf_pulgar',
        arguments=['0', '0.035', '0', '0', '0', '0', 'common_frame_T', 'th_link0_passive']
    )

   
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz',
        output='screen',
        arguments=['-d', LaunchConfiguration('config_rviz')]
    )

    return LaunchDescription([
        arg_gui, arg_modelo_indice, arg_modelo_pulgar, arg_config_rviz,
        rsp_indice, jsp_indice_cli, jsp_indice_gui,
        rsp_pulgar, jsp_pulgar_cli, jsp_pulgar_gui,
        tf_indice, tf_pulgar,
        rviz
    ])
