import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
import sys

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='kurome').find('kurome')
    default_model_path = os.path.join(pkg_share, 'config/demo_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'config/config.rviz')
    default_config_path = os.path.join(pkg_share, 'config/demo_config.yaml')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[default_model_path]
    )

    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
            executable='spawn_entity.py',
        arguments=['-entity', 'robot', '-topic', 'robot_description'],
        output='screen'
    )    

    robot_localization_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share,'config/ekf.yaml'),{'use_sim_time' : LaunchConfiguration('use_sim_time')}]
    )

    lidar_transform = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['--x', '0', '--y', '0', '--z', '0.12', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'lidar_link']
    )

    odom_transform = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'odom', '--child-frame-id', 'base_link']
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )

    pino_node = launch_ros.actions.Node(
        package='kurome',
        executable='pino',
        namespace='kurome',
        name='pino',
        parameters=[LaunchConfiguration('config')]
    )

    misao_node = launch_ros.actions.Node(
        package='kurome',
        executable='misao',
        namespace='kurome',
        name='misao',
        parameters=[LaunchConfiguration('config')]
    )

    yoriko_node = launch_ros.actions.Node(
        package='kurome',
        executable='yoriko',
        namespace='kurome',
        name='yoriko',
        parameters=[LaunchConfiguration('config')]
    )

    pose_publisher_node = launch_ros.actions.Node(
        package='kurome',
        executable='pose_publisher',
        namespace='kurome',
        name='pose_publisher',
        parameters=[LaunchConfiguration('config')]
    )

    mask_node = launch_ros.actions.Node(
        package='kurome',
        executable='local_mask',
        namespace='kurome',
        name='local_mask',
        parameters=[LaunchConfiguration('config')]
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', os.path.join(pkg_share,'config/demo_world.sdf')], output='screen'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time',default_value='True',
                                             description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                             description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='config',default_value=default_config_path,
                                             description='Absolute path to kurome config file'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
        robot_localization_node,
        lidar_transform,
#        odom_transform,
        pino_node,
        yoriko_node,
        misao_node,
        pose_publisher_node,
        mask_node,
        rviz_node
    ])
