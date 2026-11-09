from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('lidar_intensity_assist')
    config = LaunchConfiguration('config_file')
    rviz_config = LaunchConfiguration('rviz_config_file')
    return LaunchDescription([
        DeclareLaunchArgument('node_name', default_value='pcd_correction_node'),
        DeclareLaunchArgument('config_file', default_value=PathJoinSubstitution([share, 'config', 'start.yaml'])),
        DeclareLaunchArgument('raw_cloud_topic', default_value='/raw_cloud'),
        DeclareLaunchArgument('deskewed_cloud_topic', default_value='/deskewed_cloud'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config_file', default_value=PathJoinSubstitution([share, 'rviz', 'start.rviz'])),
        Node(package='lidar_intensity_assist', executable='pcd_correction_node', name=LaunchConfiguration('node_name'),
             output='screen', parameters=[config], remappings=[
                 ('raw_cloud', LaunchConfiguration('raw_cloud_topic')),
                 ('deskewed_cloud', LaunchConfiguration('deskewed_cloud_topic'))]),
        Node(package='rviz2', executable='rviz2', name='lidar_intensity_assist_rviz',
             arguments=['-d', rviz_config], output='screen', condition=IfCondition(LaunchConfiguration('start_rviz'))),
    ])
