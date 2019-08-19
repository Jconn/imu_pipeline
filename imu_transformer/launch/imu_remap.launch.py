import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch.actions


def generate_launch_description():
    
    imu_tf = Node(
            package='imu_transformer',
            node_executable='imu_transformer',
            node_name='ImuTransformer',
            output='screen',
            parameters=[{ 'target_frame':'base_link', 'imu_frame': 'LSM9DS1'}]
            )
    ld = []
    ld.append(imu_tf) 

    return LaunchDescription(ld)
