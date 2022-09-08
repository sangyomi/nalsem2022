import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAMESPACE = os.environ.get('ROS_NAMESPACE', '')

def generate_launch_description():
    # 카메라 subscriber 노드
    camera_sub_node = Node(
        namespace=NAMESPACE,
        package='mecha_autoship_camera',
        executable='mecha_autoship_image_sub_node',
        name='mecha_autoship_image_sub_node',
        output='screen',
        emulate_tty=True,
        parameters=[],
    )

    return LaunchDescription([
        camera_sub_node
    ])