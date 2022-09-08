import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAMESPACE = os.environ.get('ROS_NAMESPACE', '')

def generate_launch_description():
    # 카메라 publishing 노드
    mecha_autoship_image_pub_params = LaunchConfiguration(
        'mecha_autoship_image_pub_params',
        default=os.path.join(
            get_package_share_directory('mecha_autoship_camera'),
            'param',
            'mecha_autoship_camera.yaml'
        )
    )
    camera_pub_node_arg = DeclareLaunchArgument(
        'mecha_autoship_image_pub_params',
        default_value=mecha_autoship_image_pub_params
    )
    camera_pub_node = Node(
        namespace=NAMESPACE,
        package='mecha_autoship_camera',
        executable='mecha_autoship_image_pub_node',
        name='mecha_autoship_image_pub_node',
        output='screen',
        emulate_tty=True,
        parameters=[mecha_autoship_image_pub_params],
    )

    # 카메라 필터 노드
    mecha_autoship_image_color_filter_params = LaunchConfiguration(
        'mecha_autoship_image_color_filter_params',
        default=os.path.join(
            get_package_share_directory('mecha_autoship_camera'),
            'param',
            'mecha_autoship_camera.yaml'
        )
    )
    camera_filter_node_arg = DeclareLaunchArgument(
        'mecha_autoship_image_color_filter_params',
        default_value=mecha_autoship_image_color_filter_params
    )
    camera_filter_node = Node(
        namespace=NAMESPACE,
        package='mecha_autoship_camera',
        executable='mecha_autoship_image_color_filter_node',
        name='mecha_autoship_image_color_filter_node',
        output='screen',
        emulate_tty=True,
        parameters=[mecha_autoship_image_color_filter_params],
    )

    return LaunchDescription([
        camera_pub_node_arg,
        camera_pub_node,
        camera_filter_node_arg,
        camera_filter_node
    ])