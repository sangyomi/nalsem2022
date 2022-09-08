import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace, LifecycleNode

NAMESPACE = os.environ.get('ROS_NAMESPACE', '')

def generate_launch_description():
    # MCU
    mcu_node = Node(
            namespace=NAMESPACE,
            package='mecha_autoship_bringup',
            executable='mecha_autoship_mcu_node',
            name='mecha_autoship_mcu_node',
            parameters=[],
            output='screen'
        )
    # IMU 데이터 가공
    imu_filter_node = Node(
            namespace=NAMESPACE,
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick_node',
            parameters=[{
                'gain': 0.1,
                # 'zeta': 0,
                'use_mag': True,
                'fixed_frame': 'base_link',
                'publish_tf': True
            }]
        )

    # LiDAR 드라이버
    mecha_autoship_lidar_param = LaunchConfiguration(
            'mecha_autoship_lidar_param',
            default=os.path.join(
                get_package_share_directory('mecha_autoship_bringup'),
                'param',
                'mecha_autoship_lidar.yaml'
            )
        )
    lidar_driver_node = LifecycleNode(
            namespace=NAMESPACE,
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            emulate_tty=True,
            parameters=[mecha_autoship_lidar_param],
        )
    # LiDAR TF2
    lidar_tf_node = Node(
            namespace=NAMESPACE,
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_laser',
            arguments=['0', '0', '0.02','0', '0', '0', '1','base_link','laser_frame'],
        )
    # LiDAR 데이터 가공
    lidar_data_node = Node(
            namespace=NAMESPACE,
            package='mecha_autoship_bringup',
            executable='mecha_autoship_lidar_node',
            name='mecha_autoship_lidar_node',
            parameters=[],
            output='screen'
        )

    return LaunchDescription([
        mcu_node,
        imu_filter_node,
        lidar_driver_node,
        lidar_tf_node,
        lidar_data_node
    ])