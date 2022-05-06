from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('ohm_tsd_slam'),
        'config',
        'single-laser.yaml'
        )

    return LaunchDescription([
        Node(
            package='ohm_tsd_slam',
            executable='slam_node',
            name='tsd_slam',
            remappings=[
                # Subscriptions
                ('/tsd_slam/laser', 'tsd_slam/laser'),
                # Publisher
                ('/tsd_slam/map', 'tsd_slam/map'),
                ('/tsd_slam/estimated_pose', 'tsd_slam/estimated_pose'),
                ('/tsd_slam/map/image', 'tsd_slam/map/image2'),
                # Services
                ('/tsd_slam/start_stop_slam', 'tsd_slam/start_stop_slam'),
                ('/tsd_slam/get_map', 'tsd_slam/get_map')
            ],
            parameters=[config],
        )
    ])