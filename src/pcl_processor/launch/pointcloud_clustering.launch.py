from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('pcl_processor')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'pointcloud_clustering.rviz')

    return LaunchDescription([
        Node(
            package='pcl_processor',
            executable='pointcloud_clustering',
            name='pointcloud_clustering_node',
            output='screen',
            parameters=[
                {'eps': 0.5},
                {'min_samples': 10},
                {'min_cluster_size': 20},
                {'max_cluster_size': 1000}
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])