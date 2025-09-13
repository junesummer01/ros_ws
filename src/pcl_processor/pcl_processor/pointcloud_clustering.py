#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from sklearn.cluster import DBSCAN
from std_msgs.msg import ColorRGBA

class PointCloudClustering(Node):
    def __init__(self):
        super().__init__('pointcloud_clustering')

        # Subscriber for point cloud data
        self.subscription = self.create_subscription(
            PointCloud2,
            '/cloud_registered_body',
            self.pointcloud_callback,
            10)

        # Publisher for bounding box visualization
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/clustering_markers',
            10)

        # Clustering parameters
        self.eps = 0.5  # DBSCAN eps parameter
        self.min_samples = 10  # DBSCAN min_samples parameter
        self.min_cluster_size = 20  # Minimum points to consider as valid cluster
        self.max_cluster_size = 1000  # Maximum points to consider as valid cluster

        self.get_logger().info('PointCloud Clustering Node Started')

    def pointcloud_callback(self, msg):
        try:
            # Convert PointCloud2 to numpy array
            points = self.pointcloud2_to_array(msg)

            if len(points) == 0:
                self.get_logger().warn('Empty point cloud received')
                return

            # Perform clustering
            clusters = self.perform_clustering(points)

            # Create and publish visualization markers
            self.publish_bounding_boxes(clusters, msg.header)

        except Exception as e:
            self.get_logger().error(f'Error in pointcloud callback: {str(e)}')

    def pointcloud2_to_array(self, cloud_msg):
        """Convert PointCloud2 message to numpy array"""
        points_list = []

        # Extract x, y, z coordinates from point cloud
        for point in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            points_list.append([point[0], point[1], point[2]])

        return np.array(points_list)

    def perform_clustering(self, points):
        """Perform DBSCAN clustering on point cloud"""
        if len(points) < self.min_samples:
            return []

        # Apply DBSCAN clustering
        clustering = DBSCAN(eps=self.eps, min_samples=self.min_samples)
        labels = clustering.fit_predict(points)

        # Group points by cluster labels
        clusters = []
        unique_labels = set(labels)

        for label in unique_labels:
            if label == -1:  # Skip noise points
                continue

            cluster_points = points[labels == label]

            # Filter out clusters that are too small or too large
            if self.min_cluster_size <= len(cluster_points) <= self.max_cluster_size:
                clusters.append(cluster_points)

        self.get_logger().info(f'Found {len(clusters)} valid clusters')
        return clusters

    def calculate_bounding_box(self, cluster_points):
        """Calculate bounding box for a cluster of points"""
        min_vals = np.min(cluster_points, axis=0)
        max_vals = np.max(cluster_points, axis=0)

        # Calculate center and dimensions
        center = (min_vals + max_vals) / 2.0
        dimensions = max_vals - min_vals

        return center, dimensions

    def publish_bounding_boxes(self, clusters, header):
        """Create and publish bounding box markers for clusters"""
        marker_array = MarkerArray()

        for i, cluster in enumerate(clusters):
            center, dimensions = self.calculate_bounding_box(cluster)

            # Create cube marker for bounding box
            marker = Marker()
            marker.header = header
            marker.ns = "clustering_boxes"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            # Set position
            marker.pose.position.x = float(center[0])
            marker.pose.position.y = float(center[1])
            marker.pose.position.z = float(center[2])
            marker.pose.orientation.w = 1.0

            # Set scale (dimensions)
            marker.scale.x = float(max(dimensions[0], 0.1))  # Minimum size 0.1m
            marker.scale.y = float(max(dimensions[1], 0.1))
            marker.scale.z = float(max(dimensions[2], 0.1))

            # Set color (different colors for different clusters)
            color = self.get_cluster_color(i)
            marker.color = color

            # Set lifetime
            marker.lifetime.sec = 1  # Marker lasts for 1 second

            marker_array.markers.append(marker)

            # Create text marker for cluster info
            text_marker = Marker()
            text_marker.header = header
            text_marker.ns = "clustering_text"
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            # Position text above the bounding box
            text_marker.pose.position.x = float(center[0])
            text_marker.pose.position.y = float(center[1])
            text_marker.pose.position.z = float(center[2] + dimensions[2]/2 + 0.2)
            text_marker.pose.orientation.w = 1.0

            text_marker.scale.z = 0.3  # Text size
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0

            text_marker.text = f"Cluster {i}\n{len(cluster)} pts"
            text_marker.lifetime.sec = 1

            marker_array.markers.append(text_marker)

        # Clear old markers by publishing empty markers with DELETE action
        if len(clusters) < 50:  # Avoid creating too many markers
            for j in range(len(clusters), 50):
                delete_marker = Marker()
                delete_marker.header = header
                delete_marker.ns = "clustering_boxes"
                delete_marker.id = j
                delete_marker.action = Marker.DELETE
                marker_array.markers.append(delete_marker)

                delete_text = Marker()
                delete_text.header = header
                delete_text.ns = "clustering_text"
                delete_text.id = j
                delete_text.action = Marker.DELETE
                marker_array.markers.append(delete_text)

        self.marker_pub.publish(marker_array)

    def get_cluster_color(self, cluster_id):
        """Generate different colors for different clusters"""
        colors = [
            (1.0, 0.0, 0.0, 0.7),  # Red
            (0.0, 1.0, 0.0, 0.7),  # Green
            (0.0, 0.0, 1.0, 0.7),  # Blue
            (1.0, 1.0, 0.0, 0.7),  # Yellow
            (1.0, 0.0, 1.0, 0.7),  # Magenta
            (0.0, 1.0, 1.0, 0.7),  # Cyan
            (1.0, 0.5, 0.0, 0.7),  # Orange
            (0.5, 0.0, 1.0, 0.7),  # Purple
        ]

        color_rgba = ColorRGBA()
        color_tuple = colors[cluster_id % len(colors)]
        color_rgba.r = color_tuple[0]
        color_rgba.g = color_tuple[1]
        color_rgba.b = color_tuple[2]
        color_rgba.a = color_tuple[3]

        return color_rgba

def main(args=None):
    rclpy.init(args=args)

    try:
        node = PointCloudClustering()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()