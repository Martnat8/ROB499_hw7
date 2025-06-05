#!/usr/bin/env python3


# cluster_finder.py
#
# Nathan Martin
#
# Node that subscribes to a PointCloud2 topic and publishes the nubmer of clusters in it


import rclpy
import math
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32
from sklearn.cluster import DBSCAN
import numpy as np


class ClusterFinder(Node):
	'''
	This node subscribes to a PointCloud2 topic and publishe the number of clusters
	'''
	def __init__(self):
		# Initialize the parent class.
		super().__init__('cluster_finder')

		# Create the subscriber
		self.sub = self.create_subscription(PointCloud2, '/cleaned_PC2', self.callback, 10)

		# Create the publisher
		self.pub = self.create_publisher(Float32, '/num_clusters', 10)


	# This callback will be called whenever we receive a new message on the topic.
	def callback(self, msg):

		num_clusters = Float32()

		points = []
		for point in point_cloud2.read_points(msg, field_names=("x", "y", "z")):
			points.append([point[0], point[1], point[2]])

		if len(points) == 0:
			self.get_logger().warn("No points in scan to cluster; skipping")
			return



		# Using DBSCAn from Scikit learn to find clusters
		# Turning the cluster into 2D
		features = np.array([[x, y] for x, y, z in points])
		clustering = DBSCAN(eps=0.05, min_samples=5).fit(features)

		# Labels correlate with the index of the points passed in
		labels = clustering.labels_
		n_clusters = labels.max() + 1

		# Publish with count of clusters
		num_clusters.data = float(n_clusters) if n_clusters >= 0 else 0.0

		self.get_logger().info(f'{num_clusters.data} found above the table')

		self.pub.publish(num_clusters)


	
def main(args=None):

	# Initialize rclpy. 
	rclpy.init(args=args)

	# Make a node class.
	counter = ClusterFinder()

	# Handover to ROS2
	rclpy.spin(counter)

	# Clean Shutdown
	rclpy.shutdown()


# If we run the node as a script, then we're going to start here.
if __name__ == '__main__':

	main()
