#!/usr/bin/env python3

# Node that subscribes to a PointCloud topic, removes the background and the table
# and republishes the new PointCloud 
#
# lasercull.py
#
# Nathan Martin
#


import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2



class CleanPointCloud(Node):
	'''
	This node subscribes to a laser scan topic, has a service that takes a snapshot of the
	laser scan, removes the static points and republishes.
	'''
	def __init__(self):
		# Initialize the parent class.
		super().__init__('cleanpointcloud')

		# Parameter to define later
		self.declare_parameter('boundary', 2.0)
		self.boundary = self.get_parameter('boundary').get_parameter_value().double_value

		# Create the subscriber to the bag's PointCloud2 topic
		self.sub = self.create_subscription(PointCloud2, '/astra_ros/devices/default/point_cloud', self.callback, 10)

		# Create the publisher for the cleaned PointCloud2
		self.pub = self.create_publisher(PointCloud2, '/clean_point_cloud', 10)


	# This callback will be called whenever we receive a new message on the topic. (Currenly leftover from hw5)
	def callback(self, msg):

		# Create a copy of the current laser scan
		self.current_scan = msg

		# Only run if there's been a snapshot taken
		if self.laser_scan_snapshot is not None:

			# Create a new message and copy over headers
			n = len(msg.ranges)
			clean_scan = LaserScan()
			clean_scan.ranges = [float('inf')]*n
			clean_scan.intensities = [0.0]*n

			# Duplicate headers
			clean_scan.header = msg.header
			clean_scan.angle_min = msg.angle_min
			clean_scan.angle_max = msg.angle_max
			clean_scan.angle_increment = msg.angle_increment
			clean_scan.time_increment = msg.time_increment
			clean_scan.scan_time = msg.scan_time
			clean_scan.range_min = msg.range_min
			clean_scan.range_max = msg.range_max

			for i, r in enumerate(msg.ranges):

				snap = self.laser_scan_snapshot.ranges[i]

				# Skip points that are too close to the poins in snapshot
				if abs(snap-r) <=  self.boundary:

					continue

				# Copy over points that are 'unique'
				clean_scan.ranges[i] = r
				clean_scan.intensities[i] = msg.intensities[i]
			

			self.get_logger().info(f'Publishing adjusted scan with adjusted points')

			# Republish the message. 
			self.pub.publish(clean_scan)


# This is a entry point.	
def main(args=None):

	# Initialize rclpy. 
	rclpy.init(args=args)

	# Make a node class.
	cleaned = CleanPointCloud()

	# Handover to ROS2
	rclpy.spin(cleaned)

	# Clean Shutdown
	rclpy.shutdown()


# If we run the node as a script, then we're going to start here.
if __name__ == '__main__':

	main()
