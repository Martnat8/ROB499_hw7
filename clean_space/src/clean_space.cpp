// clean_space.cpp
//
// Nathan Martin
//
// This node takes in a PointCloud2, segments out the dominant plane (likely a tabletop),
// and republishes just the points belonging to that plane.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>


using std::placeholders::_1;

class TablePlanePublisher : public rclcpp::Node {
public:
	using PointCloud2 = sensor_msgs::msg::PointCloud2;

	TablePlanePublisher() : Node("table_plane_publisher") {
		// Publisher for the segmented plane
		publisher_ = this->create_publisher<PointCloud2>("table_plane", 1);

		// Subscriber for raw point cloud
		subscriber_ = this->create_subscription<PointCloud2>(
			"/astra_ros/devices/default/point_cloud", 1,
			std::bind(&TablePlanePublisher::callback, this, _1));
	}

private:
	rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;
	rclcpp::Subscription<PointCloud2>::SharedPtr subscriber_;

	void callback(const PointCloud2::SharedPtr msg) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(*msg, *input_cloud);

		// Filter out points outside of a range
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(input_cloud);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.5, 2.0);  // Keep points 0.5m to 2.0m away
		pass.filter(*input_cloud);

		// Segment the plane
		pcl::PointCloud<pcl::PointXYZ>::Ptr table_plane(new pcl::PointCloud<pcl::PointXYZ>);
		segment_plane(input_cloud, table_plane);

		// Convert to ROS message
		PointCloud2 output;
		pcl::toROSMsg(*table_plane, output);
		output.header = msg->header;

		RCLCPP_INFO(this->get_logger(), "Publishing %lu table plane points.", table_plane->size());
		publisher_->publish(output);
	}

	void segment_plane(
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
		pcl::PointCloud<pcl::PointXYZ>::Ptr plane_out) {

		pcl::SACSegmentation<pcl::PointXYZ> seg;
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);

		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.01);
		seg.setInputCloud(cloud);
		seg.segment(*inliers, *coeffs);

		RCLCPP_INFO(this->get_logger(), "Plane inliers: %lu", inliers->indices.size());

		// Keep points above the table
		for (const auto& point : cloud->points) {
			float distance = coeffs->values[0] * point.x + coeffs->values[1] * point.y + 
							coeffs->values[2] * point.z + coeffs->values[3];
			if (distance > 0.05) {  // Above table by 8cm
				plane_out->points.push_back(point);
			}
		}
		}
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<TablePlanePublisher>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
