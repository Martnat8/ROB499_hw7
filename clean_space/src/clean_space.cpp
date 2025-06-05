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
		cleaned_pub_ = this->create_publisher<PointCloud2>("/cleaned_PC2", 1);
		table_pub_ = this->create_publisher<PointCloud2>("/table", 1);

		// Subscriber for raw point cloud
		subscriber_ = this->create_subscription<PointCloud2>(
			"/astra_ros/devices/default/point_cloud", 1,
			std::bind(&TablePlanePublisher::callback, this, _1));
	}

private:
	rclcpp::Publisher<PointCloud2>::SharedPtr cleaned_pub_;
	rclcpp::Publisher<PointCloud2>::SharedPtr table_pub_;
	rclcpp::Subscription<PointCloud2>::SharedPtr subscriber_;

	void callback(const PointCloud2::SharedPtr msg) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(*msg, *input_cloud);

		auto points_out = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
		auto table_out = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);


		// Filter out points outside of a range
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(input_cloud);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.5, 2.0);  // Keep points 0.5m to 2.0m away
		pass.filter(*input_cloud);

		// Find points above the plane
		pcl::PointCloud<pcl::PointXYZ>::Ptr above_plane(new pcl::PointCloud<pcl::PointXYZ>);
		segment_plane(input_cloud, points_out);

		// Find table plane
		publish_plane(input_cloud, table_out);		

		// Convert to ROS message
		PointCloud2 output1, output2;
		pcl::toROSMsg(*points_out, output1);
		pcl::toROSMsg(*table_out, output2);
		output1.header = msg->header;
		output2.header = msg->header;
		


		cleaned_pub_->publish(output1);
		table_pub_->publish(output2);
		
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
		seg.setDistanceThreshold(0.05);
		seg.setInputCloud(cloud);
		seg.segment(*inliers, *coeffs);

		// Keep points above the table
		for (const auto& point : cloud->points) {
			float distance = coeffs->values[0] * point.x + coeffs->values[1] * point.y + 
							coeffs->values[2] * point.z + coeffs->values[3];
			if (distance > 0.05) {  // Above table by 5cm
				plane_out->points.push_back(point);
			}
		}
		}

	void publish_plane(
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
		pcl::PointCloud<pcl::PointXYZ>::Ptr filtered){
			
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
	
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.01);
		seg.setInputCloud(cloud);
		seg.segment(*inliers, *coeffs);
	
	
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*filtered);	
		}
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<TablePlanePublisher>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
