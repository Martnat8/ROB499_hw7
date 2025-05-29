// cylinders.cpp
//
// Bill Smart
//
// This is an example of using the Point Cloud Library in ROS 2
// to find cylinders in point cloud data.


// Include the basic ROS 2 stuff.
#include <rclcpp/rclcpp.hpp>

// We're going to be dealing with point clouds.
#include <sensor_msgs/msg/point_cloud2.hpp>

// Include the point cloud library (PCL) stuff.
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>


// Binding parameters for the callback.
using std::placeholders::_1;


// The idiom is always to create a Node subclass.
class CylinderFinder : public rclcpp::Node {
public:
	// Simplify the syntax a bit.
	using PointCloud2 = sensor_msgs::msg::PointCloud2;

	CylinderFinder() :Node("cylinder_finder") {
		// Create a publisher for the modified point cloud.
		publisher_ = this->create_publisher<PointCloud2>("modified", 1);

		// Create a subscriber for the point cloud.  We're only going to keep an input
		// buffer of size 1, since this might be a slow process.
		subscriber_ = this->create_subscription<PointCloud2>("/astra_ros/devices/default/point_cloud", 1, std::bind(&CylinderFinder::callback, this, _1));
	}

private:

	rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;
	rclcpp::Subscription<PointCloud2>::SharedPtr subscriber_;

	void callback(const PointCloud2::SharedPtr msg) {


		// Translate the ROS PointCloud2 to a PCL PointCloud.
		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(*msg, *pcl_cloud);

		// Remove the table plane
		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_no_plane(new pcl::PointCloud<pcl::PointXYZ>);
		segment_plane(pcl_cloud, pcl_cloud_no_plane);
		
		// Point cloud for only cylinders
		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_only_cylinders(new pcl::PointCloud<pcl::PointXYZ>);

		int count = 0;
		const int MAX_ITER = 5;
		
		while (count++ < MAX_ITER) {
			pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder(new pcl::PointCloud<pcl::PointXYZ>);
			if (!segment_cylinder(pcl_cloud_no_plane, cylinder)) {
				break;
			}
			if (cylinder->empty()) {
				break;
			}
			*pcl_cloud_only_cylinders += *cylinder;
		
			if (pcl_cloud_no_plane->empty()) {
				RCLCPP_WARN(this->get_logger(), "Remaining cloud is empty, breaking loop.");
				break;
			}
		}

		// Convert PCL PointCloud to a ROS PointCloud2.
		PointCloud2 ros_cloud;
		pcl::toROSMsg(*pcl_cloud_only_cylinders, ros_cloud);

		// Set the header information.
		ros_cloud.header.stamp = msg->header.stamp;
		ros_cloud.header.frame_id = "ramsis/base_laser_scanner"; 
		

		RCLCPP_INFO(this->get_logger(), "Publishing cylinder point cloud...");

		// And, publish it out.
		this->publisher_->publish(ros_cloud);

	}

	void segment_plane(
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
	
		RCLCPP_INFO(this->get_logger(), "Plane inliers: %li", inliers->indices.size());
	
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*filtered);	
		}

	bool segment_cylinder(
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
		pcl::PointCloud<pcl::PointXYZ>::Ptr extracted_cylinder){

		// Compute normals
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

		// Set the parameters.
		ne.setSearchMethod(tree);
		ne.setInputCloud(cloud);
		ne.setKSearch(10);
		ne.compute(*cloud_normals);

		// Allocate a segmentation model for cylinder
		pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);		

		// Set the model type, algorithm, distance threshold, and the point cloud we're
		// using as input.
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_CYLINDER);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.05);
		seg.setRadiusLimits(0, 1);
		seg.setInputCloud(cloud);
		seg.setInputNormals(cloud_normals);
		seg.segment(*inliers, *coefficients);

		if (inliers->indices.empty()){
			return false;
		}

		// Filter the model points out of the cloud. 
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*extracted_cylinder);

		// Remove cylinder points from cloud
		extract.setNegative(true);
		pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
		extract.filter(*temp);
		*cloud = *temp;

		// Log how many points are claimed by the model.
		RCLCPP_INFO(this->get_logger(), "Got %li inliers.", inliers->indices.size());
		return true;
	}
};


// This is the entry point for the executable.
int main(int argc, char **argv) {
	// Initialize ROS.
	rclcpp::init(argc, argv);

	// Create a node and store a shared pointer to it.  We're
	// going to use the auto keyword here to make sure we get
	// the type right.
	auto node = std::make_shared<CylinderFinder>();

	// Give control to ROS via the shared pointer.
	rclcpp::spin(node);

	// Once the event handler is done, shut things down nicely.
	rclcpp::shutdown();

	// Main always returns 0 on success.
	return 0;
}
