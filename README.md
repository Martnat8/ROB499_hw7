# ROB 499 Robot Software Frameworks – HW7

This package contains HW7 for ROB 499 (Robotic Software Frameworks) at Oregon State University.

It includes two ROS 2 nodes that take in a PointCloud2 message, detect the tabletop plane,
and republish:
- The points of the table
- The points above the table 
- A count of the clusters above the table

---

## Dependencies

This package requires **scikit-learn** and the **PCL library**.

Install them with:

```bash
pip install scikit-learn
sudo apt install libpcl-dev ros-jazzy-pcl-conversions ros-jazzy-pcl-msgs
```

---

## Node Descriptions

- `clean_space`: A C++ node that uses the PCL library to segment the table and filter the point cloud.
- `cluster_finder`: A Python node that uses DBSCAN from scikit-learn to detect clusters above the table and publishes the cluster count.

---

## Usage

To play the bag file:

```bash
ros2 bag play drink_table
```

After unpacking, building, and sourcing the workspace, launch the nodes:

```bash
ros2 launch cluster_finder hw7launch.py
```

---

## Maintainer

Nathan Martin  
Email: martnat8@oregonstate.edu  

License: BSD 3-Clause

---

## References

- Cluster finder was largely repurposed from HW5
- **DBSCAN**: [scikit-learn DBSCAN](https://scikit-learn.org/stable/modules/generated/sklearn.cluster.DBSCAN.html)
- **PointCloud2 Python API**:  
  https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/src/sensor_msgs/point_cloud2.py
- **PointCloud2 ROS 2 Demo**:  
  https://github.com/SebastianGrans/ROS2-Point-Cloud-Demo/blob/master/pcd_demo/pcd_subscriber/pcd_subscriber_node.py
- **RViz Config Basics**:  
  https://www.learnros2.com/ros/ros2-building-blocks/configurations/rviz-configuration
- **Advanced RViz Config**:  
  https://robotics.snowcron.com/robotics_ros2/nav_improved_launch.htm
