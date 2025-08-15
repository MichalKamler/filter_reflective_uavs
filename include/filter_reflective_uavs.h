#ifndef FILTER_REFLECTIVE_UAVS_H
#define FILTER_REFLECTIVE_UAVS_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

// I/O
#include <stdio.h>

// matrix math
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

// messages
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>

// custom helper functions from mrs library
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>
#include <mrs_msgs/PoseWithCovarianceIdentified.h>
// PCL
#include <ouster_ros/point.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/search/kdtree.h>
// #include <pcl/search/impl/kdtree.hpp>
// #include <pcl/kdtree/kdtree.h>
// #include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/common/common.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/impl/crop_box.hpp>
#include <pcl/filters/plane_clipper3D.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/organized.h>
#include <pcl/search/impl/organized.hpp>
#include <pcl/search/search.h>
#include <pcl/search/impl/search.hpp>

#include <pcl/common/centroid.h> 
#include <pcl/segmentation/extract_clusters.h> 


// helper libraries
#include <string>
#include <vector>
#include <cmath>
#include <queue>
#include <mutex>
#include <shared_mutex>

namespace filter_reflective_uavs {

class FilterReflectiveUavs : public nodelet::Nodelet {

public:
	virtual void onInit();
	bool 																										is_initialized_ = false;

	std::string              																					_uav_name_;
	int 																										_min_intensity_;
	int 																										_max_intensity_;
	double																										_search_radius_;
	double 																										_max_distance_from_seed_;
	int 																										_max_removed_points_;
	bool 																										_ouster_;
	bool																										_load_gt_uav_positions_;
	std::vector<std::pair<ros::Time, Eigen::Vector3d>>																				uav_positions;
	mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2>															sh_pointcloud_;
	mrs_lib::SubscribeHandler<mrs_msgs::PoseWithCovarianceArrayStamped>															sh_uav_position_estimation_;
	ros::Publisher                 																				pub_pointCloud_;
	ros::Publisher																								pub_agent_pcl_;
	// mutable std::mutex 																							uav_positions_mutex;
	mrs_lib::Transformer transformer_;
	mutable std::shared_mutex uav_positions_mutex;


	void callbackPointCloudOuster(const sensor_msgs::PointCloud2::ConstPtr msg);
	void callbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr msg);
	void filterOutUavs(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud, const std::string& frame_id, const ros::Time& timestamp);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_agent_pcl_to_centroids(pcl::PointCloud<pcl::PointXYZ>::Ptr agent_pcl);
	void callbackPoses(const mrs_msgs::PoseWithCovarianceArrayStamped::ConstPtr msg);
	void timeoutGeneric(const std::string& topic, const ros::Time& last_msg);
};

} //namespace filter_reflective_uavs

#endif
