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
#include <nav_msgs/Odometry.h>
// custom helper functions from mrs library
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>
#include <mrs_msgs/PoseWithCovarianceIdentified.h>
// #include <mrs_lib/scope_timer.h>
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
#include <pcl/filters/conditional_removal.h>
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

#include <filter_reflective_uavs/PoseVelocityArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>




// helper libraries
#include <string>
#include <vector>
#include <cmath>
#include <queue>
#include <mutex>
#include <shared_mutex>

namespace filter_reflective_uavs {

struct Track {
    int id;
    ros::Time last_update;
    Eigen::VectorXd x;   // KF state vector (pos, vel, acc)
    Eigen::MatrixXd P;   // KF covariance
};

class FilterReflectiveUavs : public nodelet::Nodelet {

public:
	virtual void onInit();
	bool 																										is_initialized_ = false;

	std::string              																					_uav_name_;
	std::string 																								_global_frame_;
	float 																										_min_intensity_;
	float 																										_max_intensity_;

	std::vector<Track> 																							tracks_;
	double 																										_dt_;
	double 																										_max_no_update_;
	double 																										_gate_treshold_;

	float 																										_reflective_clustering_tolerance_;
	int         																								_reflective_clustering_min_points_;
  	int         																								_reflective_clustering_max_points_;
	bool        																								_use_voxel_grid_;
  	float       																								_voxel_grid_size_x_;
  	float       																								_voxel_grid_size_y_;
  	float       																								_voxel_grid_size_z_;
	double																										_search_radius_;
	double 																										_max_distance_from_seed_;
	int 																										_max_removed_points_;
	bool 																										_ouster_;
	bool																										_load_gt_uav_positions_;
  	double                                                  													_time_keep_;
	Eigen::Vector3d																								_agent_pos_;
	std::vector<std::pair<ros::Time, Eigen::Vector3d>>															centroid_positions_;
  	std::vector<std::pair<ros::Time, Eigen::Vector3d>>															uav_positions;
	mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2>															sh_pointcloud_;
	mrs_lib::SubscribeHandler<mrs_msgs::PoseWithCovarianceArrayStamped>															sh_uav_position_estimation_;
	ros::Publisher 																								publisher_pointcloud_reflective_centroids_;
	ros::Publisher 																								publisher_estimates_;
	ros::Publisher                 																				pub_pointCloud_;
	ros::Publisher																								pub_pointCloud_removed_;
	ros::Publisher																								pub_seeds_;
	ros::Publisher																								pub_agent_pcl_;
	ros::Publisher 																								pub_pose_vel_array_;
	ros::Publisher																								pub_velocity_markers_;
	ros::Subscriber																								sub_pointCloud2_pos_;
	ros::Subscriber																								sub_odom_;
	tf::TransformListener 																						tf_listener_;
	// mutable std::mutex 																							uav_positions_mutex;
	mrs_lib::Transformer transformer_;
	mutable std::shared_mutex uav_positions_mutex;


	void callbackPointCloudOuster(const sensor_msgs::PointCloud2::ConstPtr msg);
	void callbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr msg);

	//Multiuav tracking
	void update(const std::vector<std::pair<ros::Time, Eigen::Vector3d>>& measurements);
	bool associateMeasurement(const Eigen::Vector3d& meas, int& track_id);
    void initializeTrack(const ros::Time& t, const Eigen::Vector3d& meas);
    void predictTrack(Track& track);
    void updateTrack(Track& track, const Eigen::Vector3d& meas);
    void deleteStaleTracks(const ros::Time& current_time);
	void publishVelAsArrow(const std::string& frame_id, const ros::Time& timestamp, std::vector<Track>& tracks);
	void publishEstimates(const std::string& frame_id, const ros::Time& timestamp, std::vector<Track>& tracks);

	//Clustering to centroids
	std::vector<std::pair<ros::Time, Eigen::Vector3d>> clusterToCentroids(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, ros::Time timestamp, const std::string& frame_id);
	void calculateCentroid2(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const std::vector<pcl::PointIndices>& cluster_indices, std::vector<pcl::PointXYZ>& result);
	std::vector<pcl::PointIndices> doEuclideanClustering(const pcl::search::KdTree<pcl::PointXYZI>::Ptr tree_orig, const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float clustering_tolerance, int min_points, int max_points, const pcl::IndicesConstPtr indices_within_radius = nullptr);

	//Filtering out uavs for map
	void filterOutUavs(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud, const std::string& frame_id, const ros::Time& timestamp, std::vector<Track>& tracks);

	//helper func
	geometry_msgs::Pose pose_to_msg(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity);
	void covariance_to_msg(const Eigen::Matrix3d& cov, boost::array<double, 36>& msg_cov_out);
	void callbackPoses(const mrs_msgs::PoseWithCovarianceArrayStamped::ConstPtr msg);
	void timeoutGeneric(const std::string& topic, const ros::Time& last_msg);
	void pointCloud2PosCallback(const sensor_msgs::PointCloud2& pcl_cloud2);
	void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
};

} //namespace filter_reflective_uavs

#endif
