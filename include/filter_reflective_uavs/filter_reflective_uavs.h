#ifndef FILTER_REFLECTIVE_UAVS_FILTER_REFLECTIVE_UAVS_H
#define FILTER_REFLECTIVE_UAVS_FILTER_REFLECTIVE_UAVS_H

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

// MRS LIB
#include <mrs_lib/node.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/subscriber_handler.h>

// MRS MSGs
#include <mrs_msgs/msg/pose_with_covariance_array_stamped.hpp>
#include <mrs_msgs/msg/pose_with_covariance_identified.hpp>

// MSGS
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// ROS 2
#include <rclcpp/rclcpp.hpp>

// TF
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// CUSTOM
#include <filter_reflective_uavs/msg/pose_velocity_array.hpp>
#include <ouster_ros/os_point.h>

// PCL
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/crop_box.hpp>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/plane_clipper3D.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/kdtree.h>
#include <pcl/pcl_base.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/impl/organized.hpp>
#include <pcl/search/impl/search.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

namespace filter_reflective_uavs {

struct Track {
  int id;
  rclcpp::Time last_update;
  Eigen::VectorXd x;
  Eigen::MatrixXd P;
};

class FilterReflectiveUavs : public mrs_lib::Node {
public:
  explicit FilterReflectiveUavs(rclcpp::NodeOptions options);

  void initialize();

private:
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;
  using PointCloudMsgPtr = PointCloudMsg::ConstSharedPtr;
  using OdomMsg = nav_msgs::msg::Odometry;
  using OdomMsgPtr = OdomMsg::ConstSharedPtr;
  using PoseArrayMsg = mrs_msgs::msg::PoseWithCovarianceArrayStamped;
  using StampPositionPair = std::pair<rclcpp::Time, Eigen::Vector3d>;

  void loadParameters();
  void callbackPointCloud2(const PointCloudMsgPtr msg);
  void timerPointCloudCallback();
  void splitCloudBySpheres( const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& input_cloud,
                          const std::vector<Eigen::Vector3d>& centers,
                          float radius,
                          pcl::PointCloud<pcl::PointXYZI>& kept_cloud,
                          pcl::PointCloud<pcl::PointXYZI>& removed_cloud);


  void callbackPointCloud(const PointCloudMsgPtr msg);
  void pointCloud2PosCallback(const PointCloudMsgPtr msg);
  void odomCallback(const OdomMsgPtr msg);
  void pruneExpiredPositions(std::vector<StampPositionPair>& positions,
                             const rclcpp::Time&             current_time) const;
  void cacheDetectedCentroids(const std::vector<StampPositionPair>& centroid_positions_global,
                              const rclcpp::Time&                    current_time);
  std::vector<StampPositionPair> getCachedDetectedCentroids(const rclcpp::Time& current_time);
  std::vector<StampPositionPair> loadGtUavCentroids(const std::string& frame_id,
                                                    const rclcpp::Time& timestamp) const;
  void addPointsToCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr         pcl_cloud,
                        const std::vector<StampPositionPair>&         points) const;
  void injectDetectedCentroids(pcl::PointCloud<pcl::PointXYZI>::Ptr               pcl_cloud,
                               const std::string&                                 frame_id,
                               const rclcpp::Time&                                timestamp,
                               const std::vector<StampPositionPair>&              cached_detected_centroids) const;

  std::vector<StampPositionPair> filterLatestDetections(const std::vector<StampPositionPair>& collected_centroid_positions, 
                                                        double                                search_radius) const;
  std::vector<StampPositionPair> transfromAndPublishCentroids(const std::vector<StampPositionPair>&  centroid_positions, 
                                                              const std::string&                     frame_id, 
                                                              const rclcpp::Time&                    timestamp);

  void update(const std::vector<StampPositionPair>& measurements);
  bool associateMeasurement(const Eigen::Vector3d&  meas, 
                            int&                    track_id) const;
  void initializeTrack(const rclcpp::Time&    stamp, 
                       const Eigen::Vector3d& meas);
  void predictTrack(Track &track) const;
  void updateTrack(Track&                 track, 
                   const Eigen::Vector3d& meas) const;
  void deleteStaleTracks(const rclcpp::Time& current_time);
  void publishVelAsArrow(const std::string&        frame_id,
                         const rclcpp::Time&       timestamp,
                         const std::vector<Track>& tracks);
  void publishEstimates(const std::string&         frame_id,
                        const rclcpp::Time&        timestamp,
                        const std::vector<Track>&  tracks);

  std::vector<StampPositionPair> clusterToCentroids(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                                                    const rclcpp::Time&                        timestamp,
                                                    const std::string&                         frame_id) const;
  void calculateCentroid(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                         const std::vector<pcl::PointIndices>&           cluster_indices,
                         std::vector<pcl::PointXYZ>&                     result) const;
  std::vector<pcl::PointIndices> doEuclideanClustering(const pcl::search::KdTree<pcl::PointXYZI>::Ptr& tree_orig,
                                                       const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud,
                                                       float                                          clustering_tolerance,
                                                       int                                            min_points,
                                                       int                                            max_points,
                                                       const pcl::IndicesConstPtr                     indices_within_radius = nullptr) const;

  void filterOutUavs(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud,
                     const std::string&                    frame_id,
                     const rclcpp::Time&                   timestamp,
                     const std::vector<Track>&             tracks);

  geometry_msgs::msg::Pose poseToMsg(const Eigen::Vector3d& position,
                                     const Eigen::Vector3d& velocity) const;
  void covarianceToMsg(const Eigen::Matrix3d&  cov,
                       std::array<double, 36>& msg_cov_out) const;

  std::optional<geometry_msgs::msg::TransformStamped> lookupTransform(const std::string&  target_frame,
                                                                      const std::string&  source_frame,
                                                                      const rclcpp::Time& stamp) const;
  std::optional<Eigen::Vector3d> transformPoint(const Eigen::Vector3d& point,
                                                const std::string&     from_frame,
                                                const std::string&     to_frame,
                                                const rclcpp::Time&    stamp) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_lidar_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_aux_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_timers_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr               collected_reflective_cloud_{std::make_shared<pcl::PointCloud<pcl::PointXYZI>>()};
  std::mutex                                         collected_reflective_cloud_mutex_;
  std::string                                        collected_reflective_cloud_frame_id_;
  rclcpp::Time                                       collected_reflective_cloud_timestamp_;
  std::vector<Eigen::Vector3d>                       estimates_;
  std::mutex                                         estimates_mutex_;

  void cbTmEstimate();
  std::shared_ptr<TimerType> tm_estimate_;

  bool                                                                          is_initialized_{false};
  bool                                                                          debug_{false};
  bool                                                                          simulation_{false};

  std::string                                                                   uav_name_;
  std::string                                                                   global_frame_;
  std::string                                                                   frame_id_;
  std::vector<std::string>                                                      detected_uav_names_;
  double                                                                        min_intensity_{250.0};
  double                                                                        max_intensity_{255.0};
  std::vector<Track>                                                            tracks_;
  double                                                                        dt_{0.2};
  double                                                                        max_no_update_{1.0};
  double                                                                        gate_treshold_{2.0};
  std::chrono::steady_clock::time_point                                         last_update_;
  double                                                                        reflective_clustering_tolerance_{0.4};
  int                                                                           reflective_clustering_min_points_{1};
  int                                                                           reflective_clustering_max_points_{999999};
  bool                                                                          use_voxel_grid_{true};
  double                                                                        voxel_grid_size_x_{0.04};
  double                                                                        voxel_grid_size_y_{0.04};
  double                                                                        voxel_grid_size_z_{0.04};
  double                                                                        search_radius_{1.0};
  double                                                                        max_distance_from_seed_{1.0};
  bool                                                                          filter_out_myself_enabled_{true};
  double                                                                        filter_out_myself_dist_{1.0};
  double                                                                        time_keep_{0.2};
  int                                                                           subscriber_queue_size_{50};

  Eigen::Vector3d                                                               agent_pos_{Eigen::Vector3d::Zero()};
  std::vector<StampPositionPair>                                                centroid_positions_;
  std::vector<StampPositionPair>                                                collected_centroid_positions_;
  std::vector<StampPositionPair>                                                uav_positions_;
  std::vector<StampPositionPair>                                                detected_centroid_positions_;

  mutable std::shared_mutex                                                     uav_positions_mutex_;
  mutable std::shared_mutex                                                     detected_centroid_positions_mutex_;

  mrs_lib::SubscriberHandler<PointCloudMsg>                                     sh_pointcloud_;
  mrs_lib::SubscriberHandler<PointCloudMsg>                                     sh_pointcloud_pos_;
  mrs_lib::SubscriberHandler<OdomMsg>                                           sh_odom_;
  rclcpp::TimerBase::SharedPtr                                                  timer_pointcloud_;
  PointCloudMsgPtr                                                              latest_pointcloud_msg_;

  mrs_lib::PublisherHandler<PointCloudMsg>                                      publisher_pointcloud_reflective_centroids_;
  mrs_lib::PublisherHandler<PoseArrayMsg>                                       publisher_estimates_;
  mrs_lib::PublisherHandler<PointCloudMsg>                                      pub_pointcloud_;
  mrs_lib::PublisherHandler<PointCloudMsg>                                      pub_pointcloud_removed_;
  mrs_lib::PublisherHandler<PointCloudMsg>                                      pub_seeds_;
  mrs_lib::PublisherHandler<PointCloudMsg>                                      pub_agent_pcl_;
  mrs_lib::PublisherHandler<filter_reflective_uavs::msg::PoseVelocityArray>     pub_pose_vel_array_;
  mrs_lib::PublisherHandler<visualization_msgs::msg::MarkerArray>               pub_velocity_markers_;

  std::unique_ptr<tf2_ros::Buffer>                                              tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener>                                   tf_listener_;
};

}  // namespace filter_reflective_uavs

#endif
