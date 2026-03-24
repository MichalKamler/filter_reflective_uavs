#include <filter_reflective_uavs/filter_reflective_uavs.h>

#include <tf2/time.h>

namespace filter_reflective_uavs {

FilterReflectiveUavs::FilterReflectiveUavs(rclcpp::NodeOptions options)
: Node("filter_reflective_uavs", options) {
  initialize();
}

void FilterReflectiveUavs::initialize() {
  node_ = this->this_node_ptr();
  clock_ = node_->get_clock();
  cbkgrp_subs_ = this_node().create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(clock_);
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(node_->get_logger(), "Initializing");

  loadParameters();

  mrs_lib::SubscriberHandlerOptions shopts;
  shopts.node = node_;
  shopts.node_name = "FilterReflectiveUavs";
  shopts.threadsafe = true;
  shopts.autostart = true;
  shopts.qos = rclcpp::QoS(rclcpp::KeepLast(10));
  shopts.subscription_options.callback_group = cbkgrp_subs_;

  if (ouster_) {
    sh_pointcloud_ = mrs_lib::SubscriberHandler<PointCloudMsg>(
      shopts, "lidar3d_in", &FilterReflectiveUavs::callbackPointCloudOuster, this);
  } else {
    sh_pointcloud_ = mrs_lib::SubscriberHandler<PointCloudMsg>(
      shopts, "lidar3d_in", &FilterReflectiveUavs::callbackPointCloud, this);
  }

  if (load_gt_uav_positions_) {
    if (detected_uav_names_.empty()) {
      sh_pointcloud_pos_ = mrs_lib::SubscriberHandler<PointCloudMsg>(
        shopts, "uav_positions_in", &FilterReflectiveUavs::pointCloud2PosCallback, this);
      RCLCPP_INFO(node_->get_logger(), "GT injection source: uav_positions_in topic");
    } else {
      RCLCPP_INFO(
        node_->get_logger(),
        "GT injection source: TF frames of %zu configured UAVs",
        detected_uav_names_.size());
    }
  }

  sh_odom_ = mrs_lib::SubscriberHandler<OdomMsg>(
    shopts, "/" + uav_name_ + "/estimation_manager/odom_main", &FilterReflectiveUavs::odomCallback, this);

  mrs_lib::PublisherHandlerOptions popts(node_);
  popts.qos = rclcpp::QoS(10).transient_local();

  publisher_pointcloud_reflective_centroids_ =
    mrs_lib::PublisherHandler<PointCloudMsg>(node_, "~/reflective_centroids_out");
  publisher_estimates_ = mrs_lib::PublisherHandler<PoseArrayMsg>(node_, "~/estimates");
  pub_pointcloud_ = mrs_lib::PublisherHandler<PointCloudMsg>(popts, "~/filtered_pcl");
  pub_pointcloud_removed_ = mrs_lib::PublisherHandler<PointCloudMsg>(popts, "~/removed_pcl");
  pub_seeds_ = mrs_lib::PublisherHandler<PointCloudMsg>(popts, "~/seeds");
  pub_agent_pcl_ = mrs_lib::PublisherHandler<PointCloudMsg>(popts, "~/agents_pcl");
  pub_velocity_markers_ = mrs_lib::PublisherHandler<visualization_msgs::msg::MarkerArray>(popts, "~/velocity_viz");
  pub_pose_vel_array_ = mrs_lib::PublisherHandler<filter_reflective_uavs::msg::PoseVelocityArray>(popts, "~/pose_velocity_array");

  last_update_ = std::chrono::steady_clock::now();
  is_initialized_ = true;
  RCLCPP_INFO(node_->get_logger(), "Initialization completed");
}

void FilterReflectiveUavs::loadParameters() {
  node_->declare_parameter<std::string>("config", "");
  node_->declare_parameter<std::string>("custom_config", "");
  node_->declare_parameter<std::string>("uav_name", "");
  node_->declare_parameter<std::string>("global_frame", "world");
  node_->declare_parameter<std::vector<std::string>>("detected_uav_names", std::vector<std::string>{});

  mrs_lib::ParamLoader param_loader(node_);
  param_loader.addYamlFileFromParam("config");
  param_loader.addYamlFileFromParam("custom_config");

  param_loader.loadParam("uav_name", uav_name_);
  param_loader.loadParam("global_frame", global_frame_);
  param_loader.loadParam("filter_reflective_uavs/ros_parameters/min_intensity", min_intensity_);
  param_loader.loadParam("filter_reflective_uavs/ros_parameters/max_intensity", max_intensity_);
  param_loader.loadParam("filter_reflective_uavs/ros_parameters/reflective_clusters/tolerance", reflective_clustering_tolerance_);
  param_loader.loadParam("filter_reflective_uavs/ros_parameters/reflective_clusters/min_points", reflective_clustering_min_points_);
  param_loader.loadParam("filter_reflective_uavs/ros_parameters/reflective_clusters/max_points", reflective_clustering_max_points_);
  param_loader.loadParam("filter_reflective_uavs/ros_parameters/keep_cloud_dt", keep_cloud_dt_);
  param_loader.loadParam("filter_reflective_uavs/ros_parameters/multi_uav_tracker/dt", dt_);
  param_loader.loadParam("filter_reflective_uavs/ros_parameters/multi_uav_tracker/max_no_update", max_no_update_);
  param_loader.loadParam("filter_reflective_uavs/ros_parameters/multi_uav_tracker/gate_treshold", gate_treshold_);
  param_loader.loadParam("filter_reflective_uavs/ros_parameters/voxel_grid/use", use_voxel_grid_);
  param_loader.loadParam("filter_reflective_uavs/ros_parameters/voxel_grid/size_x", voxel_grid_size_x_);
  param_loader.loadParam("filter_reflective_uavs/ros_parameters/voxel_grid/size_y", voxel_grid_size_y_);
  param_loader.loadParam("filter_reflective_uavs/ros_parameters/voxel_grid/size_z", voxel_grid_size_z_);
  param_loader.loadParam("filter_reflective_uavs/ros_parameters/search_radius", search_radius_);
  param_loader.loadParam("filter_reflective_uavs/ros_parameters/max_distance_from_seed", max_distance_from_seed_);
  param_loader.loadParam("filter_reflective_uavs/ros_parameters/max_removed_points", max_removed_points_);
  param_loader.loadParam("filter_reflective_uavs/ros_parameters/ouster", ouster_);
  param_loader.loadParam("filter_reflective_uavs/ros_parameters/load_gt_uav_positions", load_gt_uav_positions_);
  param_loader.loadParam("filter_reflective_uavs/ros_parameters/time_keep", time_keep_);
  param_loader.loadParam("filter_reflective_uavs/ros_parameters/filter_out_myself/enabled", filter_out_myself_enabled_);
  param_loader.loadParam("filter_reflective_uavs/ros_parameters/filter_out_myself/dist", filter_out_myself_dist_);
  param_loader.loadParam("detected_uav_names", detected_uav_names_);

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "failed to load non-optional parameters!");
    rclcpp::shutdown();
    return;
  }
}

void FilterReflectiveUavs::callbackPointCloudOuster(const PointCloudMsgPtr msg) {
  if (!is_initialized_) {
    return;
  }

  const std::string frame_id = msg->header.frame_id;
  const rclcpp::Time timestamp(msg->header.stamp);

  pcl::PointCloud<ouster_ros::Point>::Ptr ouster_cloud =
    std::make_shared<pcl::PointCloud<ouster_ros::Point>>();
  pcl::fromROSMsg(*msg, *ouster_cloud);

  auto xyzi_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  xyzi_cloud->is_dense = ouster_cloud->is_dense;
  size_t excluded_myself_points = 0;
  if (filter_out_myself_enabled_) {
    const float min_sq_dist_from_sensor =
      static_cast<float>(filter_out_myself_dist_ * filter_out_myself_dist_);
    xyzi_cloud->points.reserve(ouster_cloud->points.size());

    for (const auto & ouster_point : ouster_cloud->points) {
      const float sq_dist_from_sensor =
        ouster_point.x * ouster_point.x +
        ouster_point.y * ouster_point.y +
        ouster_point.z * ouster_point.z;

      if (sq_dist_from_sensor < min_sq_dist_from_sensor) {
        ++excluded_myself_points;
        continue;
      }

      pcl::PointXYZI xyzi_point;
      xyzi_point.x = ouster_point.x;
      xyzi_point.y = ouster_point.y;
      xyzi_point.z = ouster_point.z;
      xyzi_point.intensity = ouster_point.reflectivity;
      xyzi_cloud->points.push_back(xyzi_point);
    }

    xyzi_cloud->width = static_cast<uint32_t>(xyzi_cloud->points.size());
    xyzi_cloud->height = 1;
  } else {
    xyzi_cloud->width = ouster_cloud->width;
    xyzi_cloud->height = ouster_cloud->height;
    xyzi_cloud->points.resize(ouster_cloud->points.size());

    for (size_t i = 0; i < ouster_cloud->points.size(); ++i) {
      const auto & ouster_point = ouster_cloud->points[i];
      pcl::PointXYZI xyzi_point;
      xyzi_point.x = ouster_point.x;
      xyzi_point.y = ouster_point.y;
      xyzi_point.z = ouster_point.z;
      xyzi_point.intensity = ouster_point.reflectivity;
      xyzi_cloud->points[i] = xyzi_point;
    }
  }

  if (filter_out_myself_enabled_) {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *clock_, 1000,
      "Excluded %zu self points from Ouster cloud",
      excluded_myself_points);
  }

  if (load_gt_uav_positions_) {
    injectGtUavPositions(xyzi_cloud, frame_id, timestamp);
  }

  centroid_positions_ = clusterToCentroids(xyzi_cloud, timestamp, frame_id);
  auto centroid_positions_global =
    transfromAndPublishCentroids(centroid_positions_, frame_id, timestamp);

  collected_centroid_positions_.insert(
    collected_centroid_positions_.end(),
    centroid_positions_global.begin(),
    centroid_positions_global.end());

  const auto now = std::chrono::steady_clock::now();
  const double time_elapsed =
    std::chrono::duration<double>(now - last_update_).count();

  if (time_elapsed > dt_) {
    const auto last_centroid_positions_global =
      filterLatestDetections(collected_centroid_positions_, 0.8);
    collected_centroid_positions_.clear();
    update(last_centroid_positions_global);
    publishEstimates(global_frame_, timestamp, tracks_);
    publishVelAsArrow(global_frame_, timestamp, tracks_);
    last_update_ = now;
  }

  filterOutUavs(xyzi_cloud, frame_id, timestamp, tracks_);
}

void FilterReflectiveUavs::callbackPointCloud(const PointCloudMsgPtr msg) {
  if (!is_initialized_) {
    return;
  }

  auto pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::fromROSMsg(*msg, *pcl_cloud);

  const std::string frame_id = msg->header.frame_id;
  const rclcpp::Time timestamp(msg->header.stamp);

  size_t excluded_myself_points = 0;
  if (filter_out_myself_enabled_) {
    const float min_sq_dist_from_sensor =
      static_cast<float>(filter_out_myself_dist_ * filter_out_myself_dist_);
    size_t write_idx = 0;

    for (size_t i = 0; i < pcl_cloud->points.size(); ++i) {
      const auto & point = pcl_cloud->points[i];
      const float sq_dist_from_sensor =
        point.x * point.x +
        point.y * point.y +
        point.z * point.z;

      if (sq_dist_from_sensor < min_sq_dist_from_sensor) {
        ++excluded_myself_points;
        continue;
      }

      if (write_idx != i) {
        pcl_cloud->points[write_idx] = point;
      }
      ++write_idx;
    }

    if (write_idx != pcl_cloud->points.size()) {
      pcl_cloud->points.resize(write_idx);
      pcl_cloud->width = static_cast<uint32_t>(write_idx);
      pcl_cloud->height = 1;
    }

    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *clock_, 1000,
      "Excluded %zu self points from input cloud",
      excluded_myself_points);
  }

  if (load_gt_uav_positions_) {
    injectGtUavPositions(pcl_cloud, frame_id, timestamp);
  }

  centroid_positions_ = clusterToCentroids(pcl_cloud, timestamp, frame_id);
  auto centroid_positions_global =
    transfromAndPublishCentroids(centroid_positions_, frame_id, timestamp);

  collected_centroid_positions_.insert(
    collected_centroid_positions_.end(),
    centroid_positions_global.begin(),
    centroid_positions_global.end());

  const auto now = std::chrono::steady_clock::now();
  const double time_elapsed =
    std::chrono::duration<double>(now - last_update_).count();

  if (time_elapsed > dt_) {
    const auto last_centroid_positions_global =
      filterLatestDetections(collected_centroid_positions_, 0.8);
    collected_centroid_positions_.clear();
    update(last_centroid_positions_global);
    publishEstimates(global_frame_, timestamp, tracks_);
    publishVelAsArrow(global_frame_, timestamp, tracks_);
    last_update_ = now;
  }

  filterOutUavs(pcl_cloud, frame_id, timestamp, tracks_);
}

std::vector<FilterReflectiveUavs::StampPositionPair>
FilterReflectiveUavs::filterLatestDetections(
  const std::vector<StampPositionPair> & collected_centroid_positions,
  double search_radius) const {
  std::vector<StampPositionPair> filtered_output;
  const double search_radius_sq = search_radius * search_radius;

  auto sorted_input = collected_centroid_positions;
  std::sort(
    sorted_input.begin(), sorted_input.end(),
    [](const auto & a, const auto & b) {
      return a.first < b.first;
    });

  for (int i = static_cast<int>(sorted_input.size()) - 1; i >= 0; --i) {
    const auto & current_point = sorted_input[static_cast<size_t>(i)];
    bool is_covered = false;
    for (const auto & retained_point : filtered_output) {
      const double distance_sq =
        (current_point.second - retained_point.second).squaredNorm();
      if (distance_sq <= search_radius_sq) {
        is_covered = true;
        break;
      }
    }

    if (!is_covered) {
      filtered_output.push_back(current_point);
    }
  }

  std::reverse(filtered_output.begin(), filtered_output.end());
  return filtered_output;
}

std::vector<FilterReflectiveUavs::StampPositionPair>
FilterReflectiveUavs::transfromAndPublishCentroids(
  const std::vector<StampPositionPair> & centroid_positions,
  const std::string & frame_id,
  const rclcpp::Time & timestamp) {
  auto cloud_reflective_centroids = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  std::vector<StampPositionPair> centroid_positions_global;

  for (const auto & centroid_pair : centroid_positions) {
    const auto transformed_point = transformPoint(
      centroid_pair.second, frame_id, global_frame_, centroid_pair.first);

    if (transformed_point.has_value()) {
      cloud_reflective_centroids->push_back(
        pcl::PointXYZ(
          static_cast<float>(transformed_point->x()),
          static_cast<float>(transformed_point->y()),
          static_cast<float>(transformed_point->z())));
      centroid_positions_global.emplace_back(centroid_pair.first, *transformed_point);
    } else {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *clock_, 1000,
        "Failed to transform a centroid to the global frame");
    }
  }

  PointCloudMsg cloud_msg;
  pcl::toROSMsg(*cloud_reflective_centroids, cloud_msg);
  cloud_msg.header.frame_id = global_frame_;
  cloud_msg.header.stamp = timestamp;
  publisher_pointcloud_reflective_centroids_.publish(cloud_msg);
  return centroid_positions_global;
}

void FilterReflectiveUavs::update(const std::vector<StampPositionPair> & measurements) {
  const rclcpp::Time current_time =
    measurements.empty() ? node_->now() : measurements.front().first;

  for (auto & track : tracks_) {
    predictTrack(track);
  }

  std::vector<bool> used(measurements.size(), false);
  for (size_t i = 0; i < measurements.size(); ++i) {
    int track_id = -1;
    if (associateMeasurement(measurements[i].second, track_id)) {
      updateTrack(tracks_[static_cast<size_t>(track_id)], measurements[i].second);
      tracks_[static_cast<size_t>(track_id)].last_update = measurements[i].first;
      used[i] = true;
    }
  }

  for (size_t i = 0; i < measurements.size(); ++i) {
    if (!used[i]) {
      initializeTrack(measurements[i].first, measurements[i].second);
    }
  }

  deleteStaleTracks(current_time);
}

bool FilterReflectiveUavs::associateMeasurement(const Eigen::Vector3d & meas, int & track_id) const {
  double min_dist = gate_treshold_;
  int best_track = -1;

  for (size_t i = 0; i < tracks_.size(); ++i) {
    const Eigen::Vector3d pred_pos = tracks_[i].x.head<3>();
    const double dist = (meas - pred_pos).norm();

    if (dist < min_dist) {
      min_dist = dist;
      best_track = static_cast<int>(i);
    }
  }

  if (best_track != -1) {
    track_id = best_track;
    return true;
  }
  return false;
}

void FilterReflectiveUavs::initializeTrack(
  const rclcpp::Time & stamp, const Eigen::Vector3d & meas) {
  Track new_track;
  new_track.id = tracks_.empty() ? 0 : tracks_.back().id + 1;
  new_track.last_update = stamp;
  new_track.x = Eigen::VectorXd::Zero(9);
  new_track.x.head<3>() = meas;
  new_track.P = Eigen::MatrixXd::Identity(9, 9);
  tracks_.push_back(new_track);
}

void FilterReflectiveUavs::predictTrack(Track & track) const {
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(9, 9);
  F(0, 3) = dt_;
  F(1, 4) = dt_;
  F(2, 5) = dt_;
  F(0, 6) = 0.5 * dt_ * dt_;
  F(1, 7) = 0.5 * dt_ * dt_;
  F(2, 8) = 0.5 * dt_ * dt_;
  F(3, 6) = dt_;
  F(4, 7) = dt_;
  F(5, 8) = dt_;

  const Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(9, 9) * 0.01;
  track.x = F * track.x;
  track.P = F * track.P * F.transpose() + Q;
}

void FilterReflectiveUavs::updateTrack(Track & track, const Eigen::Vector3d & meas) const {
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 9);
  H(0, 0) = 1.0;
  H(1, 1) = 1.0;
  H(2, 2) = 1.0;

  const Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * 0.05;
  const Eigen::Vector3d y = meas - H * track.x;
  const Eigen::Matrix3d S = H * track.P * H.transpose() + R;
  const Eigen::MatrixXd K = track.P * H.transpose() * S.inverse();
  track.x = track.x + K * y;

  const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(9, 9);
  track.P = (I - K * H) * track.P * (I - K * H).transpose() + K * R * K.transpose();
}

void FilterReflectiveUavs::deleteStaleTracks(const rclcpp::Time & current_time) {
  tracks_.erase(
    std::remove_if(
      tracks_.begin(), tracks_.end(),
      [&](const Track & track) {
        return (current_time - track.last_update).seconds() > max_no_update_;
      }),
    tracks_.end());
}

void FilterReflectiveUavs::publishVelAsArrow(
  const std::string & frame_id,
  const rclcpp::Time & timestamp,
  const std::vector<Track> & tracks) {
  visualization_msgs::msg::Marker base_marker;
  base_marker.header.frame_id = frame_id;
  base_marker.header.stamp = timestamp;
  base_marker.ns = "velocity_viz";
  base_marker.action = visualization_msgs::msg::Marker::ADD;
  base_marker.type = visualization_msgs::msg::Marker::ARROW;
  base_marker.color.a = 1.0;
  base_marker.color.r = 1.0;
  base_marker.scale.x = 0.05;
  base_marker.scale.y = 0.1;

  visualization_msgs::msg::MarkerArray marker_array_msg;
  for (const auto & track : tracks) {
    visualization_msgs::msg::Marker arrow_marker = base_marker;
    arrow_marker.id = track.id;

    const Eigen::Vector3d pos = track.x.segment<3>(0);
    const Eigen::Vector3d vel = track.x.segment<3>(3);

    geometry_msgs::msg::Point start_point;
    start_point.x = pos.x();
    start_point.y = pos.y();
    start_point.z = pos.z();

    geometry_msgs::msg::Point end_point;
    end_point.x = pos.x() + vel.x() * 3.0;
    end_point.y = pos.y() + vel.y() * 3.0;
    end_point.z = pos.z() + vel.z() * 3.0;

    arrow_marker.points.push_back(start_point);
    arrow_marker.points.push_back(end_point);
    marker_array_msg.markers.push_back(arrow_marker);
  }

  pub_velocity_markers_.publish(marker_array_msg);
}

void FilterReflectiveUavs::publishEstimates(
  const std::string & frame_id,
  const rclcpp::Time & timestamp,
  const std::vector<Track> & tracks) {
  PoseArrayMsg msg_estimates;
  msg_estimates.header.frame_id = frame_id;
  msg_estimates.header.stamp = timestamp;

  filter_reflective_uavs::msg::PoseVelocityArray msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = timestamp;

  for (const auto & track : tracks) {
    msg.ids.push_back(track.id);
    const Eigen::Vector3d pos = track.x.segment<3>(0);
    const Eigen::Vector3d vel = track.x.segment<3>(3);

    geometry_msgs::msg::Pose pose;
    pose.position.x = pos.x();
    pose.position.y = pos.y();
    pose.position.z = pos.z();
    pose.orientation.w = 1.0;

    geometry_msgs::msg::Vector3 velocity;
    velocity.x = vel.x();
    velocity.y = vel.y();
    velocity.z = vel.z();

    msg.poses.push_back(pose);
    msg.velocities.push_back(velocity);

    mrs_msgs::msg::PoseWithCovarianceIdentified msg_est;
    msg_est.id = track.id;
    const Eigen::Matrix3d pos_cov = track.P.block<3, 3>(0, 0);
    msg_est.pose = poseToMsg(pos, vel);
    covarianceToMsg(pos_cov, msg_est.covariance);
    msg_estimates.poses.push_back(msg_est);
  }

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *clock_, 1000,
    "Publishing %zu UAV estimates", tracks.size());

  pub_pose_vel_array_.publish(msg);
  publisher_estimates_.publish(msg_estimates);
}

std::vector<FilterReflectiveUavs::StampPositionPair>
FilterReflectiveUavs::clusterToCentroids(
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
  const rclcpp::Time & timestamp,
  const std::string & frame_id) const {
  (void)frame_id;
  std::vector<StampPositionPair> centroid_positions;

  if (cloud->empty()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "Input pointcloud is empty");
    return centroid_positions;
  }

  auto cloud_reflective = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::ConditionAnd<pcl::PointXYZI>::Ptr reflectivity_cond(
    new pcl::ConditionAnd<pcl::PointXYZI>());
  reflectivity_cond->addComparison(
    pcl::FieldComparison<pcl::PointXYZI>::Ptr(
      new pcl::FieldComparison<pcl::PointXYZI>(
        "intensity", pcl::ComparisonOps::GT, static_cast<float>(min_intensity_))));

  pcl::ConditionalRemoval<pcl::PointXYZI> reflectivity_filt;
  reflectivity_filt.setCondition(reflectivity_cond);
  reflectivity_filt.setKeepOrganized(false);
  reflectivity_filt.setInputCloud(cloud);
  reflectivity_filt.filter(*cloud_reflective);

  if (cloud_reflective->empty()) {
    return centroid_positions;
  }

  if (use_voxel_grid_) {
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(cloud);
    vg.setDownsampleAllData(false);
    vg.setLeafSize(
      static_cast<float>(voxel_grid_size_x_),
      static_cast<float>(voxel_grid_size_y_),
      static_cast<float>(voxel_grid_size_z_));
    vg.filter(*cloud);
  }

  auto tree_reflective = std::make_shared<pcl::search::KdTree<pcl::PointXYZI>>();
  tree_reflective->setInputCloud(cloud_reflective);
  const auto cluster_indices = doEuclideanClustering(
    tree_reflective, cloud_reflective,
    static_cast<float>(reflective_clustering_tolerance_),
    reflective_clustering_min_points_,
    reflective_clustering_max_points_);

  std::vector<pcl::PointXYZ> centroids_reflective;
  calculateCentroid2(cloud_reflective, cluster_indices, centroids_reflective);

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *clock_, 1000,
    "Found %zu reflective centroids", centroids_reflective.size());

  for (const auto & pt : centroids_reflective) {
    centroid_positions.emplace_back(timestamp, Eigen::Vector3d(pt.x, pt.y, pt.z));
  }

  return centroid_positions;
}

void FilterReflectiveUavs::calculateCentroid2(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
  const std::vector<pcl::PointIndices> & cluster_indices,
  std::vector<pcl::PointXYZ> & result) const {
  for (const auto & cluster : cluster_indices) {
    pcl::CentroidPoint<pcl::PointXYZ> centroid;
    for (const auto & index : cluster.indices) {
      pcl::PointXYZ pt;
      pt.x = cloud->at(static_cast<size_t>(index)).x;
      pt.y = cloud->at(static_cast<size_t>(index)).y;
      pt.z = cloud->at(static_cast<size_t>(index)).z;
      centroid.add(pt);
    }
    pcl::PointXYZ center_pt;
    centroid.get(center_pt);
    result.push_back(center_pt);
  }
}

std::vector<pcl::PointIndices> FilterReflectiveUavs::doEuclideanClustering(
  const pcl::search::KdTree<pcl::PointXYZI>::Ptr tree_orig,
  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
  float clustering_tolerance,
  int min_points,
  int max_points,
  const pcl::IndicesConstPtr indices_within_radius) const {
  std::vector<pcl::PointIndices> ret;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(clustering_tolerance);
  ec.setMinClusterSize(min_points);
  ec.setMaxClusterSize(max_points);
  ec.setSearchMethod(tree_orig);
  ec.setInputCloud(cloud);
  if (indices_within_radius != nullptr) {
    ec.setIndices(indices_within_radius);
  }
  ec.extract(ret);
  return ret;
}

void FilterReflectiveUavs::filterOutUavs(
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud,
  const std::string & frame_id,
  const rclcpp::Time & timestamp,
  const std::vector<Track> & tracks) {
  std::vector<int> seed_indices;
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;

  for (const auto & track : tracks) {
    const Eigen::Vector3d neigh_pos(track.x[0], track.x[1], track.x[2]);
    const auto transformed_point =
      transformPoint(neigh_pos, global_frame_, frame_id, timestamp);

    if (!transformed_point.has_value()) {
      continue;
    }

    pcl::PointXYZI p;
    p.x = static_cast<float>(transformed_point->x());
    p.y = static_cast<float>(transformed_point->y());
    p.z = static_cast<float>(transformed_point->z());
    p.intensity = 255.0f;

    const int current_index = static_cast<int>(pcl_cloud->points.size());
    pcl_cloud->points.push_back(p);
    pcl_cloud->width = static_cast<uint32_t>(pcl_cloud->points.size());
    pcl_cloud->height = 1;
    seed_indices.push_back(current_index);
  }

  if (pcl_cloud->points.empty()) {
    PointCloudMsg output_msg;
    pcl::toROSMsg(*pcl_cloud, output_msg);
    output_msg.header.frame_id = frame_id;
    output_msg.header.stamp = timestamp;
    pub_pointcloud_.publish(output_msg);
    return;
  }

  kdtree.setInputCloud(pcl_cloud);
  std::vector<bool> is_uav_point(pcl_cloud->points.size(), false);
  const float max_sq_distance_from_seed =
    static_cast<float>(max_distance_from_seed_ * max_distance_from_seed_);

  struct QueueElement {
    int idx;
    float sq_dist_from_seed;
  };

  for (const int idx_seed : seed_indices) {
    std::queue<QueueElement> q;
    q.push({idx_seed, 0.0f});
    const auto seed = pcl_cloud->points[static_cast<size_t>(idx_seed)];
    is_uav_point[static_cast<size_t>(idx_seed)] = true;

    while (!q.empty()) {
      const auto current_element = q.front();
      q.pop();
      std::vector<int> neighbors;
      std::vector<float> sqr_distances_to_neighbor;
      (void)current_element.sq_dist_from_seed;

      kdtree.radiusSearch(
        pcl_cloud->points[static_cast<size_t>(current_element.idx)],
        search_radius_,
        neighbors,
        sqr_distances_to_neighbor);

      for (size_t i = 0; i < neighbors.size(); ++i) {
        const int neighbor_idx = neighbors[i];
        const auto neighbor_point = pcl_cloud->points[static_cast<size_t>(neighbor_idx)];
        const float dx = neighbor_point.x - seed.x;
        const float dy = neighbor_point.y - seed.y;
        const float dz = neighbor_point.z - seed.z;
        const float sq_dist_to_seed = dx * dx + dy * dy + dz * dz;

        if (sq_dist_to_seed > max_sq_distance_from_seed ||
          is_uav_point[static_cast<size_t>(neighbor_idx)])
        {
          continue;
        }

        is_uav_point[static_cast<size_t>(neighbor_idx)] = true;
        q.push({neighbor_idx, sq_dist_to_seed});
      }
    }
  }

  auto environment_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  auto uav_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

  for (size_t i = 0; i < pcl_cloud->points.size(); ++i) {
    if (!is_uav_point[i]) {
      environment_cloud->points.push_back(pcl_cloud->points[i]);
    } else {
      uav_cloud->points.push_back(pcl_cloud->points[i]);
    }
  }

  PointCloudMsg output_msg;
  pcl::toROSMsg(*environment_cloud, output_msg);
  output_msg.header.frame_id = frame_id;
  output_msg.header.stamp = timestamp;
  pub_pointcloud_.publish(output_msg);

  PointCloudMsg output_msg_removed_points;
  pcl::toROSMsg(*uav_cloud, output_msg_removed_points);
  output_msg_removed_points.header.frame_id = frame_id;
  output_msg_removed_points.header.stamp = timestamp;
  pub_pointcloud_removed_.publish(output_msg_removed_points);

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *clock_, 1000,
    "Removed %zu points from input cloud, filtered cloud has %zu points",
    uav_cloud->points.size(), environment_cloud->points.size());

  auto seed_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  {
    std::shared_lock<std::shared_mutex> lock(uav_positions_mutex_);
    for (const auto & uav_pos : uav_positions_) {
      pcl::PointXYZI p;
      p.x = static_cast<float>(uav_pos.second.x());
      p.y = static_cast<float>(uav_pos.second.y());
      p.z = static_cast<float>(uav_pos.second.z());
      p.intensity = 255.0f;
      seed_cloud->points.push_back(p);
    }
  }

  PointCloudMsg output_msg_seeds;
  pcl::toROSMsg(*seed_cloud, output_msg_seeds);
  output_msg_seeds.header.frame_id = frame_id;
  output_msg_seeds.header.stamp = timestamp;
  pub_seeds_.publish(output_msg_seeds);
}

geometry_msgs::msg::Pose FilterReflectiveUavs::poseToMsg(
  const Eigen::Vector3d & position,
  const Eigen::Vector3d & velocity) const {
  geometry_msgs::msg::Pose msg;
  msg.position.x = position.x();
  msg.position.y = position.y();
  msg.position.z = position.z();

  if (velocity.norm() < 1e-6) {
    msg.orientation.w = 1.0;
    return msg;
  }

  const Eigen::Vector3d x_axis(1.0, 0.0, 0.0);
  const Eigen::Quaterniond eigen_quat =
    Eigen::Quaterniond::FromTwoVectors(x_axis, velocity.normalized());
  msg.orientation.x = eigen_quat.x();
  msg.orientation.y = eigen_quat.y();
  msg.orientation.z = eigen_quat.z();
  msg.orientation.w = eigen_quat.w();
  return msg;
}

void FilterReflectiveUavs::covarianceToMsg(
  const Eigen::Matrix3d & cov,
  std::array<double, 36> & msg_cov_out) const {
  msg_cov_out.fill(0.0);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      msg_cov_out[static_cast<size_t>(6 * i + j)] = cov(i, j);
    }
  }
}

void FilterReflectiveUavs::pointCloud2PosCallback(const PointCloudMsgPtr msg) {
  std::unique_lock<std::shared_mutex> lock(uav_positions_mutex_);

  const rclcpp::Time now_time = node_->now();
  const rclcpp::Time pose_time(msg->header.stamp);
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg(*msg, pcl_cloud);

  for (const auto & point : pcl_cloud.points) {
    uav_positions_.emplace_back(pose_time, Eigen::Vector3d(point.x, point.y, point.z));
  }

  uav_positions_.erase(
    std::remove_if(
      uav_positions_.begin(), uav_positions_.end(),
      [&](const auto & entry) {
        return (now_time - entry.first).seconds() > time_keep_;
      }),
    uav_positions_.end());

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *clock_, 1000,
    "Received %zu GT UAV positions on uav_positions_in, cached %zu positions",
    pcl_cloud.points.size(), uav_positions_.size());
}

void FilterReflectiveUavs::odomCallback(const OdomMsgPtr msg) {
  agent_pos_ = Eigen::Vector3d(
    msg->pose.pose.position.x,
    msg->pose.pose.position.y,
    msg->pose.pose.position.z);
}

void FilterReflectiveUavs::injectGtUavPositions(
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud,
  const std::string & frame_id,
  const rclcpp::Time & timestamp) const {
  if (!detected_uav_names_.empty()) {
    size_t injected_count = 0;
    size_t transform_failed_count = 0;

    for (const auto & detected_uav_name : detected_uav_names_) {
      if (detected_uav_name == uav_name_) {
        continue;
      }

      const auto transformed_point = transformPoint(
        Eigen::Vector3d::Zero(), detected_uav_name + "/fcu", frame_id, timestamp);

      if (!transformed_point.has_value()) {
        ++transform_failed_count;
        continue;
      }

      pcl::PointXYZI p;
      p.x = static_cast<float>(transformed_point->x());
      p.y = static_cast<float>(transformed_point->y());
      p.z = static_cast<float>(transformed_point->z());
      p.intensity = static_cast<float>(max_intensity_);

      pcl_cloud->points.push_back(p);
      pcl_cloud->width = static_cast<uint32_t>(pcl_cloud->points.size());
      pcl_cloud->height = 1;
      ++injected_count;
    }

    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *clock_, 1000,
      "Injected %zu GT UAV points from TF into cloud for frame %s (%zu TF failures, %zu configured UAVs)",
      injected_count, frame_id.c_str(), transform_failed_count, detected_uav_names_.size());
    return;
  }

  std::shared_lock<std::shared_mutex> lock(uav_positions_mutex_);

  if (uav_positions_.empty()) {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *clock_, 1000,
      "GT injection skipped: no cached UAV positions available on uav_positions_in");
    return;
  }

  size_t injected_count = 0;
  size_t transform_failed_count = 0;

  for (const auto & uav_pos : uav_positions_) {
    const auto transformed_point =
      transformPoint(uav_pos.second, global_frame_, frame_id, timestamp);

    if (!transformed_point.has_value()) {
      ++transform_failed_count;
      continue;
    }

    pcl::PointXYZI p;
    p.x = static_cast<float>(transformed_point->x());
    p.y = static_cast<float>(transformed_point->y());
    p.z = static_cast<float>(transformed_point->z());
    p.intensity = static_cast<float>(max_intensity_);

    pcl_cloud->points.push_back(p);
    pcl_cloud->width = static_cast<uint32_t>(pcl_cloud->points.size());
    pcl_cloud->height = 1;
    ++injected_count;
  }

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *clock_, 1000,
    "Injected %zu GT UAV points into cloud for frame %s (%zu TF failures, %zu cached positions)",
    injected_count, frame_id.c_str(), transform_failed_count, uav_positions_.size());
}

std::optional<geometry_msgs::msg::TransformStamped> FilterReflectiveUavs::lookupTransform(
  const std::string & target_frame,
  const std::string & source_frame,
  const rclcpp::Time & stamp) const {
  try {
    return tf_buffer_->lookupTransform(
      target_frame,
      source_frame,
      stamp,
      tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *clock_, 1000,
      "TF lookup failed from %s to %s: %s",
      source_frame.c_str(), target_frame.c_str(), ex.what());
    return std::nullopt;
  }
}

std::optional<Eigen::Vector3d> FilterReflectiveUavs::transformPoint(
  const Eigen::Vector3d & point,
  const std::string & from_frame,
  const std::string & to_frame,
  const rclcpp::Time & stamp) const {
  const auto transform = lookupTransform(to_frame, from_frame, stamp);
  if (!transform.has_value()) {
    return std::nullopt;
  }

  const auto & rotation = transform->transform.rotation;
  const auto & translation = transform->transform.translation;

  const Eigen::Quaterniond quat(rotation.w, rotation.x, rotation.y, rotation.z);
  const Eigen::Vector3d translated(
    translation.x,
    translation.y,
    translation.z);

  return quat * point + translated;
}

}  // namespace filter_reflective_uavs

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(filter_reflective_uavs::FilterReflectiveUavs)
