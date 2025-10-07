#include <filter_reflective_uavs.h>

namespace filter_reflective_uavs
{

void FilterReflectiveUavs::onInit() {
	const std::string node_name("FilterReflectiveUavs");
	ros::NodeHandle &nh = getPrivateNodeHandle();

	ROS_INFO("[%s]: Initializing", node_name.c_str());
	ros::Time::waitForValid();

	ROS_INFO("[%s]: loading parameters using ParamLoader", node_name.c_str());
	mrs_lib::ParamLoader param_loader(nh, "RBLController");
	param_loader.loadParam("uav_name", _uav_name_);
	param_loader.loadParam("global_frame", _global_frame_);
	param_loader.loadParam("min_intensity", _min_intensity_);
	param_loader.loadParam("max_intensity", _max_intensity_);
	param_loader.loadParam("reflective_clusters/tolerance", _reflective_clustering_tolerance_);
	param_loader.loadParam("reflective_clusters/min_points", _reflective_clustering_min_points_);
  	param_loader.loadParam("reflective_clusters/max_points", _reflective_clustering_max_points_);
	param_loader.loadParam("keep_cloud_dt", _keep_cloud_dt_);
	param_loader.loadParam("multi_uav_tracker/dt", _dt_);
	param_loader.loadParam("multi_uav_tracker/max_no_update", _max_no_update_);
	param_loader.loadParam("multi_uav_tracker/gate_treshold", _gate_treshold_);
	param_loader.loadParam("voxel_grid/use", _use_voxel_grid_);
  	param_loader.loadParam("voxel_grid/size_x", _voxel_grid_size_x_);
  	param_loader.loadParam("voxel_grid/size_y", _voxel_grid_size_y_);
  	param_loader.loadParam("voxel_grid/size_z", _voxel_grid_size_z_);
	param_loader.loadParam("search_radius", _search_radius_);
	param_loader.loadParam("max_distance_from_seed", _max_distance_from_seed_);	
	param_loader.loadParam("max_removed_points", _max_removed_points_);		
	param_loader.loadParam("ouster", _ouster_);
	param_loader.loadParam("load_gt_uav_positions", _load_gt_uav_positions_);
  	param_loader.loadParam("time_keep", _time_keep_);
	if (!param_loader.loadedSuccessfully()) {
    	ROS_ERROR("[%s]: parameter loading failure", node_name.c_str());
    	ros::shutdown();
  	}

	mrs_lib::SubscribeHandlerOptions shopts;
	shopts.nh                 = nh;
	shopts.node_name          = "FilterReflectiveUavs";
	shopts.no_message_timeout = mrs_lib::no_timeout;
	shopts.threadsafe         = true;
	shopts.autostart          = true;
	shopts.queue_size         = 10;
	shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

	if (_ouster_) { //for ouster and for simulation, since ouster is implemented for simulating
		sh_pointcloud_ = mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2>(shopts, "lidar3d_in", ros::Duration(_keep_cloud_dt_), &FilterReflectiveUavs::timeoutGeneric, this, &FilterReflectiveUavs::callbackPointCloudOuster, this);
	} else { //for livox and for real world experiments - due to different data storage
		sh_pointcloud_ = mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2>(shopts, "lidar3d_in", ros::Duration(_keep_cloud_dt_), &FilterReflectiveUavs::timeoutGeneric, this, &FilterReflectiveUavs::callbackPointCloud, this);
	}

	// if (_ouster_) { //for ouster and for simulation, since ouster is implemented for simulating
	// 	sh_pointcloud_ = mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2>(shopts, "lidar3d_in", ros::Duration(1.0), &FilterReflectiveUavs::timeoutGeneric, this, &FilterReflectiveUavs::callbackPointCloudOuster, this);
	// } else { //for livox and for real world experiments - due to different data storage
	// 	sh_pointcloud_ = mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2>(shopts, "lidar3d_in", ros::Duration(1.0), &FilterReflectiveUavs::timeoutGeneric, this, &FilterReflectiveUavs::callbackPointCloud, this);
	// }

	// sh_uav_position_estimation_ = mrs_lib::SubscribeHandler<mrs_msgs::PoseWithCovarianceArrayStamped>(shopts, "estimated_pos", ros::Duration(1.0), &FilterReflectiveUavs::timeoutGeneric, this, &FilterReflectiveUavs::callbackPoses, this);

	sub_pointCloud2_pos_  = nh.subscribe("estimated_pos", 1, &FilterReflectiveUavs::pointCloud2PosCallback, this);
	sub_odom_  = nh.subscribe("/" + _uav_name_ + "/estimation_manager/odom_main", 10, &FilterReflectiveUavs::odomCallback, this);

	publisher_pointcloud_reflective_centroids_ = nh.advertise<sensor_msgs::PointCloud2>("reflective_centroids_out", 10);
	publisher_estimates_ = nh.advertise<mrs_msgs::PoseWithCovarianceArrayStamped>("estimates", 10);
	pub_pointCloud_ = nh.advertise<sensor_msgs::PointCloud2>("filtered_pcl", 10, true);
	pub_pointCloud_removed_ = nh.advertise<sensor_msgs::PointCloud2>("removed_pcl", 10, true);
	pub_seeds_ = nh.advertise<sensor_msgs::PointCloud2>("seeds", 10, true);
	pub_agent_pcl_  = nh.advertise<sensor_msgs::PointCloud2>("agents_pcl", 10, true);
	pub_velocity_markers_ = nh.advertise<visualization_msgs::MarkerArray>("velocity_viz", 10, true);

	pub_pose_vel_array_ = nh.advertise<filter_reflective_uavs::PoseVelocityArray>("pose_velocity_array", 10, true);


	transformer_ = mrs_lib::Transformer("FilterReflectiveUavs");
  	transformer_.setLookupTimeout(ros::Duration(0.1));

	_last_update_ = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();

	is_initialized_ = true;
	ROS_INFO("[%s]: Initialization completed.", node_name.c_str());
};

void FilterReflectiveUavs::callbackPointCloudOuster(const sensor_msgs::PointCloud2::ConstPtr msg) {
	std::string frame_id = msg->header.frame_id;
    ros::Time timestamp = msg->header.stamp;

	if (!is_initialized_) {
    	return;
  	}

	pcl::PointCloud<ouster_ros::Point>::Ptr ouster_cloud = boost::make_shared<pcl::PointCloud<ouster_ros::Point>>();
  	pcl::fromROSMsg(*msg, *ouster_cloud);

	pcl::PointCloud<pcl::PointXYZI>::Ptr xyzi_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

	//covert the clouds
    xyzi_cloud->width = ouster_cloud->width;
    xyzi_cloud->height = ouster_cloud->height;
    xyzi_cloud->is_dense = ouster_cloud->is_dense;
    xyzi_cloud->points.resize(ouster_cloud->points.size());

    for (size_t i = 0; i < ouster_cloud->points.size(); ++i) {
        const auto& ouster_point = ouster_cloud->points[i];
        pcl::PointXYZI xyzi_point;

        xyzi_point.x = ouster_point.x;
        xyzi_point.y = ouster_point.y;
        xyzi_point.z = ouster_point.z;
        xyzi_point.intensity = ouster_point.reflectivity;
        xyzi_cloud->points[i] = xyzi_point;
    }

	//fake the reflectivity for simulation
	std::vector<std::string> uav_list_ = {"uav1", "uav2", "uav3", "uav4"};
	for (size_t i = 0; i < uav_list_.size(); ++i) {
		const std::string& uav_name = uav_list_[i];
		if (uav_name == _uav_name_) {
			continue;
		}
		std::string uav_frame = uav_name + "/fcu";

		auto tf = transformer_.getTransform(uav_frame, frame_id, timestamp);
		if (!tf) {
			ROS_WARN_THROTTLE(5.0, "[FilterReflectiveUavs]: Failed to get TF for %s", uav_frame.c_str());
			continue;
		} else {
			ROS_WARN_THROTTLE(5.0, "[FilterReflectiveUavs]: Good warining - able to get this tf :) %s", uav_frame.c_str());
		}

		pcl::PointXYZI p;
		p.x = tf.value().transform.translation.x;
		p.y = tf.value().transform.translation.y;
		p.z = tf.value().transform.translation.z;
		p.intensity = 255.0f; 

		int current_index = xyzi_cloud->points.size();
		xyzi_cloud->points.push_back(p);
		xyzi_cloud->width = xyzi_cloud->points.size();
		xyzi_cloud->height = 1;
	}

	centroid_positions_ = clusterToCentroids(xyzi_cloud, timestamp, frame_id);

	std::vector<std::pair<ros::Time, Eigen::Vector3d>> centroid_positions_global;
	centroid_positions_global = transfromAndPublishCentroids(centroid_positions_, frame_id, timestamp); 

	for (int i = 0; i < centroid_positions_global.size(); i++) {
		collected_centroid_positions_.push_back(centroid_positions_global[i]);
	}

	double current_time = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    double time_elapsed = current_time - _last_update_;
	std::vector<std::pair<ros::Time, Eigen::Vector3d>> last_centroid_positions_global;
	if (time_elapsed > _dt_) {
		last_centroid_positions_global = filterLatestDetections(collected_centroid_positions_, 0.8); //TODO max_vel*dt
		collected_centroid_positions_.clear();
		update(last_centroid_positions_global);
		publishEstimates(_global_frame_, timestamp, tracks_);
		publishVelAsArrow(_global_frame_, timestamp, tracks_);
		_last_update_ = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
	}	

	
	filterOutUavs(xyzi_cloud, frame_id, timestamp, tracks_);
}

void FilterReflectiveUavs::callbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr msg) { //this one is for livox
	if (!is_initialized_) {
    	return;
  	}

	pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::fromROSMsg(*msg, *pcl_cloud);

	std::string frame_id = msg->header.frame_id;
    ros::Time timestamp = msg->header.stamp;
	
	centroid_positions_ = clusterToCentroids(pcl_cloud, timestamp, frame_id);
	std::vector<std::pair<ros::Time, Eigen::Vector3d>> centroid_positions_global;
	centroid_positions_global = transfromAndPublishCentroids(centroid_positions_, frame_id, timestamp); 

	for (int i = 0; i < centroid_positions_global.size(); i++) {
		collected_centroid_positions_.push_back(centroid_positions_global[i]);
	}

	double current_time = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    double time_elapsed = current_time - _last_update_;
	std::vector<std::pair<ros::Time, Eigen::Vector3d>> last_centroid_positions_global;
	if (time_elapsed > _dt_) {
		last_centroid_positions_global = filterLatestDetections(collected_centroid_positions_, 0.8); //TODO max_vel*dt
		collected_centroid_positions_.clear();
		update(last_centroid_positions_global);
		publishEstimates(_global_frame_, timestamp, tracks_);
		publishVelAsArrow(_global_frame_, timestamp, tracks_);
		_last_update_ = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
	}

	filterOutUavs(pcl_cloud, frame_id, timestamp, tracks_);

}

std::vector<std::pair<ros::Time, Eigen::Vector3d>> FilterReflectiveUavs::filterLatestDetections(
									const std::vector<std::pair<ros::Time, Eigen::Vector3d>>& collected_centroid_positions,
									double search_radius){
    std::vector<std::pair<ros::Time, Eigen::Vector3d>> filtered_output;

    const double search_radius_sq = search_radius * search_radius;

    std::vector<std::pair<ros::Time, Eigen::Vector3d>> sorted_input = collected_centroid_positions;
    std::sort(sorted_input.begin(), sorted_input.end(),
        [](const auto& a, const auto& b) {
            return a.first < b.first;
        });

    for (int i = sorted_input.size() - 1; i >= 0; --i) {
        const auto& current_point = sorted_input[i];
        bool is_covered = false;
        for (const auto& retained_point : filtered_output) {
            double distance_sq = (current_point.second - retained_point.second).squaredNorm();

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

std::vector<std::pair<ros::Time, Eigen::Vector3d>> FilterReflectiveUavs::transfromAndPublishCentroids(std::vector<std::pair<ros::Time, Eigen::Vector3d>> centroid_positions, 
																									  std::string frame_id,
																									  ros::Time timestamp) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_reflective_centroids = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	std::vector<std::pair<ros::Time, Eigen::Vector3d>> centroid_positions_global; 

	for (const auto& centroid_pair : centroid_positions) {
		const ros::Time& centroid_time = centroid_pair.first; // Should be 'timestamp'
		const Eigen::Vector3d& centroid_local = centroid_pair.second;

		auto transformed_point_opt = transformer_.transformAsPoint(
			frame_id,             // from_frame (Lidar frame)
			centroid_local,       // what (the centroid point)
			_global_frame_,       // to_frame (World origin)
			centroid_time         // time_stamp
		);

		if (transformed_point_opt.has_value()) {
			const Eigen::Vector3d& centroid_global = transformed_point_opt.value();

			cloud_reflective_centroids->push_back(
				pcl::PointXYZ(centroid_global.x(), centroid_global.y(), centroid_global.z())
			);
			
			centroid_positions_global.emplace_back(centroid_time, centroid_global);
			
			std::cout << "[Filter]: Detected Centroid transformed (Global Frame): ["
					<< centroid_global.x() << ", "
					<< centroid_global.y() << ", "
					<< centroid_global.z() << "]" << std::endl;

		} else {
			ROS_WARN_THROTTLE(1.0, "[Filter]: Failed to transform a centroid to global frame.");
		}
	}

	cloud_reflective_centroids->header.frame_id = _global_frame_;
	cloud_reflective_centroids->header.stamp = pcl_conversions::toPCL(timestamp);
	publisher_pointcloud_reflective_centroids_.publish(cloud_reflective_centroids);
	return centroid_positions_global;
}	

void FilterReflectiveUavs::update(const std::vector<std::pair<ros::Time, Eigen::Vector3d>>& measurements) {
	ros::Time current_time = measurements.empty() ? ros::Time::now() : measurements.front().first;

    // 1. Predict all tracks forward
    for (auto& track : tracks_) {
        predictTrack(track);
    }

    // 2. Associate measurements to existing tracks
    std::vector<bool> used(measurements.size(), false);
    for (size_t i = 0; i < measurements.size(); ++i) {
        int track_id;
        if (associateMeasurement(measurements[i].second, track_id)) {
            // Found a track
            updateTrack(tracks_[track_id], measurements[i].second);
            tracks_[track_id].last_update = measurements[i].first;
            used[i] = true;
        }
    }

    // 3. Initialize new tracks for unassigned measurements
    for (size_t i = 0; i < measurements.size(); ++i) {
        if (!used[i]) {
            initializeTrack(measurements[i].first, measurements[i].second);
        }
    }

    // 4. Delete stale tracks
    deleteStaleTracks(current_time);
}

bool FilterReflectiveUavs::associateMeasurement(const Eigen::Vector3d& meas, int& track_id) {
	double min_dist = _gate_treshold_;
    int best_track = -1;

    for (size_t i = 0; i < tracks_.size(); ++i) {
        Eigen::Vector3d pred_pos = tracks_[i].x.head<3>();
        double dist = (meas - pred_pos).norm();

        if (dist < min_dist) {
            min_dist = dist;
            best_track = i;
        }
    }

    if (best_track != -1) {
        track_id = best_track;
        return true;
    }
    return false; // No track close enough
}

void FilterReflectiveUavs::initializeTrack(const ros::Time& t, const Eigen::Vector3d& meas) {
	Track new_track;
    new_track.id = tracks_.empty() ? 0 : tracks_.back().id + 1;
    new_track.last_update = t;

    // State: [px, py, pz, vx, vy, vz, ax, ay, az]
    new_track.x = Eigen::VectorXd::Zero(9);
    new_track.x.head<3>() = meas; // initialize position
    new_track.P = Eigen::MatrixXd::Identity(9, 9) * 1.0; // large uncertainty. TODO maybe lower since lidar is precise

    tracks_.push_back(new_track);
}

void FilterReflectiveUavs::predictTrack(Track& track) {
	Eigen::MatrixXd F = Eigen::MatrixXd::Identity(9, 9);
    // position update
	F(0, 3) = _dt_;
	F(1, 4) = _dt_;
	F(2, 5) = _dt_;

	F(0, 6) = 0.5 * _dt_ * _dt_;
	F(1, 7) = 0.5 * _dt_ * _dt_;
	F(2, 8) = 0.5 * _dt_ * _dt_;

	// velocity update
	F(3, 6) = _dt_;
	F(4, 7) = _dt_;
	F(5, 8) = _dt_;

    // Process noise
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(9, 9) * 0.01; // TODO tune

    track.x = F * track.x;
    track.P = F * track.P * F.transpose() + Q;
}

void FilterReflectiveUavs::updateTrack(Track& track, const Eigen::Vector3d& meas) {
	// Measurement matrix
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 9);
    H(0, 0) = 1;
    H(1, 1) = 1;
    H(2, 2) = 1;

	// measurement noise
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * 0.05; // measurement noise, TODO tune. -> sigma = sqrt(0.05) = 0.22 m, which could be apriximately correct? Maybe too much

	// innovation
    Eigen::Vector3d y = meas - H * track.x;
	// innovation covariance
    Eigen::Matrix3d S = H * track.P * H.transpose() + R; //3x3
	// Kalman gain (9x3)
    Eigen::MatrixXd K = track.P * H.transpose() * S.inverse();

	// state update (9x1)
    track.x = track.x + K * y;

	// covariance update
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(9, 9);
    track.P = (I - K * H) * track.P * (I - K * H).transpose() + K * R * K.transpose();
}

void FilterReflectiveUavs::deleteStaleTracks(const ros::Time& current_time) {
	tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),[&](const Track& t) {return (current_time - t.last_update).toSec() > _max_no_update_;}),tracks_.end());
}

void FilterReflectiveUavs::publishVelAsArrow(const std::string& frame_id, const ros::Time& timestamp, std::vector<Track>& tracks) {
    visualization_msgs::Marker velocity_markers;
    velocity_markers.header.frame_id = frame_id;
    velocity_markers.header.stamp = timestamp;
    velocity_markers.ns = "velocity_viz"; 
    velocity_markers.action = visualization_msgs::Marker::ADD;
    velocity_markers.type = visualization_msgs::Marker::ARROW;

    velocity_markers.color.a = 1.0; // Alpha (opacity)
    velocity_markers.color.r = 1.0; // Red 
    velocity_markers.color.g = 0.0;
    velocity_markers.color.b = 0.0;
    velocity_markers.scale.x = 0.05; // Shaft diameter (e.g., 5 cm)
    velocity_markers.scale.y = 0.1;  // Head diameter (e.g., 10 cm)
    std::vector<visualization_msgs::Marker> markers_to_publish;

    int marker_id = 0; 

    for (auto& track : tracks) {
        Eigen::Vector3d pos = track.x.segment<3>(0); // indices 0..2
        Eigen::Vector3d vel = track.x.segment<3>(3); // indices 3..5
        
        geometry_msgs::Pose p;
        p.position.x = pos.x();
        p.position.y = pos.y();
        p.position.z = pos.z();
        p.orientation.w = 1.0;  // identity quaternion

        geometry_msgs::Vector3 v;
        v.x = vel.x();
        v.y = vel.y();
        v.z = vel.z();

        visualization_msgs::Marker arrow_marker = velocity_markers; 
        arrow_marker.id = track.id; 
        
        geometry_msgs::Point start_point;
        start_point.x = pos.x();
        start_point.y = pos.y();
        start_point.z = pos.z();
        
        const double velocity_scale_factor = 3.0; 
        
        geometry_msgs::Point end_point;
        end_point.x = pos.x() + vel.x() * velocity_scale_factor;
        end_point.y = pos.y() + vel.y() * velocity_scale_factor;
        end_point.z = pos.z() + vel.z() * velocity_scale_factor;
        
        arrow_marker.points.push_back(start_point);
        arrow_marker.points.push_back(end_point);
        
        markers_to_publish.push_back(arrow_marker);
    }
    
    visualization_msgs::MarkerArray marker_array_msg;
    marker_array_msg.markers = markers_to_publish;
    pub_velocity_markers_.publish(marker_array_msg);
}

void FilterReflectiveUavs::publishEstimates(const std::string& frame_id, const ros::Time& timestamp, std::vector<Track>& tracks) {
	// ros::Time time_now = ros::Time::now();
    mrs_msgs::PoseWithCovarianceArrayStamped msg_estimates;
    msg_estimates.header.frame_id = frame_id;
    msg_estimates.header.stamp    = timestamp;

	filter_reflective_uavs::PoseVelocityArray msg;
	msg.header.stamp = timestamp;
	msg.header.frame_id = frame_id;

    for (auto& track : tracks) {
		msg.ids.push_back(track.id);
		Eigen::Vector3d pos = track.x.segment<3>(0); // indices 0..2
    	Eigen::Vector3d vel = track.x.segment<3>(3); // indices 3..5
		
		geometry_msgs::Pose p;
		p.position.x = pos.x();
		p.position.y = pos.y();
		p.position.z = pos.z();
		p.orientation.w = 1.0;  // identity quaternion

		geometry_msgs::Vector3 v;
		v.x = vel.x();
		v.y = vel.y();
		v.z = vel.z();

		msg.poses.push_back(p);
		msg.velocities.push_back(v);

		mrs_msgs::PoseWithCovarianceIdentified msg_est;
		msg_est.id            = track.id;
		// KF state: [px, py, pz, vx, vy, vz, ax, ay, az]^T
		Eigen::Matrix3d pos_cov = track.P.block<3,3>(0,0);

		msg_est.pose = pose_to_msg(pos, vel);
		covariance_to_msg(pos_cov, msg_est.covariance);
		msg_estimates.poses.push_back(msg_est);
    }
	pub_pose_vel_array_.publish(msg);
    publisher_estimates_.publish(msg_estimates);
}

std::vector<std::pair<ros::Time, Eigen::Vector3d>> FilterReflectiveUavs::clusterToCentroids(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, ros::Time timestamp, const std::string& frame_id) {
	std::vector<std::pair<ros::Time, Eigen::Vector3d>> centroid_positions;
	// int cnt = 0;
	// for (auto&& pt : cloud->points) {
	// 	if (pt.intensity>=_min_intensity_) {
	// 		cnt += 1;
	// 	}
	// }
	// std::cout << "[CentroidsClustering]: cloud has this many points actually: " << cnt << std::endl;
	if (cloud->size() == 0) {
		ROS_WARN_THROTTLE(1.0, "[CentroidsClustering]: Input pointcloud is empty!");
		return centroid_positions;
	}

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_reflective = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
	pcl::ConditionAnd<pcl::PointXYZI>::Ptr reflectivity_cond(new pcl::ConditionAnd<pcl::PointXYZI>());
	std::string                   field_name = "intensity";

	reflectivity_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::Ptr(new pcl::FieldComparison<pcl::PointXYZI>(field_name, pcl::ComparisonOps::GT, _min_intensity_)));
	pcl::ConditionalRemoval<pcl::PointXYZI> reflectivity_filt;
	reflectivity_filt.setCondition(reflectivity_cond);
	reflectivity_filt.setKeepOrganized(false);
	reflectivity_filt.setInputCloud(cloud);
	reflectivity_filt.filter(*cloud_reflective);
	// ROS_INFO_THROTTLE(1.0, "[CentroidsClustering]: Reflective points in cloud found: %d, (min threshold: %f)", (int)cloud_reflective->size(), _min_intensity_);
	if (cloud_reflective->size() == 0) {
		ROS_WARN_THROTTLE(1.0, "[CentroidsClustering]: No reflective points in cloud (min threshold: %f)", _min_intensity_);
		return centroid_positions;
	}

	if (_use_voxel_grid_) {
		pcl::VoxelGrid<pcl::PointXYZI> vg;
		vg.setInputCloud(cloud);
		vg.setDownsampleAllData(false);
		vg.setLeafSize(_voxel_grid_size_x_, _voxel_grid_size_y_, _voxel_grid_size_z_);
		vg.filter(*cloud);
	}

	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree_reflective = boost::make_shared<pcl::search::KdTree<pcl::PointXYZI>>();
	tree_reflective->setInputCloud(cloud_reflective);

	const std::vector<pcl::PointIndices> cluster_indices = doEuclideanClustering(tree_reflective, cloud_reflective, _reflective_clustering_tolerance_, _reflective_clustering_min_points_, _reflective_clustering_max_points_);

	// ROS_INFO_THROTTLE(1.0, "[CentroidsClustering]: Found %d reflective clusters.", (int)cluster_indices.size());

	std::vector<pcl::PointXYZ> centroids_reflective;
	calculateCentroid2(cloud_reflective, cluster_indices, centroids_reflective);

	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_reflective_centroids = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	for (auto&& pt : centroids_reflective) {
		// cloud_reflective_centroids->push_back(pt);

		centroid_positions.emplace_back(timestamp, Eigen::Vector3d(pt.x, pt.y, pt.z));
	}
	// cloud_reflective_centroids->header.frame_id = frame_id;
	// cloud_reflective_centroids->header.stamp = timestamp.toNSec();
	// std::cout << "[CentroidsClustering]: Publishing number of centroids: " << cloud_reflective_centroids->size() << std::endl;

	// publisher_pointcloud_reflective_centroids_.publish(cloud_reflective_centroids);
	return centroid_positions;
}

void FilterReflectiveUavs::calculateCentroid2(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const std::vector<pcl::PointIndices>& cluster_indices, std::vector<pcl::PointXYZ>& result) {
  // go through all the clusters, calculate centroid for each one, push it to the vector
  for (auto&& cluster : cluster_indices) {
    pcl::CentroidPoint<pcl::PointXYZ> centroid;
    for (auto&& index : cluster.indices) {
      /* cloud_merged->push_back(cloud->at(index)); */
      pcl::PointXYZ pt;
      pt.x = cloud->at(index).x;
      pt.y = cloud->at(index).y;
      pt.z = cloud->at(index).z;
      centroid.add(pt);
    }
    pcl::PointXYZ center_pt;
    centroid.get(center_pt);
    result.push_back(center_pt);
  }
}

std::vector<pcl::PointIndices> FilterReflectiveUavs::doEuclideanClustering(const pcl::search::KdTree<pcl::PointXYZI>::Ptr tree_orig, const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                                                                float clustering_tolerance, int min_points, int max_points,
                                                                                const pcl::IndicesConstPtr indices_within_radius) {
  std::vector<pcl::PointIndices>         ret;
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

void FilterReflectiveUavs::filterOutUavs(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud, const std::string& frame_id, const ros::Time& timestamp, std::vector<Track>& tracks) {
	// std::shared_lock<std::shared_mutex> lock(uav_positions_mutex);

	std::vector<int> seed_indices;
	pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;

    std::cout << "Injecting this many points to pcl: " << tracks.size() << std::endl;
	for (const auto& track : tracks) {
		pcl::PointXYZI p;
		p.x = track.x[0]; 
		p.y = track.x[1];
		p.z = track.x[2];
		p.intensity = 255.0f; 

		int current_index = pcl_cloud->points.size();
		pcl_cloud->points.push_back(p);
		pcl_cloud->width = pcl_cloud->points.size();
		pcl_cloud->height = 1;
		seed_indices.push_back(current_index);
	}
	// lock.unlock();

	if (pcl_cloud->points.size() ==	 0) {
		sensor_msgs::PointCloud2 output_msg;
		pcl::toROSMsg(*pcl_cloud, output_msg);
		output_msg.header.frame_id = frame_id; 
		output_msg.header.stamp = timestamp; 
		pub_pointCloud_.publish(output_msg);		
		return;
	}
	kdtree.setInputCloud(pcl_cloud);

	std::vector<bool> is_uav_point(pcl_cloud->points.size(), false);
	std::vector<float> min_sq_dist_from_seed(pcl_cloud->points.size(), std::numeric_limits<float>::max());

	struct QueueElement {
		int idx;
		float sq_dist_from_seed;
	};

	float max_sq_distance_from_seed = _max_distance_from_seed_ * _max_distance_from_seed_;

	for (int idx_seed: seed_indices) {
		/* std::cout << "loopin through idx seeds" << std::endl; */
		std::queue<QueueElement> q;
		q.push({idx_seed, 0.0f});
		pcl::PointXYZI seed = pcl_cloud->points[idx_seed];
		is_uav_point[idx_seed] = true;

		while (!q.empty()) {
			/* std::cout << "Queue size is: " << q.size() << std::endl; */
			QueueElement current_element = q.front();
			q.pop();
			int current_idx = current_element.idx;
			std::vector<int> neighbors;
			std::vector<float> sqr_distances_to_neighbor; // sqr_distances from current_idx to neighbor
			kdtree.radiusSearch(pcl_cloud->points[current_idx], _search_radius_, neighbors, sqr_distances_to_neighbor);

			for (size_t i = 0; i < neighbors.size(); ++i) {	
				int neighbor_idx = neighbors[i];
				pcl::PointXYZI neighbor_point = pcl_cloud->points[neighbor_idx];
				float dx = neighbor_point.x - seed.x;
				float dy = neighbor_point.y - seed.y;
				float dz = neighbor_point.z - seed.z;
				float sq_dist_to_seed = dx*dx + dy*dy + dz*dz;
				
				if (sq_dist_to_seed > max_sq_distance_from_seed || is_uav_point[neighbor_idx]) {
          /* std::cout << "sq_dist_to_seed: " << sq_dist_to_seed << ", max_sq_distance_from_seed: " << max_sq_distance_from_seed << ", is_uav_point: " << is_uav_point[neighbor_idx] << std::endl; */
          continue;
				} else {	
          /* std::cout << "pushing to queue point" << std::endl; */
					is_uav_point[neighbor_idx] = true; 
					pcl::PointXYZ pt_xyz(pcl_cloud->points[neighbor_idx].x, pcl_cloud->points[neighbor_idx].y, pcl_cloud->points[neighbor_idx].z);
					// agent_pcl->push_back(pt_xyz);
					q.push({neighbor_idx, sq_dist_to_seed});
				}
			}
		}
	}

	/* std::cout << "seeds are done" << std::endl; */

	pcl::PointCloud<pcl::PointXYZI>::Ptr environment_cloud(new pcl::PointCloud<pcl::PointXYZI>);

	for (size_t i = 0; i < pcl_cloud->points.size(); ++i) {
		if (!is_uav_point[i]) {
			environment_cloud->points.push_back(pcl_cloud->points[i]);
		}
	}

	std::cout << "[Livox] Removed points: " << pcl_cloud->points.size() - environment_cloud->points.size() << std::endl;

	sensor_msgs::PointCloud2 output_msg;
	pcl::toROSMsg(*environment_cloud, output_msg);
	output_msg.header.frame_id = frame_id; 
	output_msg.header.stamp = timestamp; 
	pub_pointCloud_.publish(output_msg);

	pcl::PointCloud<pcl::PointXYZI>::Ptr uav_cloud(new pcl::PointCloud<pcl::PointXYZI>);

	for (size_t i = 0; i < pcl_cloud->points.size(); ++i) {
		if (is_uav_point[i]) {
			uav_cloud->points.push_back(pcl_cloud->points[i]);
		}
	}

	sensor_msgs::PointCloud2 output_msg_removed_points;
	pcl::toROSMsg(*uav_cloud, output_msg_removed_points);
	output_msg_removed_points.header.frame_id = frame_id; 
	output_msg_removed_points.header.stamp = timestamp; 
	pub_pointCloud_removed_.publish(output_msg_removed_points);


	pcl::PointCloud<pcl::PointXYZI>::Ptr seed_cloud(new pcl::PointCloud<pcl::PointXYZI>);

	for (const auto& uav_pos : uav_positions) {
		pcl::PointXYZI p;
		p.x = uav_pos.second.x(); 
		p.y = uav_pos.second.y();
		p.z = uav_pos.second.z();
		p.intensity = 255.0f; 

		seed_cloud->points.push_back(p);
	}

	sensor_msgs::PointCloud2 output_msg_seeds;
	pcl::toROSMsg(*seed_cloud, output_msg_seeds);
	output_msg_seeds.header.frame_id = frame_id; 
	output_msg_seeds.header.stamp = timestamp; 
	pub_seeds_.publish(output_msg_seeds);
}

geometry_msgs::Pose FilterReflectiveUavs::pose_to_msg(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity) {
	geometry_msgs::Pose msg;
	msg.position.x = position.x();
	msg.position.y = position.y();
	msg.position.z = position.z();
	Eigen::Vector3d v(1, 0, 0);
	auto            eigen_quat = Eigen::Quaterniond::FromTwoVectors(v, velocity);
	msg.orientation.x          = eigen_quat.x();
	msg.orientation.y          = eigen_quat.y();
	msg.orientation.z          = eigen_quat.z();
	msg.orientation.w          = eigen_quat.w();
	return msg;
}

void FilterReflectiveUavs::covariance_to_msg(const Eigen::Matrix3d& cov, boost::array<double, 36>& msg_cov_out) {
	msg_cov_out.assign(0.0); 	

	// copy the 3x3 block into the top-left corner of the 6x6
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			msg_cov_out[6 * i + j] = cov(i, j);	
		}
	}
}

void FilterReflectiveUavs::pointCloud2PosCallback(const sensor_msgs::PointCloud2& pcl_cloud2) {
	std::unique_lock<std::shared_mutex> lock(uav_positions_mutex);

	/* uav_positions.clear(); */
	ros::Time now = ros::Time::now();
	ros::Time pose_time = pcl_cloud2.header.stamp;
	pcl::PointCloud<pcl::PointXYZ> pcl_cloud;

    pcl::fromROSMsg(pcl_cloud2, pcl_cloud);

    for (const auto& point : pcl_cloud.points) {
        double x = point.x;
        double y = point.y;
        double z = point.z;
		uav_positions.emplace_back(pose_time, Eigen::Vector3d(x, y, z));
	}

  uav_positions.erase(std::remove_if(uav_positions.begin(), uav_positions.end(),[&](const auto& entry) {
          return (now - entry.first).toSec() > _time_keep_;}),uav_positions.end());
}

void FilterReflectiveUavs::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	_agent_pos_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
}

void FilterReflectiveUavs::callbackPoses(const mrs_msgs::PoseWithCovarianceArrayStamped::ConstPtr msg) {
	std::unique_lock<std::shared_mutex> lock(uav_positions_mutex);

	/* uav_positions.clear(); */
  ros::Time now = ros::Time::now();
  ros::Time pose_time = msg->header.stamp;
	for (const auto& pose : msg->poses) {
		    double x = pose.pose.position.x;
		    double y = pose.pose.position.y;
		    double z = pose.pose.position.z;

        uav_positions.emplace_back(pose_time ,Eigen::Vector3d(x, y, z));
    }
  uav_positions.erase(std::remove_if(uav_positions.begin(), uav_positions.end(),[&](const auto& entry) {
          return (now - entry.first).toSec() > _time_keep_;}),uav_positions.end());
}

void FilterReflectiveUavs::timeoutGeneric(const std::string& topic, const ros::Time& last_msg) {
  ROS_WARN_THROTTLE(1.0, "[FilterReflectiveUavs]: not receiving '%s' for %.3f s", topic.c_str(), (ros::Time::now() - last_msg).toSec());
}

}  // namespace filter_reflective_uavs

PLUGINLIB_EXPORT_CLASS(filter_reflective_uavs::FilterReflectiveUavs, nodelet::Nodelet);
