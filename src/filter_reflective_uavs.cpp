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
	param_loader.loadParam("min_intensity", _min_intensity_);
	param_loader.loadParam("max_intensity", _max_intensity_);
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
		sh_pointcloud_ = mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2>(shopts, "lidar3d_in", ros::Duration(1.0), &FilterReflectiveUavs::timeoutGeneric, this, &FilterReflectiveUavs::callbackPointCloudOuster, this);
	} else { //for livox and for real world experiments - due to different data storage
		sh_pointcloud_ = mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2>(shopts, "lidar3d_in", ros::Duration(1.0), &FilterReflectiveUavs::timeoutGeneric, this, &FilterReflectiveUavs::callbackPointCloud, this);
	}

	sh_uav_position_estimation_ = mrs_lib::SubscribeHandler<mrs_msgs::PoseWithCovarianceArrayStamped>(shopts, "estimated_pos", ros::Duration(1.0), &FilterReflectiveUavs::timeoutGeneric, this, &FilterReflectiveUavs::callbackPoses, this);

	pub_pointCloud_ = nh.advertise<sensor_msgs::PointCloud2>("filtered_pcl", 10, true);
	pub_agent_pcl_  = nh.advertise<sensor_msgs::PointCloud2>("agents_pcl", 10, true);

	transformer_ = mrs_lib::Transformer("FilterReflectiveUavs");
  	transformer_.setLookupTimeout(ros::Duration(0.1));

	is_initialized_ = true;
	ROS_INFO("[%s]: Initialization completed.", node_name.c_str());
};

void FilterReflectiveUavs::callbackPointCloudOuster(const sensor_msgs::PointCloud2::ConstPtr msg) {
	//todo also publish maybe centroid of removed points for the rbl agent feed

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

	std::string frame_id = msg->header.frame_id;
    ros::Time timestamp = msg->header.stamp;

	filterOutUavs(xyzi_cloud, frame_id, timestamp);
}


void FilterReflectiveUavs::filterOutUavs(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud, const std::string& frame_id, const ros::Time& timestamp) {
	std::shared_lock<std::shared_mutex> lock(uav_positions_mutex);

	std::vector<int> seed_indices;
	pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;

	if (_load_gt_uav_positions_) {
		std::vector<std::string> uav_list_ = {"uav1", "uav2", "uav3", "uav4"};
		for (size_t i = 0; i < uav_list_.size(); ++i) {
			const std::string& uav_name = uav_list_[i];
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

			int current_index = pcl_cloud->points.size();
			pcl_cloud->points.push_back(p);
			pcl_cloud->width = pcl_cloud->points.size();
			pcl_cloud->height = 1;
			seed_indices.push_back(current_index);
		}
	} else {
    /* std::cout << "Saved this many uav positions: " << uav_positions.size() << std::endl; */
		for (const auto& uav_pos : uav_positions) {
			pcl::PointXYZI p;
      p.x = uav_pos.second.x(); 
			p.y = uav_pos.second.y();
			p.z = uav_pos.second.z();
			p.intensity = 255.0f; 

			int current_index = pcl_cloud->points.size();
			pcl_cloud->points.push_back(p);
			pcl_cloud->width = pcl_cloud->points.size();
			pcl_cloud->height = 1;
			seed_indices.push_back(current_index);
		}
	}
	lock.unlock();

	/* std::cout << "added points to the pcl cloud" << std::endl; */

	if (pcl_cloud->points.size() ==	 0) {
		/* std::cout << "pcl cloud is 0" << std::endl; */

		sensor_msgs::PointCloud2 output_msg;
		pcl::toROSMsg(*pcl_cloud, output_msg);
		output_msg.header.frame_id = frame_id; 
		output_msg.header.stamp = timestamp; 
		pub_pointCloud_.publish(output_msg);		
		// pcl::PointCloud<pcl::PointXYZ>::Ptr agent_centroids;
		// sensor_msgs::PointCloud2 output_centroid_msg;
		// pcl::toROSMsg(*agent_centroids, output_centroid_msg);
		// output_centroid_msg.header = msg->header;
		// pub_agent_pcl_.publish(output_centroid_msg);

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

	/* std::cout << "[Livox] Removed points: " << pcl_cloud->points.size() - environment_cloud->points.size() << std::endl; */

	sensor_msgs::PointCloud2 output_msg;
	pcl::toROSMsg(*environment_cloud, output_msg);
	output_msg.header.frame_id = frame_id; 
	output_msg.header.stamp = timestamp; 
	pub_pointCloud_.publish(output_msg);
}

void FilterReflectiveUavs::callbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr msg) { //this one is for livox
	if (!is_initialized_) {
    	return;
  	}

	pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::fromROSMsg(*msg, *pcl_cloud);

	std::string frame_id = msg->header.frame_id;
    ros::Time timestamp = msg->header.stamp;

	filterOutUavs(pcl_cloud, frame_id, timestamp);

}

pcl::PointCloud<pcl::PointXYZ>::Ptr FilterReflectiveUavs::cluster_agent_pcl_to_centroids(pcl::PointCloud<pcl::PointXYZ>::Ptr agent_pcl) {
	// TODO consider moving the centroid a little bit further from the current sensing UAV - Will see mainly the closer boundary points
	pcl::PointCloud<pcl::PointXYZ>::Ptr agent_centroids(new pcl::PointCloud<pcl::PointXYZ>);

    if (agent_pcl->empty()) {
        ROS_WARN_STREAM("Do not see any agents. Input agent_pcl is empty, no centroids to compute.");
        return agent_centroids;
    }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_for_clustering(new pcl::search::KdTree<pcl::PointXYZ>());
    kdtree_for_clustering->setInputCloud(agent_pcl);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

    ec.setClusterTolerance(_search_radius_);
    ec.setMinClusterSize(1);
    ec.setMaxClusterSize(50);

    ec.setSearchMethod(kdtree_for_clustering);
    ec.setInputCloud(agent_pcl);

    std::vector<pcl::PointIndices> cluster_indices;

    ec.extract(cluster_indices);

    ROS_INFO_STREAM("Found " << cluster_indices.size() << " UAV clusters.");

    for (const auto& cluster : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        for (int index : cluster.indices) {
            current_cluster_cloud->points.push_back(agent_pcl->points[index]);
        }

        Eigen::Vector4f cluster_centroid_eigen; 
        pcl::compute3DCentroid(*current_cluster_cloud, cluster_centroid_eigen);

        pcl::PointXYZ centroid_point;
        centroid_point.x = cluster_centroid_eigen[0];
        centroid_point.y = cluster_centroid_eigen[1];
        centroid_point.z = cluster_centroid_eigen[2];

        agent_centroids->points.push_back(centroid_point);

        ROS_INFO_STREAM("Cluster centroid: (" << centroid_point.x << ", " << centroid_point.y << ", " << centroid_point.z << ")");
    }

    agent_centroids->width = agent_centroids->points.size();
    agent_centroids->height = 1;
    agent_centroids->is_dense = true;

    return agent_centroids;
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
