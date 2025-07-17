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
	param_loader.loadParam("ouster", _ouster_);
	
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

	if (_ouster_) {
		sh_pointcloud_ = mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2>(shopts, "lidar3d_in", ros::Duration(1.0), &FilterReflectiveUavs::timeoutGeneric, this, &FilterReflectiveUavs::callbackPointCloudOuster, this);
	} else {
		sh_pointcloud_ = mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2>(shopts, "lidar3d_in", ros::Duration(1.0), &FilterReflectiveUavs::timeoutGeneric, this, &FilterReflectiveUavs::callbackPointCloud, this);
	}

	pub_pointCloud_ = nh.advertise<sensor_msgs::PointCloud2>("filtered_pcl", 10, true);

	is_initialized_ = true;
	ROS_INFO("[%s]: Initialization completed.", node_name.c_str());
};

void FilterReflectiveUavs::callbackPointCloudOuster(const sensor_msgs::PointCloud2::ConstPtr msg) {
	//todo max diameter fo the uav - so it doesnt search further - also load, also publish maybe centroid of removed points for the rbl agent feed

	if (!is_initialized_) {
    	return;
  	}
	// pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>());
	// pcl::fromROSMsg(*msg, *pcl_cloud);


	pcl::PointCloud<ouster_ros::Point>::Ptr pcl_cloud = boost::make_shared<pcl::PointCloud<ouster_ros::Point>>();
  	pcl::fromROSMsg(*msg, *pcl_cloud);

	// print all different intensities
	// std::vector<bool> intensities(256, false);

	// for (auto&& pt : *pcl_cloud) {
	// 	if (intensities[pt.reflectivity] == false) {
	// 	intensities[pt.reflectivity] = true;
	// 	}
	// }

	// for (size_t j = 0; j < intensities.size(); ++j){
	// 	if (intensities[j] == true) {
	// 	std::cout << j << ", ";
	// 	}
	// }
	// std::cout << std::endl;

	// pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
	pcl::search::KdTree<ouster_ros::Point>::Ptr kdtree = boost::make_shared<pcl::search::KdTree<ouster_ros::Point>>();
	kdtree->setInputCloud(pcl_cloud);

	std::vector<int> seed_indices;
	for (int i = 0; i < pcl_cloud->points.size(); ++i) {
		if (pcl_cloud->points[i].reflectivity >= _min_intensity_ && pcl_cloud->points[i].reflectivity <= _max_intensity_) {
			seed_indices.push_back(i);
		}
	}

	std::vector<bool> is_uav_point(pcl_cloud->points.size(), false);
	for (int idx : seed_indices) {
		is_uav_point[idx] = true;
	}

	for (int idx : seed_indices) {
		std::queue<int> q;
		q.push(idx);

		while (!q.empty()) {
			int current_idx = q.front();
			q.pop();

			std::vector<int> neighbors;
			std::vector<float> sqr_distances;
			kdtree->radiusSearch(pcl_cloud->points[current_idx], _search_radius_, neighbors, sqr_distances);

			for (int n_idx : neighbors) {
				if (!is_uav_point[n_idx]) {
				is_uav_point[n_idx] = true;
				q.push(n_idx);
				}
			}
		}
	}

	pcl::PointCloud<ouster_ros::Point>::Ptr environment_cloud = boost::make_shared<pcl::PointCloud<ouster_ros::Point>>();
	for (size_t i = 0; i < pcl_cloud->points.size(); ++i) {
		if (!is_uav_point[i]) {
			environment_cloud->points.push_back(pcl_cloud->points[i]);
		}
	}

	std::cout << "Removed points: " << pcl_cloud->points.size() - environment_cloud->points.size() << std::endl;

	sensor_msgs::PointCloud2 output_msg;
	pcl::toROSMsg(*environment_cloud, output_msg);
	output_msg.header = msg->header; // timestamp,frame_id
	pub_pointCloud_.publish(output_msg);
}


void FilterReflectiveUavs::callbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr msg) {
	//todo interval, search radius load, max diameter fo the uav - so it doesnt search further - also load, also publish maybe centroid of removed points for the rbl agent feed

	if (!is_initialized_) {
    	return;
  	}
	pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::fromROSMsg(*msg, *pcl_cloud);

	// // print all different intensities
	// std::vector<bool> intensities(256, false);

	// for (auto&& pt : *pcl_cloud) {
	// 	if (intensities[pt.intensity] == false) {
	// 	intensities[pt.intensity] = true;
	// 	}
	// }

	// for (size_t j = 0; j < intensities.size(); ++j){
	// 	if (intensities[j] == true) {
	// 	std::cout << j << ", ";
	// 	}
	// }
	// std::cout << std::endl;

	pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
	kdtree.setInputCloud(pcl_cloud);

	std::vector<int> seed_indices;
	for (int i = 0; i < pcl_cloud->points.size(); ++i) {
		if (pcl_cloud->points[i].intensity == 255) {
			seed_indices.push_back(i);
		}
	}

	std::vector<bool> is_uav_point(pcl_cloud->points.size(), false);
	for (int idx : seed_indices) {
		is_uav_point[idx] = true;
	}

	double radius = 0.2; //TODO tune, load

	for (int idx : seed_indices) {
		std::queue<int> q;
		q.push(idx);

		while (!q.empty()) {
			int current_idx = q.front();
			q.pop();

			std::vector<int> neighbors;
			std::vector<float> sqr_distances;
			kdtree.radiusSearch(pcl_cloud->points[current_idx], radius, neighbors, sqr_distances);

			for (int n_idx : neighbors) {
				if (!is_uav_point[n_idx]) {
				is_uav_point[n_idx] = true;
				q.push(n_idx);
				}
			}
		}
	}

	pcl::PointCloud<pcl::PointXYZI>::Ptr environment_cloud(new pcl::PointCloud<pcl::PointXYZI>);

	for (size_t i = 0; i < pcl_cloud->points.size(); ++i) {
		if (!is_uav_point[i]) {
			environment_cloud->points.push_back(pcl_cloud->points[i]);
		}
	}

	std::cout << "Removed points: " << pcl_cloud->points.size() - environment_cloud->points.size() << std::endl;

	sensor_msgs::PointCloud2 output_msg;
	pcl::toROSMsg(*environment_cloud, output_msg);
	output_msg.header = msg->header; // timestamp,frame_id
	pub_pointCloud_.publish(output_msg);
}

void FilterReflectiveUavs::timeoutGeneric(const std::string& topic, const ros::Time& last_msg) {
  ROS_WARN_THROTTLE(1.0, "[FilterReflectiveUavs]: not receiving '%s' for %.3f s", topic.c_str(), (ros::Time::now() - last_msg).toSec());
}

}  // namespace filter_reflective_uavs

PLUGINLIB_EXPORT_CLASS(filter_reflective_uavs::FilterReflectiveUavs, nodelet::Nodelet);