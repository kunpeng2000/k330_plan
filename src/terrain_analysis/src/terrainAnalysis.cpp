/**
  ******************************************************************************
  * @file    main.cc
  * @author  Alex Liu 
  * @version V1.0.0
  * @date    2022/01/11
  * @brief   terrain analysis module for MobiRo @ tib_k331
  ******************************************************************************
  * @attention modified from FALCO code by Zhang ji 
  *
  ******************************************************************************
  */

#include "terrainAnalysis.h"

namespace tib_k331_perception {

TerrainAnalysis::TerrainAnalysis(const ros::NodeHandle& nh): 
	system_inited_(false), new_laser_cloud_(false), 
	no_data_inited_(0), nh_(nh) {

	ParamsHandler(nh_);

	odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(sub_odom_topic_, 5,
		&TerrainAnalysis::OdometryCallback, this);
	laser_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(sub_scan_topic_, 5,
		&TerrainAnalysis::ScanCallback, this);
	terrain_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/terrain/terrain_map", 2);
	local_terrain_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/terrain/local_map", 2);
	removed_height_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/terrain/removed_height_map", 2);
	removed_dyn_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/terrain/removed_dyn_map", 2);
	colored_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/terrain/colored_terrain_map", 2);

	laser_cloud_current_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	laser_cloud_current_crop_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
	cloud_down_sampled_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
	local_terrain_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
	dyn_candidate_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
	terrain_cloud_elev_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
	removed_height_num_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
	removed_dyn_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
	colored_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
	terrain_voxel_cloud_ = new pcl::PointCloud<pcl::PointXYZI>::Ptr[terrain_voxel_num_];
	for (int i = 0; i < terrain_voxel_num_; i++) {
		terrain_voxel_cloud_[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
	}

	down_size_filter_.setLeafSize(downsample_voxel_size_, downsample_voxel_size_, downsample_voxel_size_);
	terrain_voxel_update_num_ = new int[terrain_voxel_num_]();
	terrain_voxel_update_time_ = new float[terrain_voxel_num_]();
	planar_voxel_elev_ = new float[planar_voxel_num_]();
	planar_voxel_edge_ = new int[planar_voxel_num_]();
	planar_voxel_dyn_obs_ = new int[planar_voxel_num_]();
	planar_point_elev_ = new std::vector<float>[planar_voxel_num_]();

	sys_init_time_ = 0; 
	scan_time_stamp_ = 0;
	robot_state_ = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};  // x y z r p y
	terrain_voxel_shift_ = {0, 0};
}

TerrainAnalysis::~TerrainAnalysis() {
	delete [] terrain_voxel_update_num_;
	delete [] terrain_voxel_update_time_;
	delete [] planar_voxel_elev_;
	delete [] planar_voxel_edge_;
	delete [] planar_voxel_dyn_obs_;
	delete [] planar_point_elev_;
}

void TerrainAnalysis::ParamsHandler(const ros::NodeHandle &nh) {
	nh.param("voxel_point_update_thre", terrrain_voxel_num_update_thre_, 100);
	nh.param("terrain_voxel_time_update_thre", terrain_voxel_time_update_thre_, 2.0);
	nh.param("downsample_voxel_size", downsample_voxel_size_, 0.05);
	nh.param("min_rel_z", min_rel_z_, -0.8); //
	nh.param("max_rel_z", max_rel_z_, 0.8); //
	nh.param("dis_ratio_z_", dis_ratio_z_, 0.2); //
	nh.param("decay_time", decay_time_, 5.0);
	nh.param("no_decay_dis", no_decay_dis_, 4.0);

	nh.param("robot_height", robot_height_, 0.6);
	nh.param("remove_dyn_obs_flag", remove_dyn_obs_flag_, true);
	nh.param("min_dyn_obs_dis", min_dyn_obs_dis_, 0.3); //
	nh.param("min_dyn_obs_rel_z", min_dyn_obs_rel_z_, 0.0);
	nh.param("min_dyn_obs_angle", min_dyn_obs_angle_, 0.0); //
	nh.param("min_dyn_obs_fov", min_dyn_obs_fov_, -16.0); //
	nh.param("max_dyn_obs_fov", max_dyn_obs_fov_, 16.0); //
	nh.param("min_dyn_obs_pt_num", min_dyn_obs_pt_num_, 0);

	nh.param("use_sorting", use_sorting_flag_, true);
	nh.param("quantile_z", quantile_z_, 0.25);	
	nh.param("max_ground_lift", max_ground_lift_, 0.2);
	nh.param("limit_ground_lift", limit_ground_lift_flag_, false); //

	nh.param("min_dyn_obs_pt_thres", min_dyn_obs_pt_thres_, 10);
	nh.param("consider_drop", consider_drop_flag_, false);
	nh.param("min_block_pt_num", min_block_pt_num_, 0); //

	nh.param("no_data_obstacle", no_data_obstacle_, false);
	nh.param("no_data_block_skip_num", no_data_block_skip_num_, 0);

	nh.param<std::string>("sub_odom_topic", sub_odom_topic_, "/Odometry");
	nh.param<std::string>("sub_scan_topic", sub_scan_topic_, "/registered_scan");
	nh.param<std::string>("frame_id", frame_id_, "map");
}

void TerrainAnalysis::AnalysisInstance() {
	ros::Rate loop_rate(100);
	while(ros::ok()) {
		ros::spinOnce();
		if (new_laser_cloud_) {
			auto t_start = std::chrono::high_resolution_clock::now();

			new_laser_cloud_ = false;

			// terrain voxel roll over till map center and robot in the same voxel
			this->TerrainCenterUpdate();

			// stack registered laser scans in belonging voxel
			pcl::PointXYZI laser_point_current;
			int crop_cloud_size = laser_cloud_current_crop_->points.size();
			for (int i = 0; i < crop_cloud_size; i++) {
				laser_point_current = laser_cloud_current_crop_->points[i];
				VoxelIndex idx = LidarPt2TerrainCoord(laser_point_current);
				if (idx.x >= 0 && idx.x < terrain_voxel_width_ && idx.y >= 0 && idx.y < terrain_voxel_width_) {
					int cloud_idx = terrain_voxel_width_ * idx.x + idx.y;
					terrain_voxel_cloud_[cloud_idx]->push_back(laser_point_current);
					terrain_voxel_update_num_[cloud_idx]++;
				}
			}

			// downsample, select points based on z and time
			this->TerrainCloudDecayHandler();

			// crop stacked cloud with size of 10 x 10 voxel, buffer in local_terrain_cloud_
			local_terrain_cloud_->clear();
			for (int idx_x = terrain_width_half_ - 5; idx_x <= terrain_width_half_ + 5; idx_x++) {
				for (int idx_y = terrain_width_half_ - 5; idx_y <= terrain_width_half_ + 5; idx_y++) {
					int cloud_idx = terrain_voxel_width_ * idx_x + idx_y;
					*local_terrain_cloud_ += *terrain_voxel_cloud_[cloud_idx];
				}    
			}
			pcl::toROSMsg(*local_terrain_cloud_, local_terrain_msg_);
			local_terrain_msg_.header.stamp = ros::Time().fromSec(scan_time_stamp_);
			local_terrain_msg_.header.frame_id = frame_id_;
			local_terrain_pub_.publish(local_terrain_msg_);

			for (int i = 0; i < planar_voxel_num_; i++) {  // planar params init
				planar_voxel_elev_[i] = -robot_height_; // floor z
				planar_voxel_edge_[i] = 0;
				planar_voxel_dyn_obs_[i] = 0; // dyn obs num in each voxel
				planar_point_elev_[i].clear(); // points' abs z in each voxel
			}

			// cal floor and dynamic
			// stack laser point height in belonging voxel, one point belong to 9 neighbor planar voxel
			int terrain_cloud_size = local_terrain_cloud_->points.size();
			for (int i = 0; i < terrain_cloud_size; i++) {
				pcl::PointXYZI laser_point = local_terrain_cloud_->points[i];
				VoxelIndex idx = LidarPt2PlanarCoord(laser_point);
				if (laser_point.z - robot_state_.z > min_rel_z_ &&
					laser_point.z - robot_state_.z < max_rel_z_) { 
					for (int dx = -1; dx <= 1; dx++) {
						for (int dy = -1; dy <= 1; dy++) {
							if (idx.x + dx >= 0 && idx.x + dx < planar_voxel_width_ &&
								idx.y + dy >= 0 && idx.y + dy < planar_voxel_width_) {
								int pt_idx = planar_voxel_width_ * (idx.x + dx) + idx.y + dy;
								planar_point_elev_[pt_idx].push_back(laser_point.z);
							}
						}
					}  
				}
				
				if (remove_dyn_obs_flag_) {  // avoid dyn obs lasting
					dyn_candidate_cloud_ -> clear();
					// judge whether the points belong to dynamic points based on distance, angle, fov
					if (idx.x >= 0 && idx.x < planar_voxel_width_ && 
						idx.y >= 0 && idx.y < planar_voxel_width_) {
						Eigen::Vector3f relative_pos(laser_point.x - robot_state_.x,
													laser_point.y - robot_state_.y,
													laser_point.z - robot_state_.z);
						float map_horizon_dis = relative_pos.head(2).lpNorm<2>();

						if (map_horizon_dis > min_dyn_obs_dis_) {
							float relative_angle = std::atan2(relative_pos(2) - \      
								min_dyn_obs_rel_z_, map_horizon_dis) * 180.0 / M_PI;

							if (relative_angle > min_dyn_obs_angle_) {
								Eigen::AngleAxisf rot_yaw(-robot_state_.yaw,
														Eigen::Vector3f::UnitZ());
								
								Eigen::AngleAxisf rot_pitch(-robot_state_.pitch,
															Eigen::Vector3f::UnitY());
								
								Eigen::AngleAxisf rot_roll(-robot_state_.roll,
														Eigen::Vector3f::UnitX());
								Eigen::Quaternionf rot_q = rot_roll * rot_pitch * rot_yaw;

								Eigen::Vector3f robot_coord_pos = rot_q * relative_pos; // obs pos in robot's coor
								float robot_coord_dis = robot_coord_pos.head(2).lpNorm<2>();   
								float robot_coord_angle = std::atan2(robot_coord_pos(2), \
									robot_coord_dis) * 180.0 / M_PI;       
								
								if (robot_coord_angle > min_dyn_obs_fov_ &&
									robot_coord_angle < max_dyn_obs_fov_) {
									int voxel_idx = planar_voxel_width_ * idx.x + idx.y;
									planar_voxel_dyn_obs_[voxel_idx]++; 
									dyn_candidate_cloud_ -> push_back(laser_point);
								}
							}
						}
						else {
							int voxel_idx = planar_voxel_width_ * idx.x + idx.y;    
							planar_voxel_dyn_obs_[voxel_idx] += min_dyn_obs_pt_num_;
						}
					}
				}
			}

			// keep the current obs
			if (remove_dyn_obs_flag_) {
				for (int i = 0; i < crop_cloud_size; i++) {
					pcl::PointXYZI laser_point = laser_cloud_current_crop_->points[i];
					VoxelIndex idx = LidarPt2PlanarCoord(laser_point);

					if (idx.x >= 0 && idx.x < planar_voxel_width_ && idx.y >= 0 &&
						idx.y < planar_voxel_width_) {
						Eigen::Vector3f relative_pos(laser_point.x - robot_state_.x,
													laser_point.y - robot_state_.y,
													laser_point.z - robot_state_.z);
						float dis = relative_pos.head(2).lpNorm<2>();
						float angle = std::atan2(relative_pos(2) - \
							min_dyn_obs_rel_z_, dis) * 180.0 / M_PI;
						if (angle > min_dyn_obs_angle_) {
							int voxel_idx = planar_voxel_width_ * idx.x + idx.y;
							planar_voxel_dyn_obs_[voxel_idx] = 0; 
						} 
					}
				}
			}

			if (use_sorting_flag_) {
				for (int i = 0; i < planar_voxel_num_; i++) {
					int planar_pt_elev_size = planar_point_elev_[i].size();
					if (planar_pt_elev_size > 0) {
						std::sort(planar_point_elev_[i].begin(),
								planar_point_elev_[i].end());
						
						int quantile_id = int(quantile_z_ * planar_pt_elev_size);
						LowerBound(quantile_id, 0);
						UpperBound(quantile_id, planar_pt_elev_size - 1);

						double lift_thresh = planar_point_elev_[i][0] + max_ground_lift_;

						if (limit_ground_lift_flag_ && 
							planar_point_elev_[i][quantile_id] > lift_thresh) { 
							// fake obstacle
							planar_voxel_elev_[i] = lift_thresh;
						} 
						else {  
							// true obstacle 
							planar_voxel_elev_[i] = planar_point_elev_[i][quantile_id];  
						}
					}
				}
			} 
			else {  // manually set all voxel elev as the lowest pt height  直接找最低点
				for (int i = 0; i < planar_voxel_num_; i++) {
					int planar_pt_elev_size = planar_point_elev_[i].size();
					if (planar_pt_elev_size > 0) {
						float min_z = 1000.0;
						int min_id = -1;
						for (int j = 0; j < planar_pt_elev_size; j++) {
							if (planar_point_elev_[i][j] < min_z) {
								min_z = planar_point_elev_[i][j];
								min_id = j;
							}
						}
						if (min_id != -1) {
							planar_voxel_elev_[i] = planar_point_elev_[i][min_id];
						}
					}
				}
			}

			// final terrain cloud: rel_z_, min_dyn_obs_pt_thres_, robot_height_, min_block_pt_num_
			terrain_cloud_elev_->clear(); 
			removed_height_num_cloud_->clear();
			removed_dyn_cloud_->clear();
			int terrain_cloud_elev_size = 0;
			for (int i = 0; i < terrain_cloud_size; i++) {  
				pcl::PointXYZI laser_point = local_terrain_cloud_->points[i];  // select a laser point
				if (laser_point.z - robot_state_.z > min_rel_z_ &&  // again hard crop
					laser_point.z - robot_state_.z < max_rel_z_) {
					VoxelIndex idx = LidarPt2PlanarCoord(laser_point);

					if (idx.x >= 0 && idx.x < planar_voxel_width_ && idx.y >= 0 && idx.y < planar_voxel_width_) {
						int voxel_idx = planar_voxel_width_ * idx.x + idx.y;

						if (planar_voxel_dyn_obs_[voxel_idx] < min_dyn_obs_pt_thres_ || !remove_dyn_obs_flag_) {
							float dis_z = laser_point.z - planar_voxel_elev_[voxel_idx];
							// dis_z = (dis_z > 0.1) ? dis_z : 0;
							if (consider_drop_flag_) dis_z = std::fabs(dis_z);
							else LowerBound(dis_z, 0.0f);
							int planar_pt_elev_size = planar_point_elev_[voxel_idx].size();

							if (dis_z >= 0 && dis_z < robot_height_ * 1.2 && // coef to reserve more points
								planar_pt_elev_size >= min_block_pt_num_) {
								terrain_cloud_elev_->push_back(laser_point);
								auto pts_ptr = &(terrain_cloud_elev_->points);
								(*pts_ptr)[terrain_cloud_elev_size].intensity = dis_z;              
								terrain_cloud_elev_size ++;
							}
							else{
								removed_height_num_cloud_->push_back(laser_point);
							}
						} 
						else {
							removed_dyn_cloud_->push_back(laser_point);
						}
					}
				}
			}

			if (no_data_obstacle_ && no_data_inited_ == 2) {
				this->NoDataObsHandler();
			}

			// publish points with elevation
			pcl::toROSMsg(*terrain_cloud_elev_, terrain_cloud_msg_);
			terrain_cloud_msg_.header.stamp = ros::Time().fromSec(scan_time_stamp_);
			terrain_cloud_msg_.header.frame_id = frame_id_;
			terrain_pub_.publish(terrain_cloud_msg_);

			pcl::toROSMsg(*removed_height_num_cloud_, removed_height_cloud_msg_);
			removed_height_cloud_msg_.header.stamp = ros::Time().fromSec(scan_time_stamp_);
			removed_height_cloud_msg_.header.frame_id = frame_id_;
			removed_height_cloud_pub_.publish(removed_height_cloud_msg_);

			pcl::toROSMsg(*removed_dyn_cloud_, removed_dyn_cloud_msg_);
			removed_dyn_cloud_msg_.header.stamp = ros::Time().fromSec(scan_time_stamp_);
			removed_dyn_cloud_msg_.header.frame_id = frame_id_;
			removed_dyn_cloud_pub_.publish(removed_dyn_cloud_msg_);

			colored_cloud_->clear();
			colored_cloud_->resize(terrain_cloud_elev_->points.size());
			for (int i = 0; i < terrain_cloud_elev_->points.size(); i++){
				colored_cloud_->points[i].x = terrain_cloud_elev_->points[i].x;
				colored_cloud_->points[i].y = terrain_cloud_elev_->points[i].y;
				colored_cloud_->points[i].z = terrain_cloud_elev_->points[i].z;

				pcl::RGB color_tmp = heightToColor(terrain_cloud_elev_->points[i].intensity, 0.0);
				colored_cloud_->points[i].r = color_tmp.r;
				colored_cloud_->points[i].g = color_tmp.g;
				colored_cloud_->points[i].b = color_tmp.b;
			}
			
			pcl::toROSMsg(*colored_cloud_, colored_terrain_cloud_msg_);
			colored_terrain_cloud_msg_.header.stamp = ros::Time().fromSec(scan_time_stamp_);
			colored_terrain_cloud_msg_.header.frame_id = frame_id_;
			colored_pub_.publish(colored_terrain_cloud_msg_);

			auto t_end = std::chrono::high_resolution_clock::now();
			auto ms = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count();
			// std::cout << "analysisInstance: " << ms << " microseconds" << std::endl;
		}

		loop_rate.sleep();
	}	
}

void TerrainAnalysis::OdometryCallback(const nav_msgs::Odometry::ConstPtr &odom) {
	static float robot_x_rec, robot_y_rec;
	double roll, pitch, yaw;
	geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
	tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w))
		.getRPY(roll, pitch, yaw);
	// update robot state
	robot_state_.roll = roll;
	robot_state_.pitch = pitch;
	robot_state_.yaw = yaw;

	robot_state_.x = odom->pose.pose.position.x;
	robot_state_.y = odom->pose.pose.position.y;
	robot_state_.z = odom->pose.pose.position.z;

	if (no_data_inited_ == 0) {
		robot_x_rec = robot_state_.x;
		robot_y_rec = robot_state_.y;
		no_data_inited_ = 1;
	}

	if (no_data_inited_ == 1) {
		float dis = EuclideanNorm<float>(robot_state_.x - robot_x_rec,
										robot_state_.y - robot_y_rec);
		if (dis >= no_decay_dis_) no_data_inited_ = 2;
	}
}

void TerrainAnalysis::ScanCallback(const sensor_msgs::PointCloud2ConstPtr &laser_in) {
	scan_time_stamp_ = laser_in->header.stamp.toSec();  // update scan time stamp

	if (!system_inited_) {
		sys_init_time_ = scan_time_stamp_;
		system_inited_ = true;
	}

	laser_cloud_current_->clear();  // clear lidar cloud recv buffer
	pcl::fromROSMsg(*laser_in, *laser_cloud_current_);  // unpack point cloud msg
	pcl::PointXYZI laser_point;  // intermediate laser point var
	const int cloud_size = laser_cloud_current_->points.size();  // calc pc size
	
	laser_cloud_current_crop_->clear();  // clear intermediate pc buffer across thread
	for (int i = 0; i < cloud_size; i++) {
		laser_point.x = laser_cloud_current_->points[i].x;
		laser_point.y = laser_cloud_current_->points[i].y;
		laser_point.z = laser_cloud_current_->points[i].z;
		float dis = EuclideanNorm<float>(laser_point.x - robot_state_.x, 
										laser_point.y - robot_state_.y);
		
		if (laser_point.z - robot_state_.z > min_rel_z_ - dis_ratio_z_ * dis &&
			laser_point.z - robot_state_.z < max_rel_z_ + dis_ratio_z_ * dis &&
			dis < terrain_voxel_size_ * (terrain_width_half_ + 1)) {
			laser_point.intensity = scan_time_stamp_ - sys_init_time_;    //选择一定半径和高度范围内的点云，intensity只用于储存时间数据
			laser_cloud_current_crop_->push_back(laser_point);
		}
	}
	new_laser_cloud_ = true;
}

void TerrainAnalysis::TerrainCenterUpdate() {
	float terrain_voxel_cen_x = terrain_voxel_size_ * terrain_voxel_shift_.x;
	float terrain_voxel_cen_y = terrain_voxel_size_ * terrain_voxel_shift_.y;

	while (robot_state_.x - terrain_voxel_cen_x < -terrain_voxel_size_) {
		for (int idx_y = 0; idx_y < terrain_voxel_width_; idx_y++) {
			int cloud_idx = terrain_voxel_width_ * (terrain_voxel_width_ - 1) + idx_y;               
			pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr = terrain_voxel_cloud_[cloud_idx];
			for (int idx_x = terrain_voxel_width_ - 1; idx_x >= 1; idx_x--) {
				terrain_voxel_cloud_[cloud_idx] =
					terrain_voxel_cloud_[cloud_idx - terrain_voxel_width_];
				cloud_idx -= terrain_voxel_width_;
			}
			terrain_voxel_cloud_[cloud_idx] = cloud_ptr;
			terrain_voxel_cloud_[cloud_idx]->clear();
		}
		terrain_voxel_shift_.x--; 
		terrain_voxel_cen_x = terrain_voxel_size_ * terrain_voxel_shift_.x;
	}

	while (robot_state_.x - terrain_voxel_cen_x > terrain_voxel_size_) {
		for (int idx_y = 0; idx_y < terrain_voxel_width_; idx_y++) {
			int cloud_idx = idx_y;
			pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr = terrain_voxel_cloud_[cloud_idx];
			for (int idx_x = 0; idx_x < terrain_voxel_width_ - 1; idx_x++) {
				terrain_voxel_cloud_[cloud_idx] =
					terrain_voxel_cloud_[cloud_idx + terrain_voxel_width_];
				cloud_idx += terrain_voxel_width_;
			}
			terrain_voxel_cloud_[cloud_idx] = cloud_ptr;
			terrain_voxel_cloud_[cloud_idx]->clear();
		}
		terrain_voxel_shift_.x++;  
		terrain_voxel_cen_x = terrain_voxel_size_ * terrain_voxel_shift_.x;
	}

	while (robot_state_.y - terrain_voxel_cen_y < -terrain_voxel_size_) {
		for (int idx_x = 0; idx_x < terrain_voxel_width_; idx_x++) {
			int cloud_idx = terrain_voxel_width_ * idx_x + terrain_voxel_width_ - 1;
			pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr = terrain_voxel_cloud_[cloud_idx];
			for (int idx_y = terrain_voxel_width_ - 1; idx_y >= 1; idx_y--) {
				terrain_voxel_cloud_[cloud_idx] = terrain_voxel_cloud_[cloud_idx - 1];
				cloud_idx--;
			}
			terrain_voxel_cloud_[cloud_idx] = cloud_ptr;
			terrain_voxel_cloud_[cloud_idx]->clear();
		}
		terrain_voxel_shift_.y--;
		terrain_voxel_cen_y = terrain_voxel_size_ * terrain_voxel_shift_.y;
	}

	while (robot_state_.y - terrain_voxel_cen_y > terrain_voxel_size_) {
		for (int idx_x = 0; idx_x < terrain_voxel_width_; idx_x++) {
			int cloud_idx = terrain_voxel_width_ * idx_x; 
			pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr = terrain_voxel_cloud_[cloud_idx];
			for (int idx_y = 0; idx_y < terrain_voxel_width_ - 1; idx_y++) {
				terrain_voxel_cloud_[cloud_idx] = terrain_voxel_cloud_[cloud_idx + 1];
				cloud_idx++;
			}
			terrain_voxel_cloud_[cloud_idx] = cloud_ptr;
			terrain_voxel_cloud_[cloud_idx]->clear();
		}
		terrain_voxel_shift_.y++;
		terrain_voxel_cen_y = terrain_voxel_size_ * terrain_voxel_shift_.y;
	}
}

void  TerrainAnalysis::TerrainCloudDecayHandler() {
	for (int i = 0; i < terrain_voxel_num_; i++) {
		double time_interval = scan_time_stamp_ - sys_init_time_ -
								terrain_voxel_update_time_[i];

		if (terrain_voxel_update_num_[i] >= terrrain_voxel_num_update_thre_ || 
			time_interval >= terrain_voxel_time_update_thre_) { 
			pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr = terrain_voxel_cloud_[i];
			cloud_down_sampled_->clear(); 
			down_size_filter_.setInputCloud(cloud_ptr);
			down_size_filter_.filter(*cloud_down_sampled_);
			cloud_ptr->clear();
			int cloud_size = cloud_down_sampled_->points.size();

			for (int i = 0; i < cloud_size; i++) {
				pcl::PointXYZI laser_point(cloud_down_sampled_->points[i]);
				float dis = EuclideanNorm<float>(laser_point.x - robot_state_.x,
													laser_point.y - robot_state_.y);
				time_interval = scan_time_stamp_ - sys_init_time_ - laser_point.intensity;

				if (laser_point.z - robot_state_.z > min_rel_z_ - dis_ratio_z_ * dis &&
					laser_point.z - robot_state_.z < max_rel_z_ + dis_ratio_z_ * dis &&
					(time_interval < decay_time_ || dis < no_decay_dis_)) {
						cloud_ptr->push_back(laser_point);
				}
			}
			terrain_voxel_update_num_[i] = 0;
			terrain_voxel_update_time_[i] = scan_time_stamp_ - sys_init_time_;
		}
	}
}

void TerrainAnalysis::NoDataObsHandler() {
	for (int i = 0; i < planar_voxel_num_; i++) {
		int planar_pt_elev_size = planar_point_elev_[i].size();
		if (planar_pt_elev_size < min_block_pt_num_) {  // voxel no data
			planar_voxel_edge_[i] = 1;  // consider as negative obs candidate
		}
	}

	for (int skip_count = 0; skip_count < no_data_block_skip_num_; skip_count++) {
		for (int i = 0; i < planar_voxel_num_; i++) {
			if (planar_voxel_edge_[i] < 1) continue;  // exist data skip
			int idx_x = int(i / planar_voxel_width_);
			int idx_y = i % planar_voxel_width_;
			bool is_edge_voxel = false;
			for (int dx = -1; dx <= 1; dx++) {
				for (int dy = -1; dy <= 1; dy++) {
					if (idx_x + dx >= 0 && idx_x + dx < planar_voxel_width_ &&
						idx_y + dy >= 0 && idx_y + dy < planar_voxel_width_) {
						int voxel_idx = planar_voxel_width_ * (idx_x + dx) + idx_y + dy;
						if (planar_voxel_edge_[voxel_idx] < planar_voxel_edge_[i])
							is_edge_voxel = true;
					}
				}
			}
			if (!is_edge_voxel) planar_voxel_edge_[i]++;
		}
	}

	for (int i = 0; i < planar_voxel_num_; i++) {
		if (planar_voxel_edge_[i] <= no_data_block_skip_num_) continue;
		int idx_x = int(i / planar_voxel_width_);
		int idx_y = i % planar_voxel_width_;
		pcl::PointXYZI laser_point;
		laser_point.x = 
			planar_voxel_size_ * (idx_x - planar_width_half_) + robot_state_.x;

		laser_point.y = 
			planar_voxel_size_ * (idx_y - planar_width_half_) + robot_state_.y;

		laser_point.z = robot_state_.z;
		laser_point.intensity = robot_height_;

		laser_point.x -= planar_voxel_size_ / 4.0;  // bottom left 
		laser_point.y -= planar_voxel_size_ / 4.0;
		terrain_cloud_elev_->push_back(laser_point);

		laser_point.x += planar_voxel_size_ / 2.0;  // bottom right
		terrain_cloud_elev_->push_back(laser_point);

		laser_point.y += planar_voxel_size_ / 2.0;  // top right
		terrain_cloud_elev_->push_back(laser_point);

		laser_point.x -= planar_voxel_size_ / 2.0;  // top left
		terrain_cloud_elev_->push_back(laser_point);
	}
}

VoxelIndex TerrainAnalysis::LidarPt2TerrainCoord(const pcl::PointXYZI &point) {
	VoxelIndex idx;
	idx.x = int((point.x - robot_state_.x + (terrain_voxel_size_ / 2)) /
			terrain_voxel_size_) + terrain_width_half_;
	idx.y = int((point.y - robot_state_.y + (terrain_voxel_size_ / 2)) /
			terrain_voxel_size_) + terrain_width_half_;
	if (point.x - robot_state_.x + (terrain_voxel_size_ / 2 < 0)) idx.x--;
	if (point.y - robot_state_.y + (terrain_voxel_size_ / 2 < 0)) idx.y--;

	return idx;
}

VoxelIndex TerrainAnalysis::LidarPt2PlanarCoord(const pcl::PointXYZI &point) {
	VoxelIndex idx;
	idx.x = int((point.x - robot_state_.x + (planar_voxel_size_ / 2)) /
			planar_voxel_size_) + planar_width_half_;
	idx.y = int((point.y - robot_state_.y + (planar_voxel_size_ / 2)) /
			planar_voxel_size_) + planar_width_half_;
	if (point.x - robot_state_.x + (planar_voxel_size_ / 2 < 0)) idx.x--;
	if (point.y - robot_state_.y + (planar_voxel_size_ / 2 < 0)) idx.y--;

	return idx;
}

pcl::RGB TerrainAnalysis::heightToColor(float z, float base_height) {
    pcl::RGB color;

    float range = 0.025f; //
    float interval = 0.2f; //

    if (std::abs(z - base_height) <= range) {
        color.r = 0;
        color.g = 255;
        color.b = 0;
    } 
    else {
        float diff = z - base_height - range;
        int level = static_cast<int>(std::floor(std::abs(diff) / interval));
        if (diff < 0) level = -level;
        level = (level > 2) ? 2 : level;

        RGB tmp = kDistinctColors[level];
        color.r = tmp.r;
        color.g = tmp.g;
        color.b = tmp.b;
    }

    return color;
}

}  // tib_k331_perception


int main(int argc, char **argv) {
	ros::init(argc, argv, "terrain_analysis_node");
	ros::NodeHandle nh_private("~");
	tib_k331_perception::TerrainAnalysis terrain_analysis(nh_private);
	ROS_INFO("Terrain Analysis Ongoing~");
	terrain_analysis.AnalysisInstance();

	return 0;
}