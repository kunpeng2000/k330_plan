/**
 * @file localPlanner.cpp
 * @brief local planner for MobiRo @ tib_k331
 * @author Alex Liu
 * @author kunpeng fan (Modified)
 * @copyright Copyright (c) 2026. Licensed under the MIT License.
 * * @acknowledgement 
 * this file is referenced and adapted from the following open-source project:
 * Repository: https://github.com/HongbiaoZ/autonomous_exploration_development_environment
 */

#include "localPlanner.h"

namespace tib_k331_perception {

localPlanner::localPlanner(const ros::NodeHandle& nh): 
	nh_(nh), laser_cloud_stacked_(laser_cloud_stack_num_), start_paths_(group_num_), paths_(path_num_){
	ParamsHandler(nh_);

	sub_odom_ = nh_.subscribe<nav_msgs::Odometry>
							("/odometry/filtered", 5, &localPlanner::odometryHandler, this);
	sub_laser_cloud_ = nh_.subscribe<sensor_msgs::PointCloud2>
							("/cloud_registered", 5, &localPlanner::laserCloudHandler, this);
	sub_terrain_cloud_ = nh_.subscribe<sensor_msgs::PointCloud2>
							("/terrain/terrain_map", 5, &localPlanner::terrainCloudHandler, this);
	sub_boundary_ = nh_.subscribe<geometry_msgs::PolygonStamped> 
							("/navigation_boundary", 5, &localPlanner::boundaryHandler, this);
	if (mannual_nav_goal_flag_){
		sub_goal_ = nh_.subscribe<geometry_msgs::PoseStamped> 
							("/move_base_simple/goal", 5, &localPlanner::goalPoseHandler, this);
	}
	else{
		sub_goal_ = nh_.subscribe<geometry_msgs::PointStamped> 
							("/way_point", 5, &localPlanner::goalPointHandler, this);
	}	
	
	pub_path_ = nh_.advertise<nav_msgs::Path> ("/local_planner/path", 5);
	pub_free_paths_ = nh_.advertise<sensor_msgs::PointCloud2> ("/local_planner/free_paths", 2);
	pub_planner_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/local_planner/plannar_cloud", 2);

	laser_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
	laser_cloud_crop_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
	laser_cloud_dwz_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
	terrain_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
	terrain_cloud_crop_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
	terrain_cloud_removed_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
	terrain_cloud_dwz_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
	planner_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
	planner_cloud_crop_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
	boundary_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
	free_paths_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

	for (auto& ptr : laser_cloud_stacked_) {
		ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
	}
	for (auto& ptr : start_paths_) {
		ptr.reset(new pcl::PointCloud<pcl::PointXYZ>);
	}
	for (auto& ptr : paths_) {
		ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
	}

	laser_cloud_dwz_filter_.setLeafSize(laser_voxel_size_, laser_voxel_size_, laser_voxel_size_);
  	terrain_cloud_dwz_filter_.setLeafSize(terrain_voxel_size_, terrain_voxel_size_, terrain_voxel_size_);

	for (int i = 0; i < grid_voxel_num_; i++) {
    	correspondences_[i].resize(0);
  	}
}

void localPlanner::ParamsHandler(const ros::NodeHandle &nh) {
	nh.getParam("robot_length", robot_length_);
	nh.getParam("robot_width", robot_width_);
	nh.getParam("sensor_offset_x", sensor_offset_x_);
	nh.getParam("sensor_offset_y", sensor_offset_y_);
	
	nh.getParam("laser_voxel_size", laser_voxel_size_);
	nh.getParam("terrain_voxel_size", terrain_voxel_size_);
	nh.getParam("min_rel_z", min_rel_z_);
	nh.getParam("max_rel_z", max_rel_z_);

	nh.getParam("twoway_drive_flag", twoway_drive_flag_);
	nh.getParam("use_terrain_analysis_flag", use_terrain_analysis_flag_);
	nh.getParam("check_obs_flag", check_obs_flag_); //
	nh.getParam("check_rot_obs_flag", check_rot_obs_flag_); //
	nh.getParam("use_cost_flag", use_cost_flag_);
	nh.getParam("dir_to_robot_flag", dir_to_robot_flag_);
	nh.getParam("autonomy_mode_flag", autonomy_mode_flag_);
	nh.getParam("path_scale_by_speed_flag", path_scale_by_speed_flag_);
	nh.getParam("path_range_by_speed_flag", path_range_by_speed_flag_); 
	nh.getParam("path_crop_by_goal_flag", path_crop_by_goal_flag_);
	nh.getParam("mannual_nav_goal_flag", mannual_nav_goal_flag_);

	nh.getParam("adjacent_range", adjacent_range_); //
	nh.getParam("obs_height_thres", obs_height_thres_);
	nh.getParam("ground_height_thres", ground_height_thres_); //
	nh.getParam("cost_height_thres", cost_height_thres_); //
	nh.getParam("min_cost_score", min_cost_score_);
	nh.getParam("dir_weight", dir_weight_);
	
	nh.getParam("point_per_path_thres", point_per_path_thres_);
	nh.getParam("goal_angle_diff_thres", goal_angle_diff_thres_);

	nh.getParam("path_scale", path_scale_); //
	nh.getParam("min_path_scale", min_path_scale_);
	nh.getParam("min_path_range", min_path_range_); //
	nh.getParam("path_scale_step", path_scale_step_);
	nh.getParam("path_range_step", path_range_step_);
	
	nh.getParam("autonomy_speed", autonomy_speed_);
	nh.getParam("max_speed", max_speed_);
	
	nh.getParam("goal_x", goal_x_);
	nh.getParam("goal_y", goal_y_);
	nh.getParam("goal_clear_range", goal_clear_range_); //

	nh.getParam("path_folder", path_folder_);
	nh.getParam("path_frame_id", path_frame_id_);
}

void localPlanner::checkReachGoal(float x, float y, float z){
	if (!(goal_x_ == 0.0 && goal_y_ == 0.0)){
		float dis = sqrt((x - goal_x_) * (x - goal_x_) + (y - goal_y_) * (y - goal_y_));
		if (dis <= 0.05){
			ROS_INFO("Reach the Goal!");
		}
	}
}

void localPlanner::odometryHandler(const nav_msgs::Odometry::ConstPtr& odom){
  odom_time_ = odom->header.stamp.toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  robot_roll_ = roll;
  robot_pitch_ = pitch;
  robot_yaw_ = yaw;
  robot_x_ = odom->pose.pose.position.x - cos(yaw) * sensor_offset_x_ + sin(yaw) * sensor_offset_y_;
  robot_y_ = odom->pose.pose.position.y - sin(yaw) * sensor_offset_x_ - cos(yaw) * sensor_offset_y_;
  robot_z_ = odom->pose.pose.position.z;

  checkReachGoal(robot_x_, robot_y_, robot_z_);
}

void localPlanner::laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloud2){
	if (!use_terrain_analysis_flag_) {
		laser_cloud_->clear();
		pcl::fromROSMsg(*laserCloud2, *laser_cloud_);

		pcl::PointXYZI point;
		laser_cloud_crop_->clear();
		int laserCloudSize = laser_cloud_->points.size();
		for (int i = 0; i < laserCloudSize; i++) {
			point = laser_cloud_->points[i];

			float pointX = point.x;
			float pointY = point.y;
			float pointZ = point.z;

			float dis = sqrt((pointX - robot_x_) * (pointX - robot_x_) + (pointY - robot_y_) * (pointY - robot_y_));
			if (dis < adjacent_range_) {
			point.x = pointX;
			point.y = pointY;
			point.z = pointZ;
			laser_cloud_crop_->push_back(point);
			}
		}

		laser_cloud_dwz_->clear();
		laser_cloud_dwz_filter_.setInputCloud(laser_cloud_crop_);
		laser_cloud_dwz_filter_.filter(*laser_cloud_dwz_);

		new_laser_cloud_flag_ = true;
	}
}

void localPlanner::terrainCloudHandler(const sensor_msgs::PointCloud2ConstPtr& terrainCloud2){
	if (use_terrain_analysis_flag_) {
		terrain_cloud_->clear();
		pcl::fromROSMsg(*terrainCloud2, *terrain_cloud_);

		pcl::PointXYZI point;
		terrain_cloud_crop_->clear();
		terrain_cloud_removed_->clear();
		int terrainCloudSize = terrain_cloud_->points.size();
		for (int i = 0; i < terrainCloudSize; i++) {
			point = terrain_cloud_->points[i];

			float pointX = point.x;
			float pointY = point.y;
			float pointZ = point.z;

			float dis = sqrt((pointX - robot_x_) * (pointX - robot_x_) + (pointY - robot_y_) * (pointY - robot_y_));
			if (dis < adjacent_range_ && (point.intensity > obs_height_thres_ || use_cost_flag_)) {
				point.x = pointX;
				point.y = pointY;
				point.z = pointZ;
				terrain_cloud_crop_->push_back(point);
			}
			else{
				point.x = pointX;
				point.y = pointY;
				point.z = pointZ;
				terrain_cloud_removed_->push_back(point);
			}
		}

		terrain_cloud_dwz_->clear();
		terrain_cloud_dwz_filter_.setInputCloud(terrain_cloud_crop_);
		terrain_cloud_dwz_filter_.filter(*terrain_cloud_dwz_);

		new_terrain_cloud_flag_ = true;
	}
}

void localPlanner::goalPointHandler(const geometry_msgs::PointStamped::ConstPtr& goal)
{
	ROS_INFO("receive new goal");
	goal_x_ = goal->point.x;
	goal_y_ = goal->point.y;
}


void localPlanner::goalPoseHandler(const geometry_msgs::PoseStamped::ConstPtr& goal){
	ROS_INFO("receive new goal");
	goal_x_ = goal->pose.position.x;
	goal_y_ = goal->pose.position.y;
}

void localPlanner::boundaryHandler(const geometry_msgs::PolygonStamped::ConstPtr& boundary){
  boundary_cloud_->clear();
  pcl::PointXYZI point, point1, point2;
  int boundarySize = boundary->polygon.points.size();

  if (boundarySize >= 1) {
    point2.x = boundary->polygon.points[0].x;
    point2.y = boundary->polygon.points[0].y;
    point2.z = boundary->polygon.points[0].z;
  }

  for (int i = 0; i < boundarySize; i++) {
    point1 = point2;

    point2.x = boundary->polygon.points[i].x;
    point2.y = boundary->polygon.points[i].y;
    point2.z = boundary->polygon.points[i].z;

    if (point1.z == point2.z) {
      float disX = point1.x - point2.x;
      float disY = point1.y - point2.y;
      float dis = sqrt(disX * disX + disY * disY);

      int pointNum = int(dis / terrain_voxel_size_) + 1;
      for (int pointID = 0; pointID < pointNum; pointID++) {
        point.x = float(pointID) / float(pointNum) * point1.x + (1.0 - float(pointID) / float(pointNum)) * point2.x;
        point.y = float(pointID) / float(pointNum) * point1.y + (1.0 - float(pointID) / float(pointNum)) * point2.y;
        point.z = 0;
        point.intensity = 100.0;

        for (int j = 0; j < point_per_path_thres_; j++) {
          boundary_cloud_->push_back(point);
        }
      }
    }
  }
}

int localPlanner::readPlyHeader(FILE *filePtr){
	char str[50];
	int val, pointNum;
	std::string strCur, strLast;
	while (strCur != "end_header") {
		val = fscanf(filePtr, "%s", str);
		if (val != 1) {
			printf ("\nError reading input files, exit.\n\n");
			ros::shutdown();
			return 0;
		}

		strLast = strCur;
		strCur = std::string(str);

		if (strCur == "vertex" && strLast == "element") {
			val = fscanf(filePtr, "%d", &pointNum);
			if (val != 1) {
				printf ("\nError reading input files, exit.\n\n");
				ros::shutdown();
				return 0;
			}
		}
	}

	return pointNum;
}

void localPlanner::readStartPaths(){
	std::string fileName = path_folder_ + "/startPaths.ply";

	FILE *filePtr = fopen(fileName.c_str(), "r");
	if (filePtr == NULL) {
		ROS_FATAL("Cannot read StartPaths files! Current path folder is %s", path_folder_.c_str());
		ros::shutdown();
		return;
	}

	int pointNum = readPlyHeader(filePtr);

	pcl::PointXYZ point;
	int val1, val2, val3, val4, groupID;
	for (int i = 0; i < pointNum; i++) {
		val1 = fscanf(filePtr, "%f", &point.x);
		val2 = fscanf(filePtr, "%f", &point.y);
		val3 = fscanf(filePtr, "%f", &point.z);
		val4 = fscanf(filePtr, "%d", &groupID);

		if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1) {
		printf ("\nError reading StartPaths files, exit.\n\n");
			exit(1);
		}

		if (groupID >= 0 && groupID < group_num_) {
			start_paths_[groupID]->push_back(point);
		}
	}

	fclose(filePtr);
}

void localPlanner::readPaths(){
	std::string fileName = path_folder_ + "/paths.ply";

	FILE *filePtr = fopen(fileName.c_str(), "r");
	if (filePtr == NULL) {
		printf ("\nCannot read Paths files, exit.\n\n");
		exit(1);
	}

	int pointNum = readPlyHeader(filePtr);

	pcl::PointXYZI point;
	int pointSkipNum = 30;
	int pointSkipCount = 0;
	int val1, val2, val3, val4, val5, pathID;
	for (int i = 0; i < pointNum; i++) {
		val1 = fscanf(filePtr, "%f", &point.x);
		val2 = fscanf(filePtr, "%f", &point.y);
		val3 = fscanf(filePtr, "%f", &point.z);
		val4 = fscanf(filePtr, "%d", &pathID);
		val5 = fscanf(filePtr, "%f", &point.intensity);

		if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
		printf ("\nError reading Paths files, exit.\n\n");
			exit(1);
		}

		if (pathID >= 0 && pathID < path_num_) {
		pointSkipCount++;
		if (pointSkipCount > pointSkipNum) {
			paths_[pathID]->push_back(point);
			pointSkipCount = 0;
		}
		}
	}

	fclose(filePtr);
}

void localPlanner::readPathList(){
	std::string fileName = path_folder_ + "/pathList.ply";

	FILE *filePtr = fopen(fileName.c_str(), "r");
	if (filePtr == NULL) {
		printf ("\nCannot read PathList files, exit.\n\n");
		exit(1);
	}

	if (path_num_ != readPlyHeader(filePtr)) {
		printf ("\nIncorrect path number, exit.\n\n");
		exit(1);
	}

	int val1, val2, val3, val4, val5, pathID, groupID;
	float endX, endY, endZ;
	for (int i = 0; i < path_num_; i++) {
		val1 = fscanf(filePtr, "%f", &endX);
		val2 = fscanf(filePtr, "%f", &endY);
		val3 = fscanf(filePtr, "%f", &endZ);
		val4 = fscanf(filePtr, "%d", &pathID);
		val5 = fscanf(filePtr, "%d", &groupID);

		if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
		printf ("\nError reading PathList files, exit.\n\n");
			exit(1);
		}

		if (pathID >= 0 && pathID < path_num_ && groupID >= 0 && groupID < group_num_) {
		path_list_[pathID] = groupID;
		end_dir_path_list_[pathID] = 2.0 * atan2(endY, endX) * 180 / PI;
		}
	}

	fclose(filePtr);
}

void localPlanner::readCorrespondences(){
	std::string fileName = path_folder_ + "/correspondences.txt";

	FILE *filePtr = fopen(fileName.c_str(), "r");
	if (filePtr == NULL) {
		printf ("\nCannot read Correspondences files, exit.\n\n");
		exit(1);
	}

	int val1, gridVoxelID, pathID;
	for (int i = 0; i < grid_voxel_num_; i++) {
		val1 = fscanf(filePtr, "%d", &gridVoxelID);
		if (val1 != 1) {
		printf ("\nError reading Correspondences files, exit.\n\n");
			exit(1);
		}

		while (1) {
		val1 = fscanf(filePtr, "%d", &pathID);
		if (val1 != 1) {
			printf ("\nError reading Correspondences files, exit.\n\n");
			exit(1);
		}

		if (pathID != -1) {
			if (gridVoxelID >= 0 && gridVoxelID < grid_voxel_num_ && pathID >= 0 && pathID < path_num_) {
			correspondences_[gridVoxelID].push_back(pathID);
			}
		} else {
			break;
		}
		}
	}

	fclose(filePtr);
}

void localPlanner::planInstance(){

	joy_speed_ = autonomy_speed_ / max_speed_;

	if (joy_speed_ < 0) joy_speed_ = 0;
	else if (joy_speed_ > 1.0) joy_speed_ = 1.0;
  
	readStartPaths();
	readPaths();
	readPathList();
	readCorrespondences();
	
	ROS_INFO("Initialization complete.");

	ros::Rate rate(100);
	while (ros::ok()) {
    	ros::spinOnce();

    	if (!new_laser_cloud_flag_ && !new_terrain_cloud_flag_) {
			continue;
		}

		auto t_start = std::chrono::high_resolution_clock::now();

		if (new_laser_cloud_flag_) {
			new_laser_cloud_flag_ = false;

			laser_cloud_stacked_[laser_cloud_count_]->clear();
			*laser_cloud_stacked_[laser_cloud_count_] = *laser_cloud_dwz_;
			laser_cloud_count_ = (laser_cloud_count_ + 1) % laser_cloud_stack_num_;

			planner_cloud_->clear();
			for (int i = 0; i < laser_cloud_stack_num_; i++) {
				*planner_cloud_ += *laser_cloud_stacked_[i];
			}
		}
		
		if (new_terrain_cloud_flag_) {
			new_terrain_cloud_flag_ = false;

			planner_cloud_->clear();
			*planner_cloud_ = *terrain_cloud_dwz_;
		}

		float sin_robot_yaw = sin(robot_yaw_);
		float cos_robot_yaw = cos(robot_yaw_);

		// planner_cloud_crop_
		// crop cloud by adjacent_range_, min_rel_z_, max_rel_z_
		pcl::PointXYZI point_tmp;
		planner_cloud_crop_->clear();
		int planner_cloud_size = planner_cloud_->points.size();
		for (int i = 0; i < planner_cloud_size; i++) {
			float point_x_rel = planner_cloud_->points[i].x - robot_x_;
			float point_y_rel = planner_cloud_->points[i].y - robot_y_;
			float point_z_rel = planner_cloud_->points[i].z - robot_z_;

			point_tmp.x = point_x_rel * cos_robot_yaw + point_y_rel * sin_robot_yaw;
			point_tmp.y = -point_x_rel * sin_robot_yaw + point_y_rel * cos_robot_yaw;
			point_tmp.z = point_z_rel;
			point_tmp.intensity = planner_cloud_->points[i].intensity;

			float dis = sqrt(point_tmp.x * point_tmp.x + point_tmp.y * point_tmp.y);
			if (dis < adjacent_range_ && ((point_tmp.z > min_rel_z_ && point_tmp.z < max_rel_z_) || use_terrain_analysis_flag_)) {
				planner_cloud_crop_->push_back(point_tmp);
			}
		}

		int boundary_cloud_size = boundary_cloud_->points.size();
		for (int i = 0; i < boundary_cloud_size; i++) {
			point_tmp.x = ((boundary_cloud_->points[i].x - robot_x_) * cos_robot_yaw 
					+ (boundary_cloud_->points[i].y - robot_y_) * sin_robot_yaw);
			point_tmp.y = (-(boundary_cloud_->points[i].x - robot_x_) * sin_robot_yaw 
					+ (boundary_cloud_->points[i].y - robot_y_) * cos_robot_yaw);
			point_tmp.z = boundary_cloud_->points[i].z;
			point_tmp.intensity = boundary_cloud_->points[i].intensity;

			float dis = sqrt(point_tmp.x * point_tmp.x + point_tmp.y * point_tmp.y);
			if (dis < adjacent_range_) {
				planner_cloud_crop_->push_back(point_tmp);
			}
		}

		sensor_msgs::PointCloud2 planner_cloud_msg;
		pcl::toROSMsg(*planner_cloud_crop_, planner_cloud_msg);
		planner_cloud_msg.header.stamp = ros::Time().fromSec(odom_time_);
		planner_cloud_msg.header.frame_id = path_frame_id_;
		pub_planner_cloud_.publish(planner_cloud_msg);

		// goal_dis, goal_dir
		float path_range = adjacent_range_;
		if (path_range_by_speed_flag_) path_range = adjacent_range_ * joy_speed_;
		if (path_range < min_path_range_) path_range = min_path_range_;
		float goal_dis = adjacent_range_;

		if (autonomy_mode_flag_) {
			float relative_goal_x = ((goal_x_ - robot_x_) * cos_robot_yaw + (goal_y_ - robot_y_) * sin_robot_yaw);
			float relative_goal_y = (-(goal_x_ - robot_x_) * sin_robot_yaw + (goal_y_ - robot_y_) * cos_robot_yaw);

			goal_dis = sqrt(relative_goal_x * relative_goal_x + relative_goal_y * relative_goal_y);
			goal_dir_ = atan2(relative_goal_y, relative_goal_x) * 180 / PI;

			if (!twoway_drive_flag_) {
				if (goal_dir_ > 90.0) goal_dir_ = 90.0;
				else if (goal_dir_ < -90.0) goal_dir_ = -90.0;
			}
		}

		// path_scale_
		bool path_found_flag = false;
		float def_path_scale = path_scale_;
		if (path_scale_by_speed_flag_) {
			path_scale_ = def_path_scale * joy_speed_;
		}
		if (path_scale_ < min_path_scale_) {
			path_scale_ = min_path_scale_;
		}

		while (path_scale_ >= min_path_scale_ && path_range >= min_path_range_) {
			for (int i = 0; i < 36 * path_num_; i++) {
				clear_path_list_[i] = 0;
				penalty_path_list_[i] = 0;
			}
			for (int i = 0; i < 36 * group_num_; i++) {
				clear_path_per_group_score_[i] = 0;
			}

			float min_obs_ang_cw = -180.0;
			float min_obs_ang_ccw = 180.0;
			float robot_radius = sqrt(robot_length_ / 2.0 * robot_length_ / 2.0 + robot_width_ / 2.0 * robot_width_ / 2.0);
			float robot_angle_offset = atan2(robot_width_, robot_length_) * 180.0 / PI;

			// clear_path_list_, penalty_path_list_
			int planner_cloud_crop_size = planner_cloud_crop_->points.size();
			for (int i = 0; i < planner_cloud_crop_size; i++) {
				float x_rel = planner_cloud_crop_->points[i].x / path_scale_;
				float y_rel = planner_cloud_crop_->points[i].y / path_scale_;
				float height = planner_cloud_crop_->points[i].intensity;
				float dis = sqrt(x_rel * x_rel + y_rel * y_rel);

				if (dis < path_range / path_scale_ && 
					(dis <= (goal_dis + goal_clear_range_) / path_scale_ || !path_crop_by_goal_flag_) && 
					check_obs_flag_) {

					for (int rot_dir = 0; rot_dir < 36; rot_dir ++) {
						float rot_angle = (10.0 * rot_dir - 180.0) * PI / 180;
						float angle_diff = fabs(goal_dir_ - (10.0 * rot_dir - 180.0));

						if (angle_diff > 180.0) {
							angle_diff = 360.0 - angle_diff;
						}

						if ((angle_diff > goal_angle_diff_thres_ && !dir_to_robot_flag_) || 
							(fabs(10.0 * rot_dir - 180.0) > goal_angle_diff_thres_ && fabs(goal_dir_) <= 90.0 && dir_to_robot_flag_) ||
							((10.0 * rot_dir > goal_angle_diff_thres_ && 360.0 - 10.0 * rot_dir > goal_angle_diff_thres_) && fabs(goal_dir_) > 90.0 && dir_to_robot_flag_)) {
							continue;
						}

						// point in current path's coor
						float x_rel_rot = cos(rot_angle) * x_rel + sin(rot_angle) * y_rel;
						float y_rel_rot = -sin(rot_angle) * x_rel + cos(rot_angle) * y_rel;
						float scale_y = x_rel_rot / grid_voxel_offset_x_ + 
							search_radius_ / grid_voxel_offset_y_ * (grid_voxel_offset_x_ - x_rel_rot) / grid_voxel_offset_x_;
						int ind_x = int((grid_voxel_offset_x_ + grid_voxel_size_ / 2 - x_rel_rot) / grid_voxel_size_);
						int ind_y = int((grid_voxel_offset_y_ + grid_voxel_size_ / 2 - y_rel_rot / scale_y) / grid_voxel_size_);

						if (ind_x >= 0 && ind_x < grid_voxel_num_x_ && ind_y >= 0 && ind_y < grid_voxel_num_y_) {
							int ind = grid_voxel_num_y_ * ind_x + ind_y;
							int blocked_path_by_voxel_num = correspondences_[ind].size();
							for (int j = 0; j < blocked_path_by_voxel_num; j++) {
								if (height > obs_height_thres_ || !use_terrain_analysis_flag_) {
									clear_path_list_[path_num_ * rot_dir + correspondences_[ind][j]] ++;
								} 
								else {
									if (penalty_path_list_[path_num_ * rot_dir + correspondences_[ind][j]] < height && height > ground_height_thres_) {
										penalty_path_list_[path_num_ * rot_dir + correspondences_[ind][j]] = height;
									}
								}
							}
						}
					}
				}

				if (dis < robot_radius / path_scale_ && 
					(fabs(x_rel) > robot_length_ / path_scale_ / 2.0 || fabs(y_rel) > robot_width_ / path_scale_ / 2.0) && 
					(height > obs_height_thres_ || !use_terrain_analysis_flag_) && 
					check_rot_obs_flag_) {
						
					float obs_angle = atan2(y_rel, x_rel) * 180.0 / PI;
					if (obs_angle > 0) {
						if (min_obs_ang_ccw > obs_angle - robot_angle_offset) min_obs_ang_ccw = obs_angle - robot_angle_offset;
						if (min_obs_ang_cw < obs_angle + robot_angle_offset - 180.0) min_obs_ang_cw = obs_angle + robot_angle_offset - 180.0;
					} 
					else {
						if (min_obs_ang_cw < obs_angle + robot_angle_offset) min_obs_ang_cw = obs_angle + robot_angle_offset;
						if (min_obs_ang_ccw > 180.0 + obs_angle - robot_angle_offset) min_obs_ang_ccw = 180.0 + obs_angle - robot_angle_offset;
					}
				}
			}

			if (min_obs_ang_cw > 0) min_obs_ang_cw = 0;
			if (min_obs_ang_ccw < 0) min_obs_ang_ccw = 0;

			// score
			for (int i = 0; i < 36 * path_num_; i++) {
				int rot_dir = int(i / path_num_);
				float angle_diff = fabs(goal_dir_ - (10.0 * rot_dir - 180.0));
				if (angle_diff > 180.0) {
					angle_diff = 360.0 - angle_diff;
				}
				if ((angle_diff > goal_angle_diff_thres_ && !dir_to_robot_flag_) || 
					(fabs(10.0 * rot_dir - 180.0) > goal_angle_diff_thres_ && fabs(goal_dir_) <= 90.0 && dir_to_robot_flag_) ||
					((10.0 * rot_dir > goal_angle_diff_thres_ && 360.0 - 10.0 * rot_dir > goal_angle_diff_thres_) && fabs(goal_dir_) > 90.0 && dir_to_robot_flag_)) {
					continue;
				}

				if (clear_path_list_[i] < point_per_path_thres_) {
					float penalty_score = 1.0 - penalty_path_list_[i] / cost_height_thres_;
					if (penalty_score < min_cost_score_) penalty_score = min_cost_score_;

					float dir_diff = fabs(goal_dir_ - end_dir_path_list_[i % path_num_] - (10.0 * rot_dir - 180.0));
					if (dir_diff > 360.0) {
						dir_diff -= 360.0;
					}
					if (dir_diff > 180.0) {
						dir_diff = 360.0 - dir_diff;
					}

					float rot_dir_weight;
					if (rot_dir < 18) rot_dir_weight = fabs(fabs(rot_dir - 9) + 1);
					else rot_dir_weight = fabs(fabs(rot_dir - 27) + 1);
					float score = (1 - sqrt(sqrt(dir_weight_ * dir_diff))) * rot_dir_weight * rot_dir_weight * rot_dir_weight * rot_dir_weight * penalty_score;
					if (score > 0) {
						clear_path_per_group_score_[group_num_ * rot_dir + path_list_[i % path_num_]] += score;
					}
				}
			}

			float max_score = 0;
			int selected_group_id = -1;
			for (int i = 0; i < 36 * group_num_; i++) {
				int rot_dir = int(i / group_num_);
				float rot_angle = (10.0 * rot_dir - 180.0) * PI / 180;
				float rot_deg = 10.0 * rot_dir;
				if (rot_deg > 180.0) rot_deg -= 360.0;
				if (max_score < clear_path_per_group_score_[i] && 
					((rot_angle * 180.0 / PI > min_obs_ang_cw && rot_angle * 180.0 / PI < min_obs_ang_ccw) || 
					(rot_deg > min_obs_ang_cw && rot_deg < min_obs_ang_ccw && twoway_drive_flag_) || 
					!check_rot_obs_flag_)) {
					
					max_score = clear_path_per_group_score_[i];
					selected_group_id = i;
				}
			}

			if (selected_group_id >= 0) {
				int rot_dir = int(selected_group_id / group_num_);
				float rot_angle = (10.0 * rot_dir - 180.0) * PI / 180;

				selected_group_id = selected_group_id % group_num_;
				int selected_path_length = start_paths_[selected_group_id]->points.size();
				path_.poses.resize(selected_path_length);
				for (int i = 0; i < selected_path_length; i++) {
					float x = start_paths_[selected_group_id]->points[i].x;
					float y = start_paths_[selected_group_id]->points[i].y;
					float z = start_paths_[selected_group_id]->points[i].z;
					float dis = sqrt(x * x + y * y);

					if (dis <= path_range / path_scale_ && dis <= goal_dis / path_scale_) {
						path_.poses[i].pose.position.x = path_scale_ * (cos(rot_angle) * x - sin(rot_angle) * y);
						path_.poses[i].pose.position.y = path_scale_ * (sin(rot_angle) * x + cos(rot_angle) * y);
						path_.poses[i].pose.position.z = path_scale_ * z;
					} 
					else {
						path_.poses.resize(i);
						break;
					}
				}

				path_.header.stamp = ros::Time().fromSec(odom_time_);
				path_.header.frame_id = path_frame_id_;

				pub_path_.publish(path_);
				
				free_paths_cloud_->clear();
				for (int i = 0; i < 36 * path_num_; i++) {
					int rot_dir = int(i / path_num_);
					float rot_angle = (10.0 * rot_dir - 180.0) * PI / 180;
					float rot_deg = 10.0 * rot_dir;
					if (rot_deg > 180.0) {
						rot_deg -= 360.0;
					}
					float angle_diff = fabs(goal_dir_ - (10.0 * rot_dir - 180.0));
					if (angle_diff > 180.0) {
						angle_diff = 360.0 - angle_diff;
					}
					if ((angle_diff > goal_angle_diff_thres_ && !dir_to_robot_flag_) ||
						(fabs(10.0 * rot_dir - 180.0) > goal_angle_diff_thres_ &&
						fabs(goal_dir_) <= 90.0 && dir_to_robot_flag_) ||
						((10.0 * rot_dir > goal_angle_diff_thres_ && 360.0 - 10.0 * rot_dir > goal_angle_diff_thres_) && 
						fabs(goal_dir_) > 90.0 && dir_to_robot_flag_) || 
						!((rot_angle * 180.0 / PI > min_obs_ang_cw && rot_angle * 180.0 / PI < min_obs_ang_ccw) || 
						(rot_deg > min_obs_ang_cw && rot_deg < min_obs_ang_ccw && twoway_drive_flag_) || !check_rot_obs_flag_)) {
						continue;
					}

					if (clear_path_list_[i] < point_per_path_thres_) {
						int free_path_length = paths_[i % path_num_]->points.size();
						for (int j = 0; j < free_path_length; j++) {
							point_tmp = paths_[i % path_num_]->points[j];

							float x_rel = point_tmp.x;
							float y_rel = point_tmp.y;
							float z_rel = point_tmp.z;

							float dis = sqrt(x_rel * x_rel + y_rel * y_rel);
							if (dis <= path_range / path_scale_ && 
								(dis <= (goal_dis + goal_clear_range_) / path_scale_ || !path_crop_by_goal_flag_)) {

								point_tmp.x = path_scale_ * (cos(rot_angle) * x_rel - sin(rot_angle) * y_rel);
								point_tmp.y = path_scale_ * (sin(rot_angle) * x_rel + cos(rot_angle) * y_rel);
								point_tmp.z = path_scale_ * z_rel;
								point_tmp.intensity = 1.0;

								free_paths_cloud_->push_back(point_tmp);
							}
						}
					}
				}

				sensor_msgs::PointCloud2 free_paths_cloud_msg;
				pcl::toROSMsg(*free_paths_cloud_, free_paths_cloud_msg);
				free_paths_cloud_msg.header.stamp = ros::Time().fromSec(odom_time_);
				free_paths_cloud_msg.header.frame_id = path_frame_id_;
				pub_free_paths_.publish(free_paths_cloud_msg);
			}

			if (selected_group_id < 0) {
				if (path_scale_ >= min_path_scale_ + path_scale_step_) {
					path_scale_ -= path_scale_step_;
					path_range = adjacent_range_ * path_scale_ / def_path_scale;
				} 
				else {
					path_range -= path_range_step_;
				}
			} 
			else {
				path_found_flag = true;
				break;
			}
		}
		path_scale_ = def_path_scale;

		if (!path_found_flag) {
			path_.poses.resize(1);
			path_.poses[0].pose.position.x = 0;
			path_.poses[0].pose.position.y = 0;
			path_.poses[0].pose.position.z = 0;

			path_.header.stamp = ros::Time().fromSec(odom_time_);
			path_.header.frame_id = path_frame_id_;
			pub_path_.publish(path_);

			free_paths_cloud_->clear();
			sensor_msgs::PointCloud2 free_paths_cloud_msg;
			pcl::toROSMsg(*free_paths_cloud_, free_paths_cloud_msg);
			free_paths_cloud_msg.header.stamp = ros::Time().fromSec(odom_time_);
			free_paths_cloud_msg.header.frame_id = path_frame_id_;
			pub_free_paths_.publish(free_paths_cloud_msg);
		}

		auto t_end = std::chrono::high_resolution_clock::now();
		auto ms = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count();
    	// std::cout << "planInstance: " << ms << " microseconds" << std::endl;

		rate.sleep();

	}
}

}  // tib_k331_perception

int main(int argc, char **argv) {
	ros::init(argc, argv, "local_planner_node");
	ros::NodeHandle nh_private("~");
	tib_k331_perception::localPlanner local_planner(nh_private);
	ROS_INFO("Local Planner Ongoing~");
	local_planner.planInstance();

	return 0;
}
