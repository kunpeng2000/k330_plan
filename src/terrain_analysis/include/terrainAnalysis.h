#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <thread>
#include <ros/ros.h>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "utils.h"

namespace tib_k331_perception {

struct RobotState {
	float x;
	float y;
	float z;
	float roll;
	float pitch;
	float yaw;
};

struct VoxelIndex {
	int x;
	int y;
};

struct RGB {
    uint8_t r, g, b;
};

const std::vector<RGB> kDistinctColors = {
    {255, 0, 0},      // 0  红
    {0, 0, 255},      // 1  蓝
    {255, 255, 0},    // 2  黄
};

class TerrainAnalysis {
public:
	TerrainAnalysis(const ros::NodeHandle& nh);
	~TerrainAnalysis();
	void AnalysisInstance();

private:
    void ParamsHandler(const ros::NodeHandle &nh);
	void OdometryCallback(const nav_msgs::Odometry::ConstPtr &odom);
	void ScanCallback(const sensor_msgs::PointCloud2ConstPtr &laser_in);
	void TerrainCenterUpdate();
	void TerrainCloudDecayHandler();
	void NoDataObsHandler();
	auto LidarPt2TerrainCoord(const pcl::PointXYZI &point) -> VoxelIndex;
	auto LidarPt2PlanarCoord(const pcl::PointXYZI &point) -> VoxelIndex;
	pcl::RGB heightToColor(float z, float base_height);

	ros::NodeHandle nh_;
	ros::Subscriber odom_sub_;
	ros::Subscriber laser_sub_;
	ros::Subscriber purge_sub_;
	ros::Publisher terrain_pub_;
	ros::Publisher local_terrain_pub_;
	ros::Publisher removed_height_cloud_pub_;
    ros::Publisher removed_dyn_cloud_pub_;
	ros::Publisher colored_pub_;

	pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud_current_;  // current cloudin
	pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_current_crop_;  // crop off the ground and ceiling
    pcl::PointCloud<pcl::PointXYZI>::Ptr *terrain_voxel_cloud_; // voxel stacked cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_down_sampled_;  // downsampled voxel stacked cloud
	pcl::PointCloud<pcl::PointXYZI>::Ptr local_terrain_cloud_; // local stacked cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr dyn_candidate_cloud_; // dynamic points candidate
	pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_elev_; // final terrain cloud
	pcl::PointCloud<pcl::PointXYZI>::Ptr removed_height_num_cloud_; // removed cloud in final terrain cloud due to height or points num
    pcl::PointCloud<pcl::PointXYZI>::Ptr removed_dyn_cloud_; // removed cloud in final terrain cloud due to dyn
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud_; // colored final terrain cloud
	pcl::VoxelGrid<pcl::PointXYZI> down_size_filter_;
    // local_terrain_cloud_ = terrain_cloud_elev_ + removed_dyn_cloud_ + removed_height_num_cloud_

    sensor_msgs::PointCloud2 local_terrain_msg_; // initial local cloud
    sensor_msgs::PointCloud2 terrain_cloud_msg_; // final terrain cloud
    sensor_msgs::PointCloud2 removed_height_cloud_msg_;
    sensor_msgs::PointCloud2 removed_dyn_cloud_msg_;
    sensor_msgs::PointCloud2 colored_terrain_cloud_msg_; // final colored terrain cloud

	int *terrain_voxel_update_num_;  // terrain voxel point num count
	float *terrain_voxel_update_time_;  // terrain voxel update interval
	float *planar_voxel_elev_;
	int *planar_voxel_edge_;
	int *planar_voxel_dyn_obs_;
	std::vector<float> *planar_point_elev_;

	// time stamps
	double sys_init_time_;
	double scan_time_stamp_;
    
	// flags
	bool system_inited_;
	bool new_laser_cloud_;
	int no_data_inited_;

	RobotState robot_state_;
	VoxelIndex terrain_voxel_shift_;

    // const terrain voxel parameters
    // terrain_voxel_cloud_
    const float terrain_voxel_size_ = 1.0;
    const int terrain_voxel_width_ = 21;  // width seems count by num
    const int terrain_width_half_ = (terrain_voxel_width_ - 1) / 2;
    const int terrain_voxel_num_ = terrain_voxel_width_ * terrain_voxel_width_;

    // const planar voxel parameters
    const float planar_voxel_size_ = 0.2;
    const int planar_voxel_width_ = 51;
    const int planar_width_half_ = (planar_voxel_width_ - 1) / 2;
    const int planar_voxel_num_ = planar_voxel_width_ * planar_voxel_width_;

    // variable params
    // terrain_voxel_cloud_
    int terrrain_voxel_num_update_thre_ = 100;
    double terrain_voxel_time_update_thre_ = 2.0;
    double downsample_voxel_size_ = 0.05;
    double min_rel_z_ = -0.8; // min z thres
    double max_rel_z_ = 0.8; // max z thres
    double dis_ratio_z_ = 0.2; // dis-ratio for z
    double decay_time_ = 5.0; // time thres
    double no_decay_dis_ = 4.0; // dis thres whatever time

    // dyn_candidate_cloud_
    double robot_height_ = 0.6;
    bool remove_dyn_obs_flag_ = true;
    double min_dyn_obs_dis_ = 0.3; // dyn dis thres
    double min_dyn_obs_rel_z_ = 0; // dyn z reference
    double min_dyn_obs_angle_ = 0; // dyn angle thres
    double min_dyn_obs_fov_ = -16.0; // dyn fov range
    double max_dyn_obs_fov_ = 16.0;
    int min_dyn_obs_pt_num_ = 0; // within min_dyn_obs_dis_

    // planar_voxel_elev_
    bool use_sorting_flag_ = true;
    double quantile_z_ = 0.25;
    double max_ground_lift_ = 0.2;
    bool limit_ground_lift_flag_ = false;
    
    // terrain_cloud_elev_
    int min_dyn_obs_pt_thres_ = 10; // dyn points num thres in final
    bool consider_drop_flag_ = false;
    int min_block_pt_num_ = 0; // point's floor voxel num thres 
    
    // on data obs
    bool no_data_obstacle_ = false;
    int no_data_block_skip_num_ = 0;

    std::string sub_odom_topic_;
    std::string sub_scan_topic_;
    std::string frame_id_;
};

}  // tib_k331_perception