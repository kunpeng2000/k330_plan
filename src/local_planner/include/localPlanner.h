#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <chrono>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

const double PI = 3.1415926;

#define PLOTPATHSET 1

namespace tib_k331_perception {

class localPlanner{
public:
    localPlanner(const ros::NodeHandle& nh);
    ~localPlanner() = default;
    void planInstance();

private:
    double robot_length_ = 0.6;
    double robot_width_ = 0.6;
    double sensor_offset_x_ = 0;
    double sensor_offset_y_ = 0;

    double laser_voxel_size_ = 0.05;
    double terrain_voxel_size_ = 0.2;
    double min_rel_z_ = -0.5;
    double max_rel_z_ = 0.25;

    bool twoway_drive_flag_ = false;
    bool use_terrain_analysis_flag_ = true;
    bool check_obs_flag_ = true;
    bool check_rot_obs_flag_ = false;
    bool use_cost_flag_ = true;
    bool dir_to_robot_flag_ = false;
    bool autonomy_mode_flag_ = true;
    bool path_scale_by_speed_flag_ = true;
    bool path_range_by_speed_flag_ = true;
    bool path_crop_by_goal_flag_ = true;
    bool mannual_nav_goal_flag_ = true;

    double adjacent_range_ = 5.0; 
    double obs_height_thres_ = 0.2;
    double ground_height_thres_ = 0.1;
    double cost_height_thres_ = 0.;
    double min_cost_score_ = 0.02;
    double dir_weight_ = 0.02;
    
    int point_per_path_thres_ = 2;
    double goal_angle_diff_thres_ = 90.0;
    
    double path_scale_ = 1.0;
    double min_path_scale_ = 0.75;
    double min_path_range_ = 1.0;
    double path_scale_step_ = 0.25;
    double path_range_step_ = 0.5;

    double autonomy_speed_ = 1.0;
    double max_speed_ = 1.0;
    
    double goal_x_ = 0;
    double goal_y_ = 0;
    double goal_clear_range_ = 0.5;

    std::string path_folder_;
    std::string path_frame_id_ = "vehicle";
    
    float joy_speed_ = 0;
    int laser_cloud_count_ = 0;
    float goal_dir_ = 0;
    double odom_time_ = 0;
    float robot_roll_ = 0, robot_pitch_ = 0, robot_yaw_ = 0;
    float robot_x_ = 0, robot_y_ = 0, robot_z_ = 0;

    bool new_laser_cloud_flag_ = false;
    bool new_terrain_cloud_flag_ = false;

    static constexpr int laser_cloud_stack_num_ = 1;
    static constexpr int path_num_ = 343;
    static constexpr int group_num_ = 7;
    float grid_voxel_size_ = 0.02;
    float search_radius_ = 0.45;
    float grid_voxel_offset_x_ = 3.2;
    float grid_voxel_offset_y_ = 4.5;
    static constexpr int grid_voxel_num_x_ = 161;
    static constexpr int grid_voxel_num_y_ = 451;
    static constexpr int grid_voxel_num_ = grid_voxel_num_x_ * grid_voxel_num_y_;

    int path_list_[path_num_] = {0};
    float end_dir_path_list_[path_num_] = {0};
    int clear_path_list_[36 * path_num_] = {0};
    float penalty_path_list_[36 * path_num_] = {0};
    float clear_path_per_group_score_[36 * group_num_] = {0};
    std::vector<int> correspondences_[grid_voxel_num_];

    nav_msgs::Path path_;

    ros::NodeHandle nh_;
    ros::Subscriber sub_odom_;
	ros::Subscriber sub_laser_cloud_;
	ros::Subscriber sub_terrain_cloud_;
    ros::Subscriber sub_goal_;
	ros::Subscriber sub_boundary_;
	ros::Publisher pub_path_;
	ros::Publisher pub_free_paths_;
    ros::Publisher pub_planner_cloud_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_crop_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_dwz_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_crop_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_removed_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_dwz_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr planner_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr planner_cloud_crop_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr boundary_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr free_paths_cloud_;

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> laser_cloud_stacked_;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> start_paths_;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> paths_;
    pcl::VoxelGrid<pcl::PointXYZI> laser_cloud_dwz_filter_, terrain_cloud_dwz_filter_;

    void checkReachGoal(float x, float y, float z);
    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odom);
    void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloud2);
    void terrainCloudHandler(const sensor_msgs::PointCloud2ConstPtr& terrainCloud2);
    void goalPointHandler(const geometry_msgs::PointStamped::ConstPtr& goal);
    void goalPoseHandler(const geometry_msgs::PoseStamped::ConstPtr& goal);
    void speedHandler(const std_msgs::Float32::ConstPtr& speed);
    void boundaryHandler(const geometry_msgs::PolygonStamped::ConstPtr& boundary);
    int readPlyHeader(FILE *filePtr);
    void readStartPaths();
    void readPaths();
    void readPathList();
    void readCorrespondences();
    void ParamsHandler(const ros::NodeHandle &nh);
};

}  // tib_k331_perception