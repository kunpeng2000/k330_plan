#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>
#include <cmath>

namespace fs = boost::filesystem;

class DataSaverNode
{
public:
    DataSaverNode(ros::NodeHandle& nh) : nh_(nh), frame_count_(0), is_first_frame_(true)
    {
        nh_.param<std::string>("cloud_topic", cloud_topic_, "/velodyne_points");
        nh_.param<std::string>("odom_topic", odom_topic_, "/odom");
        nh_.param<std::string>("save_dir", save_dir_, "/map_data");
        nh_.param<double>("save_distance_thresh", save_distance_thresh_, 5.0); 

        cloud_dir_ = save_dir_ + "clouds/";
        if (!fs::exists(cloud_dir_))
        {
            fs::create_directories(cloud_dir_);
            ROS_INFO("Created directory: %s", cloud_dir_.c_str());
        }

        std::string odom_file_path = save_dir_ + "ground_truth_odom.txt";
        odom_file_.open(odom_file_path, std::ios::out | std::ios::trunc);
        if (!odom_file_.is_open())
        {
            ROS_ERROR("Failed to open odometry file: %s", odom_file_path.c_str());
            ros::shutdown();
        }
        
        odom_file_ << "# timestamp pcd_filename tx ty tz qx qy qz qw\n";

        cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, cloud_topic_, 10));
        odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh_, odom_topic_, 10));

        sync_.reset(new Sync(MySyncPolicy(10), *cloud_sub_, *odom_sub_));
        sync_->registerCallback(boost::bind(&DataSaverNode::syncCallback, this, _1, _2));

        ROS_INFO("Data Saver Node Initialized.");
        ROS_INFO("Distance Threshold set to: %.2f meters.", save_distance_thresh_);
    }

    ~DataSaverNode()
    {
        if (odom_file_.is_open())
        {
            odom_file_.close();
        }
        ROS_INFO("Data saving finished. Total saved frames: %d", frame_count_);
    }

private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;

    ros::NodeHandle nh_;
    std::string cloud_topic_, odom_topic_;
    std::string save_dir_, cloud_dir_;
    double save_distance_thresh_;

    boost::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
    boost::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
    boost::shared_ptr<Sync> sync_;

    std::ofstream odom_file_;
    int frame_count_;

    bool is_first_frame_;
    double last_saved_x_, last_saved_y_, last_saved_z_;

    void syncCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                      const nav_msgs::OdometryConstPtr& odom_msg)
    {
        double tx = odom_msg->pose.pose.position.x;
        double ty = odom_msg->pose.pose.position.y;
        double tz = odom_msg->pose.pose.position.z;

        if (!is_first_frame_){
            double dist = std::sqrt(std::pow(tx - last_saved_x_, 2) + 
                                    std::pow(ty - last_saved_y_, 2) + 
                                    std::pow(tz - last_saved_z_, 2));
            
            if (dist < save_distance_thresh_) {
                return; 
            }
        }

        last_saved_x_ = tx;
        last_saved_y_ = ty;
        last_saved_z_ = tz;
        is_first_frame_ = false;

        std::stringstream ss;
        ss << std::setfill('0') << std::setw(6) << frame_count_ << ".pcd";
        std::string pcd_filename = ss.str();
        std::string pcd_full_path = cloud_dir_ + pcd_filename;

        pcl::PointCloud<pcl::PointXYZI> pcl_cloud; 
        pcl::fromROSMsg(*cloud_msg, pcl_cloud);
        pcl::io::savePCDFileBinary(pcd_full_path, pcl_cloud);

        double timestamp = odom_msg->header.stamp.toSec();
        double qx = odom_msg->pose.pose.orientation.x;
        double qy = odom_msg->pose.pose.orientation.y;
        double qz = odom_msg->pose.pose.orientation.z;
        double qw = odom_msg->pose.pose.orientation.w;

        odom_file_ << std::fixed << std::setprecision(6) << timestamp << " " 
                   << pcd_filename << " "
                   << std::setprecision(6) << tx << " " << ty << " " << tz << " "
                   << qx << " " << qy << " " << qz << " " << qw << "\n";

        ROS_INFO("Saved frame %d at dist trigger. [XYZ: %.2f, %.2f, %.2f]", frame_count_, tx, ty, tz);
        
        frame_count_++;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_saver_node");
    ros::NodeHandle nh("~"); 

    DataSaverNode node(nh);

    ros::spin();

    return 0;
}