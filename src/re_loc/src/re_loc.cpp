#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>

#include "global_reloc.h"

Eigen::Matrix4f poseToMatrix(double tx, double ty, double tz, double qx, double qy, double qz, double qw) {
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    Eigen::Quaternionf q(qw, qx, qy, qz);
    T.block<3,3>(0,0) = q.toRotationMatrix();
    T.block<3,1>(0,3) << tx, ty, tz;

    return T;
}

class OnlineRelocalizationNode {
public:
    OnlineRelocalizationNode(ros::NodeHandle& nh) 
        : nh_(nh), relocalizer_(30.0, 1.0, 100), is_processing_(false), init_done_(false)
    {
        nh_.param<std::string>("cloud_topic", cloud_topic_, "/velodyne_points");
        nh_.param<std::string>("data_dir", data_dir_, "/tmp/dataset_collection/");
        nh_.param<std::string>("map_frame", map_frame_, "map");
        nh_.param<std::string>("base_frame", base_frame_, "base_link");
        nh_.param<int>("avg_count", n_avg_, 5);
        nh_.param<double>("global_map_pub_freq", global_map_pub_freq_, 0.5);
        
        global_map_.reset(new pcl::PointCloud<pcl::PointXYZ>());

        if (!loadDatabaseAndMap()) {
            ROS_ERROR("Failed to load database or global map. Shutting down.");
            ros::shutdown();
            return;
        }

        map_pub_  = nh_.advertise<sensor_msgs::PointCloud2>("global_map", 1, true);
        odom_pub_  = nh_.advertise<nav_msgs::Odometry>("reloc_odom", 1, true);
        cloud_sub_ = nh_.subscribe(cloud_topic_, 1, &OnlineRelocalizationNode::cloudCallback, this);

        map_pub_timer_ = nh_.createTimer(ros::Duration(1.0 / global_map_pub_freq_), &OnlineRelocalizationNode::mapTimerCallback, this);
        
        ROS_INFO("Online Relocalization Node is Ready. Waiting for point clouds on [%s]...", cloud_topic_.c_str());
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher map_pub_, odom_pub_;  
    ros::Timer map_pub_timer_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
    
    std::string cloud_topic_, data_dir_, map_frame_, base_frame_;
    int n_avg_;
    double global_map_pub_freq_;
    bool is_processing_, init_done_;
    std::vector<Eigen::Matrix4f> pose_buffer_;

    UnifiedGlobalRelocalizer relocalizer_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
    sensor_msgs::PointCloud2 global_map_msg_;
    

    bool loadDatabaseAndMap() {
        std::string odom_file = data_dir_ + "ground_truth_odom.txt";
        std::string cloud_dir = data_dir_ + "clouds/";
        std::string global_map_path = data_dir_ + "global_map.pcd";

        ROS_INFO("Loading global map from: %s", global_map_path.c_str());
        if (pcl::io::loadPCDFile(global_map_path, *global_map_) == -1) {
            ROS_ERROR("Global map not found.");
            return false;
        }
        pcl::toROSMsg(*global_map_, global_map_msg_);
        global_map_msg_.header.frame_id = map_frame_;

        ROS_INFO("Loading ScanContext database from: %s", odom_file.c_str());
        std::ifstream infile(odom_file);
        if (!infile.is_open()) {
            ROS_ERROR("Could not open odom file.");
            return false;
        }

        std::string line;
        int node_count = 0;
        getline(infile, line); // 跳过表头

        while (getline(infile, line)) {
            if (line.empty() || line[0] == '#') continue;

            std::stringstream ss(line);
            double timestamp;
            std::string pcd_name;
            double tx, ty, tz, qx, qy, qz, qw;
            ss >> timestamp >> pcd_name >> tx >> ty >> tz >> qx >> qy >> qz >> qw;

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            if (pcl::io::loadPCDFile(cloud_dir + pcd_name, *cloud) == -1) continue;

            Eigen::Matrix4f pose = poseToMatrix(tx, ty, tz, qx, qy, qz, qw);
            
            relocalizer_.addKeyframeToDatabase(cloud, node_count, pose);
            node_count++;
        }
        
        ROS_INFO("Successfully loaded %d keyframes into the database.", node_count);
        return true;
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        if (is_processing_ || init_done_) {
            return; 
        }
        is_processing_ = true;

        ROS_INFO("Received new frame. Starting relocalization...");

        pcl::PointCloud<pcl::PointXYZ>::Ptr current_frame(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *current_frame);

        // 可选：在这里可以对 current_frame 进行降采样去噪，例如 VoxelGrid 滤波
        // 这样可以进一步加快 ScanContext 和 FPFH 的计算速度

        Eigen::Matrix4f estimated_pose = Eigen::Matrix4f::Identity();
        
        ros::Time start_time = ros::Time::now();
        bool success = relocalizer_.localize(global_map_, current_frame, estimated_pose);
        ros::Time end_time = ros::Time::now();

        if (success) {
            ROS_INFO("Relocalization SUCCESS! Time cost: %.3f seconds", (end_time - start_time).toSec());
            
            Eigen::Vector3f trans = estimated_pose.block<3,1>(0,3);
            ROS_INFO("Current Absolute Position -> X: %.2f, Y: %.2f, Z: %.2f", trans.x(), trans.y(), trans.z());
            
            pose_buffer_.push_back(estimated_pose);
            ROS_INFO("Relocalization success. Buffer: %zu/%d", pose_buffer_.size(), n_avg_);

            if (pose_buffer_.size() >= static_cast<size_t>(n_avg_)) {
                Eigen::Matrix4f avg_pose = computeAveragePose(pose_buffer_);
                publishTfAndOdom(avg_pose, msg->header.stamp);
                
                ROS_INFO("--- Published AVERAGED pose ---");
                // pose_buffer_.clear(); 
                init_done_ = true;
            }
            
        } 
        else {
            ROS_WARN("Relocalization FAILED for the current frame.");
        }

        is_processing_ = false;
    }

    Eigen::Matrix4f computeAveragePose(const std::vector<Eigen::Matrix4f>& poses) {
        Eigen::Vector3f avg_translation = Eigen::Vector3f::Zero();
        
        // 四元数平均逻辑：简单累加后归一化在小范围偏差内是有效的
        float qw = 0, qx = 0, qy = 0, qz = 0;
        Eigen::Quaternionf first_q(poses[0].block<3,3>(0,0));

        for (const auto& T : poses) {
            avg_translation += T.block<3,1>(0,3);

            Eigen::Quaternionf q(T.block<3,3>(0,0));
            if (q.dot(first_q) < 0.0f) {
                q.coeffs() *= -1.0f;
            }
            qw += q.w(); qx += q.x(); qy += q.y(); qz += q.z();
        }

        float inv_n = 1.0f / static_cast<float>(poses.size());
        avg_translation *= inv_n;

        Eigen::Quaternionf avg_q(qw, qx, qy, qz);
        avg_q.normalize();

        Eigen::Matrix4f avg_T = Eigen::Matrix4f::Identity();
        avg_T.block<3,3>(0,0) = avg_q.toRotationMatrix();
        avg_T.block<3,1>(0,3) = avg_translation;
        
        return avg_T;
    }

    void publishTfAndOdom(const Eigen::Matrix4f& T, const ros::Time& stamp) {
        // TF
        geometry_msgs::TransformStamped tf_msg;
        tf_msg.header.stamp = stamp;
        tf_msg.header.frame_id = map_frame_;
        tf_msg.child_frame_id = base_frame_;
        
        tf_msg.transform.translation.x = T(0, 3);
        tf_msg.transform.translation.y = T(1, 3);
        tf_msg.transform.translation.z = T(2, 3);

        Eigen::Quaternionf q(T.block<3,3>(0,0));
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_msg.transform.rotation.w = q.w();
        static_tf_broadcaster_.sendTransform(tf_msg);

        // odom
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = stamp;
        odom_msg.header.frame_id = map_frame_;
        odom_msg.child_frame_id = base_frame_;
        
        Eigen::Affine3d affine_pose(T.cast<double>());
        odom_msg.pose.pose = tf2::toMsg(affine_pose);
        odom_pub_.publish(odom_msg);
    }

    void mapTimerCallback(const ros::TimerEvent& event) {
        if (map_pub_.getNumSubscribers() > 0) {
            global_map_msg_.header.stamp = ros::Time::now();
            map_pub_.publish(global_map_msg_);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "online_relocalizer_node");
    ros::NodeHandle nh("~");

    OnlineRelocalizationNode node(nh);

    ros::spin();

    return 0;
}