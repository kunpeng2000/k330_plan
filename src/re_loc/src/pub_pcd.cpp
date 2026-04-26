#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_publisher_node");
    ros::NodeHandle nh;

    if (argc < 2)
    {
        ROS_ERROR("Usage: rosrun <your_package_name> pcd_publisher <path_to_pcd_file>");
        return -1;
    }

    std::string pcd_file_path = argv[1];

    ros::Publisher pcd_pub = nh.advertise<sensor_msgs::PointCloud2>("/pub_pcd", 1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    ROS_INFO("Loading PCD file: %s", pcd_file_path.c_str());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path, *cloud) == -1)
    {
        ROS_ERROR("Couldn't read file %s \n", pcd_file_path.c_str());
        return (-1);
    }
    ROS_INFO("Loaded %lu data points from file.", cloud->points.size());

    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud, output_msg);

    output_msg.header.frame_id = "camera_init"; 

    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        output_msg.header.stamp = ros::Time::now();

        pcd_pub.publish(output_msg);
        ROS_INFO("Publish the pcd pointcloud.");

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}