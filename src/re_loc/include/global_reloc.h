#pragma once

#include <iostream>
#include <vector>
#include <map>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>

#include "scan_context.h" 

class UnifiedGlobalRelocalizer {
public:
    UnifiedGlobalRelocalizer(double submap_radius, double icp_thresh, int icp_iters)
        : submap_radius_(submap_radius), icp_thresh_(icp_thresh), icp_iters_(icp_iters) {}

    void addKeyframeToDatabase(pcl::PointCloud<pcl::PointXYZ>::Ptr keyframe, 
                               int node_id, 
                               Eigen::Matrix4f global_pose) {
        sc_localizer_.addGlobalMapNode(keyframe, node_id);
        node_poses_[node_id] = global_pose; 
    }

    bool localize(pcl::PointCloud<pcl::PointXYZ>::Ptr global_map,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr local_frame,
                  Eigen::Matrix4f& final_pose) {
        // ---------------------------------------------------------
        // Scan Context Global Place Recognition
        // ---------------------------------------------------------
        std::cout << "Running Scan Context global search..." << std::endl;
        int best_node_id = -1;
        float sc_yaw_diff_rad = 0.0;
        double sc_score = 0.0;        
        if (!sc_localizer_.recognize(local_frame, best_node_id, sc_yaw_diff_rad, sc_score)) {
            std::cout << "=> Scan Context failed to find a valid candidate in the global map." << std::endl;
            return false;
        }

        // candidate global pose
        Eigen::Matrix4f matched_node_pose = node_poses_[best_node_id];
        std::cout << "=> Found candidate! Node ID: " << best_node_id 
             << " at Center(" << matched_node_pose << ")" << std::endl;

        // SC yaw matrix
        Eigen::AngleAxisf yaw_rotation(sc_yaw_diff_rad, Eigen::Vector3f::UnitZ());
        Eigen::Matrix4f sc_yaw_transform = Eigen::Matrix4f::Identity();
        sc_yaw_transform.block<3,3>(0,0) = yaw_rotation.toRotationMatrix();

        // initial guess
        Eigen::Matrix4f initial_guess_pose = matched_node_pose * sc_yaw_transform;

        // submap crop
        std::cout << "Extracting local submap from global map..." << std::endl;
        Eigen::Vector3f submap_center = initial_guess_pose.block<3,1>(0,3);
        pcl::PointCloud<pcl::PointXYZ>::Ptr submap = extractSubmap(global_map, submap_center);

        // ---------------------------------------------------------
        // ICP Fine Alignment
        // ---------------------------------------------------------
        std::cout << "Running ICP fine alignment..." << std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr local_initial_aligned(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*local_frame, *local_initial_aligned, initial_guess_pose);

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(local_initial_aligned);
        icp.setInputTarget(submap);               
        icp.setMaxCorrespondenceDistance(icp_thresh_); 
        icp.setMaximumIterations(icp_iters_);

        pcl::PointCloud<pcl::PointXYZ> Final;
        icp.align(Final);

        if (icp.hasConverged()) {
            final_pose = icp.getFinalTransformation() * initial_guess_pose;
            std::cout << "============================================" << std::endl;
            std::cout << "✅ Relocalization SUCCESS!" << std::endl;
            std::cout << "ICP Fitness Score: " << icp.getFitnessScore() << std::endl;
            std::cout << "Final Pose:\n" << final_pose << std::endl;
            std::cout << "============================================" << std::endl;
            return true;
        } 
        else {
            std::cout << "=> ICP did not converge." << std::endl;
            return false;
        }     
    }

private:
    ScanContextLocalizer sc_localizer_;
    std::map<int, Eigen::Matrix4f> node_poses_;
    int icp_iters_;
    double submap_radius_, icp_thresh_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr extractSubmap(
        pcl::PointCloud<pcl::PointXYZ>::Ptr global_map, 
        Eigen::Vector3f center) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr submap(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>());
        kdtree->setInputCloud(global_map);

        pcl::PointXYZ pt(center.x(), center.y(), center.z());
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        if (kdtree->radiusSearch(pt, submap_radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
            for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
                submap->points.push_back(global_map->points[pointIdxRadiusSearch[i]]);
            }
        }
        submap->width = submap->points.size();
        submap->height = 1;
        submap->is_dense = true;
        return submap;
    }
};