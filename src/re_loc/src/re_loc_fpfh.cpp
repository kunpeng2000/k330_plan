#include <iostream>
#include <vector>
#include <map>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>

#include "fpfh.h" 
#include "scan_context.h" 

using namespace std;

class UnifiedGlobalRelocalizer {
public:
    UnifiedGlobalRelocalizer(double submap_radius, double inlier_threshold)
        : submap_radius_(submap_radius), inlier_thresh_(inlier_threshold) {}

    // ====================================================================
    // 1. 离线建图阶段：向数据库注入全局关键帧
    // ====================================================================
    void addKeyframeToDatabase(pcl::PointCloud<pcl::PointXYZ>::Ptr keyframe, 
                               int node_id, 
                               Eigen::Vector3f global_position) 
    {
        // 将关键帧的 ScanContext 存入检索库
        sc_localizer_.addGlobalMapNode(keyframe, node_id);
        // 记录 NodeID 对应的真实世界 XYZ 坐标，用于后续裁剪子图
        node_positions_[node_id] = global_position; 
    }

    // ====================================================================
    // 2. 在线重定位阶段：输入局部帧，输出其在全局地图中的绝对位姿
    // ====================================================================
    bool localize(pcl::PointCloud<pcl::PointXYZ>::Ptr global_map,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr local_frame,
                  Eigen::Matrix4f& final_pose) 
    {
        int best_node_id = -1;
        float sc_yaw_diff_rad = 0.0;
        double sc_score = 0.0;

        // ---------------------------------------------------------
        // 第一步：Scan Context 全局位置识别 (Place Recognition)
        // ---------------------------------------------------------
        cout << "[Step 1] Running Scan Context global search..." << endl;
        if (!sc_localizer_.recognize(local_frame, best_node_id, sc_yaw_diff_rad, sc_score)) {
            cout << "=> Scan Context failed to find a valid candidate in the global map." << endl;
            return false;
        }

        // 获取候选节点的全局 XYZ 坐标
        Eigen::Vector3f candidate_center = node_positions_[best_node_id];
        cout << "=> Found candidate! Node ID: " << best_node_id 
             << " at Center(" << candidate_center.transpose() << ")" << endl;

        // ---------------------------------------------------------
        // 第二步：从庞大的全局地图中裁剪候选子图 (Submap Extraction)
        // ---------------------------------------------------------
        cout << "[Step 2] Extracting local submap from global map..." << endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr submap = extractSubmap(global_map, candidate_center);
        if (submap->points.size() < 500) {
            cout << "=> Error: Submap is too sparse!" << endl;
            return false;
        }

        // ---------------------------------------------------------
        // 第三步：FPFH 6-DoF 特征粗配准 (Coarse Alignment)
        // ---------------------------------------------------------
        // SC 只能提供 (x, y, yaw) 的粗略估计，要获得准确的 z, roll, pitch，
        // 必须在截取的子图内运行 FPFH 进行 6自由度 匹配。
        cout << "[Step 3] Running FPFH coarse alignment in the submap..." << endl;
        CFPFH MovFPFH, RefFPFH;
        MovFPFH.setInputCloud(local_frame);
        MovFPFH.compute();
        
        RefFPFH.setInputCloud(submap);
        RefFPFH.compute();

        // 调用你 reLocalizer.h 中的 RANSAC 估计
        Eigen::Matrix4f coarse_Tf = EstimateCorrespondence(MovFPFH, RefFPFH, inlier_thresh_);

        // 验证 FPFH 的结果是否靠谱 (计算内点率)
        pcl::PointCloud<pcl::PointXYZ>::Ptr local_aligned(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*local_frame, *local_aligned, coarse_Tf);
        double inlier_ratio = computeFitnessScore(local_aligned, submap);
        
        cout << "=> FPFH Coarse Inlier Ratio: " << inlier_ratio << endl;
        if (inlier_ratio < 0.25) { // 阈值可调
            cout << "=> Relocalization failed: FPFH match score too low." << endl;
            return false;
        }

        // ---------------------------------------------------------
        // 第四步：ICP 几何精配准 (Fine Alignment)
        // ---------------------------------------------------------
        cout << "[Step 4] Running ICP fine alignment..." << endl;
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(local_aligned);  // 使用 FPFH 粗配准后的点云
        icp.setInputTarget(submap);
        icp.setMaxCorrespondenceDistance(inlier_thresh_);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        
        pcl::PointCloud<pcl::PointXYZ> Final;
        icp.align(Final);

        if (icp.hasConverged()) {
            // 最终位姿 = ICP位姿补偿 * FPFH粗配准位姿
            final_pose = icp.getFinalTransformation() * coarse_Tf;
            cout << "============================================" << endl;
            cout << "✅ Relocalization SUCCESS!" << endl;
            cout << "ICP Fitness Score: " << icp.getFitnessScore() << endl;
            cout << "Final Pose:\n" << final_pose << endl;
            cout << "============================================" << endl;
            return true;
        } else {
            cout << "=> ICP did not converge." << endl;
            return false;
        }
    }

private:
    ScanContextLocalizer sc_localizer_;
    std::map<int, Eigen::Vector3f> node_positions_; // 记录节点ID到绝对XYZ坐标的映射
    double submap_radius_;
    double inlier_thresh_;

    // 半径搜索裁剪子图
    pcl::PointCloud<pcl::PointXYZ>::Ptr extractSubmap(
        pcl::PointCloud<pcl::PointXYZ>::Ptr global_map, 
        Eigen::Vector3f center) 
    {
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

    // 计算内点率得分
    double computeFitnessScore(
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_aligned, 
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_submap) 
    {
        pcl::search::KdTree<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(target_submap);
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);

        int inliers = 0;
        for (const auto& pt : source_aligned->points) {
            if (kdtree.nearestKSearch(pt, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
                if (sqrt(pointNKNSquaredDistance[0]) < inlier_thresh_) {
                    inliers++;
                }
            }
        }
        return static_cast<double>(inliers) / source_aligned->points.size();
    }
};