#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;
using namespace Eigen;

// 使用你代码中的数据类型定义
typedef pcl::PointXYZ SCPointType;

class ScanContextLocalizer {
public:
    // 配置参数 (根据你的雷达型号和场景修改)
    const int PC_NUM_RING = 20;        // 行数 (半径划分)
    const int PC_NUM_SECTOR = 60;      // 列数 (角度划分)
    const double PC_MAX_RADIUS = 10.0; // 雷达最大有效探测半径
    const double LIDAR_HEIGHT = 0.0;   // 传感器高度补偿
    const double SC_DIST_THRES = 0.2; // 匹配成功的距离阈值
    const double SEARCH_RATIO = 0.1;   // 快速比对时的搜索范围比例
    const double PC_UNIT_SECTORANGLE = 360.0 / PC_NUM_SECTOR;

    ScanContextLocalizer() {}

    // ================= 1. 构建全局地图描述子库 =================
    // 在系统初始化阶段，遍历全局地图的关键帧，提取并存储所有的 Scan Context
    void addGlobalMapNode(pcl::PointCloud<SCPointType>::Ptr map_frame, int node_id) {
        MatrixXd sc = makeScancontext(*map_frame);
        MatrixXd ringkey = makeRingkeyFromScancontext(sc);
        
        global_contexts_.push_back(sc);
        global_ringkeys_.push_back(ringkey);
        node_ids_.push_back(node_id); // 记录该特征对应全局地图中的哪个关键帧或坐标点
        
        // 实际工程中，这里还会将 ringkey 插入 nanoflann KD-Tree 以加速搜索
    }

    // ================= 2. 全局重定位查询 =================
    // 输入当前局部帧，返回最匹配的全局 Node ID，匹配距离，以及计算出的 Yaw 角偏移量
    bool recognize(pcl::PointCloud<SCPointType>::Ptr local_frame, 
                   int& best_node_id, 
                   float& yaw_diff_rad, 
                   double& best_score) 
    {
        if (global_contexts_.empty()) {
            cout << "Error: Global map database is empty!" << endl;
            return false;
        }

        // 提取当前帧特征
        MatrixXd curr_sc = makeScancontext(*local_frame);
        MatrixXd curr_ringkey = makeRingkeyFromScancontext(curr_sc);

        // 步骤 1：基于 RingKey 进行粗筛 (这里用暴力搜索演示，实际用你代码里的 KD-Tree)
        // RingKey 具有旋转不变性，适合快速筛选候选帧
        std::vector<std::pair<double, int>> candidate_distances;
        for (size_t i = 0; i < global_ringkeys_.size(); i++) {
            double dist = (curr_ringkey - global_ringkeys_[i]).norm();
            candidate_distances.push_back({dist, i});
        }
        // 选出最相似的前 N 个候选者 (例如 10 个)
        int num_candidates = std::min(10, (int)candidate_distances.size());
        std::sort(candidate_distances.begin(), candidate_distances.end());

        // 步骤 2：基于完整的 Scan Context 矩阵计算精确相似度与 Yaw 偏移
        double min_dist = 10000000.0;
        int best_align_shift = 0;
        int best_idx = -1;

        for (int i = 0; i < num_candidates; i++) {
            int candidate_idx = candidate_distances[i].second;
            MatrixXd candidate_sc = global_contexts_[candidate_idx];

            // 使用你提供的距离与角度估算函数
            std::pair<double, int> sc_dist_result = distanceBtnScanContext(curr_sc, candidate_sc);
            
            if (sc_dist_result.first < min_dist) {
                min_dist = sc_dist_result.first;
                best_align_shift = sc_dist_result.second;
                best_idx = candidate_idx;
            }
        }

        best_score = min_dist;

        // 步骤 3：判断是否重定位成功
        if (min_dist < SC_DIST_THRES) {
            best_node_id = node_ids_[best_idx];
            // 列平移量乘以每列的角度，即为当前帧相对于全局地图的 Yaw 角度偏移
            yaw_diff_rad = (best_align_shift * PC_UNIT_SECTORANGLE) * M_PI / 180.0; 
            cout << "[Relocalization Success] Matched Map Node: " << best_node_id 
                 << ", Score: " << min_dist << ", Yaw Diff: " << yaw_diff_rad * 180.0 / M_PI << " deg." << endl;
            return true;
        } else {
            cout << "[Relocalization Failed] Nearest Score: " << min_dist << " (Threshold: " << SC_DIST_THRES << ")" << endl;
            return false;
        }
    }

private:
    std::vector<MatrixXd> global_contexts_;
    std::vector<MatrixXd> global_ringkeys_;
    std::vector<int> node_ids_;

    // ================= 以下为你提供的核心算法逻辑 =================
    float xy2theta(const float& _x, const float& _y) {
        if ((_x >= 0) & (_y >= 0)) return (180 / M_PI) * atan(_y / _x);
        if ((_x < 0) & (_y >= 0)) return 180 - ((180 / M_PI) * atan(_y / (-_x)));
        if ((_x < 0) & (_y < 0)) return 180 + ((180 / M_PI) * atan(_y / _x));
        if ((_x >= 0) & (_y < 0)) return 360 - ((180 / M_PI) * atan((-_y) / _x));
        return 0;
    }

    MatrixXd circshift(MatrixXd& _mat, int _num_shift) {
        if (_num_shift == 0) return _mat;
        MatrixXd shifted_mat = MatrixXd::Zero(_mat.rows(), _mat.cols());
        for (int col_idx = 0; col_idx < _mat.cols(); col_idx++) {
            int new_location = (col_idx + _num_shift) % _mat.cols();
            shifted_mat.col(new_location) = _mat.col(col_idx);
        }
        return shifted_mat;
    }

    double distDirectSC(MatrixXd& _sc1, MatrixXd& _sc2) {
        int num_eff_cols = 0;
        double sum_sector_similarity = 0;
        for (int col_idx = 0; col_idx < _sc1.cols(); col_idx++) {
            VectorXd col_sc1 = _sc1.col(col_idx);
            VectorXd col_sc2 = _sc2.col(col_idx);
            if ((col_sc1.norm() == 0) || (col_sc2.norm() == 0)) continue;
            double sector_similarity = col_sc1.dot(col_sc2) / (col_sc1.norm() * col_sc2.norm());
            sum_sector_similarity += sector_similarity;
            num_eff_cols++;
        }
        return num_eff_cols > 0 ? (1.0 - (sum_sector_similarity / num_eff_cols)) : 1.0;
    }

    int fastAlignUsingVkey(MatrixXd& _vkey1, MatrixXd& _vkey2) {
        int argmin_vkey_shift = 0;
        double min_veky_diff_norm = 10000000;
        for (int shift_idx = 0; shift_idx < _vkey1.cols(); shift_idx++) {
            MatrixXd vkey2_shifted = circshift(_vkey2, shift_idx);
            MatrixXd vkey_diff = _vkey1 - vkey2_shifted;
            double cur_diff_norm = vkey_diff.norm();
            if (cur_diff_norm < min_veky_diff_norm) {
                argmin_vkey_shift = shift_idx;
                min_veky_diff_norm = cur_diff_norm;
            }
        }
        return argmin_vkey_shift;
    }

    std::pair<double, int> distanceBtnScanContext(MatrixXd& _sc1, MatrixXd& _sc2) {
        MatrixXd vkey_sc1 = makeSectorkeyFromScancontext(_sc1);
        MatrixXd vkey_sc2 = makeSectorkeyFromScancontext(_sc2);
        int argmin_vkey_shift = fastAlignUsingVkey(vkey_sc1, vkey_sc2);

        const int SEARCH_RADIUS = round(0.5 * SEARCH_RATIO * _sc1.cols());
        std::vector<int> shift_idx_search_space{argmin_vkey_shift};
        for (int ii = 1; ii < SEARCH_RADIUS + 1; ii++) {
            shift_idx_search_space.push_back((argmin_vkey_shift + ii + _sc1.cols()) % _sc1.cols());
            shift_idx_search_space.push_back((argmin_vkey_shift - ii + _sc1.cols()) % _sc1.cols());
        }

        int argmin_shift = 0;
        double min_sc_dist = 10000000;
        for (int num_shift : shift_idx_search_space) {
            MatrixXd sc2_shifted = circshift(_sc2, num_shift);
            double cur_sc_dist = distDirectSC(_sc1, sc2_shifted);
            if (cur_sc_dist < min_sc_dist) {
                argmin_shift = num_shift;
                min_sc_dist = cur_sc_dist;
            }
        }
        return make_pair(min_sc_dist, argmin_shift);
    }

    MatrixXd makeScancontext(pcl::PointCloud<SCPointType>& _scan_down) {
        const int NO_POINT = -1000;
        MatrixXd desc = NO_POINT * MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);
        for (int pt_idx = 0; pt_idx < _scan_down.points.size(); pt_idx++) {
            SCPointType pt = _scan_down.points[pt_idx];
            pt.z += LIDAR_HEIGHT;
            float azim_range = sqrt(pt.x * pt.x + pt.y * pt.y);
            float azim_angle = xy2theta(pt.x, pt.y);

            if (azim_range > PC_MAX_RADIUS) continue;

            int ring_idx = std::max(std::min(PC_NUM_RING, int(ceil((azim_range / PC_MAX_RADIUS) * PC_NUM_RING))), 1);
            int sctor_idx = std::max(std::min(PC_NUM_SECTOR, int(ceil((azim_angle / 360.0) * PC_NUM_SECTOR))), 1);

            if (desc(ring_idx - 1, sctor_idx - 1) < pt.z) 
                desc(ring_idx - 1, sctor_idx - 1) = pt.z; 
        }

        for (int row_idx = 0; row_idx < desc.rows(); row_idx++)
            for (int col_idx = 0; col_idx < desc.cols(); col_idx++)
                if (desc(row_idx, col_idx) == NO_POINT) desc(row_idx, col_idx) = 0;
        return desc;
    }

    MatrixXd makeRingkeyFromScancontext(Eigen::MatrixXd& _desc) {
        Eigen::MatrixXd invariant_key(_desc.rows(), 1);
        for (int row_idx = 0; row_idx < _desc.rows(); row_idx++) {
            invariant_key(row_idx, 0) = _desc.row(row_idx).mean();
        }
        return invariant_key;
    }

    MatrixXd makeSectorkeyFromScancontext(Eigen::MatrixXd& _desc) {
        Eigen::MatrixXd variant_key(1, _desc.cols());
        for (int col_idx = 0; col_idx < _desc.cols(); col_idx++) {
            variant_key(0, col_idx) = _desc.col(col_idx).mean();
        }
        return variant_key;
    }
};