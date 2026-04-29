# Falco Navigation & Relocalization

This project implements an autonomous navigation system based on Falco, integrated with the Scan Context algorithm, to achieve global relocalization relying on a prior map. The system allows a robot to initialize from any unknown starting point and accurately determine its initial pose within the global map upon departure.

## 📍 Coordinate Systems

This system primarily relies on the following four core coordinate frames:
1. **`map`**: The global coordinate frame. The global point cloud map is built in this frame.
2. **`camera_init`**: The odometry initialization frame (i.e., the odometry origin when the robot starts).
3. **`start`**: The result of relocalization (Mid360 tilted placement).
4. **`base_link`**: The high-frequency odometry (robot body) frame.
5. **`aft_mapped`**: The FAST-LIVO2 low-frequency odometry frame.

---

## 🚀 Pipeline

The complete operation pipeline is divided into three main stages: Mapping & Data Collection, Global Relocalization, and Autonomous Navigation.

### 1. Global Mapping and Offline Frame Saving

This step is used to construct the prior map of the environment and record the feature data required for relocalization.

```bash
roslaunch init_relocalizer save_data.launch
```

**Description:** Start FAST-LIVO2 for global mapping. During the mapping process, the system will automatically save local point cloud frames and their corresponding odometry poses according to a preset distance threshold. Upon completion, the complete global point cloud map is saved.

**Core Parameters:**
* `cloud_topic`: The topic name for receiving the local point cloud.
* `odom_topic`: The topic name for receiving the odometry.
* `save_dir`: The directory to save the local point cloud frames and odometry information.
* `save_distance_thresh`: The distance threshold for saving keyframes (i.e., save one frame of data every $n$ meters traveled).

### 2. Global Relocalization

In an environment with an existing prior map, when the robot restarts, this node is used to retrieve its position within the global map.

```bash
roslaunch init_relocalizer re_loc.launch
```

**Description:** Start FAST-LIVO2. The system will extract the current scan frame, perform Scan Context matching with the prior data, calculate the current starting position, and publish the TF transformation between `map` and `start`.
> **💡 Tip:** When the terminal outputs **`Published AVERAGED pose`**, it indicates that the multi-frame averaged relocalization is complete and the system pose has converged. The downstream navigation module can then be launched safely.

**Core Parameters:**
* `cloud_topic`: The topic name for receiving the local point cloud.
* `data_dir`: The directory from which to read the local frames and odometry information.
* `global_map_file`: The file path to the global point cloud map (in .pcd format).
* `map_frame`: The name of the global map coordinate frame.
* `base_frame`: The odometry origin frame when the robot starts.
* `avg_count`: The number of times to smooth the relocalization pose (takes the average of $n$ relocalization results to improve accuracy).
* `global_map_pub_freq`: The publishing frequency of the global point cloud map (primarily used for RViz visualization).

### 3. Autonomous Navigation

Once relocalization is complete, launch the navigation module to execute movement tasks.

```bash
roslaunch terrain_analysis terrain_analysis_planner.launch
```

**Description:** Launch the Falco navigation and terrain analysis module. After setting a target point (Goal) in RViz, the planner will automatically generate a safe path and start publishing velocity control commands via the `cmd_vel` topic.

## 🔗 References & Acknowledgements
This project is built upon or inspired by several outstanding open-source projects. We express our gratitude to the authors for their contributions to the robotics community:

[FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2.git)

[Scan Context](https://github.com/gisbi-kim/SC-LIO-SAM.git)

[CMU Autonomous Exploration Development Environment](https://github.com/HongbiaoZ/autonomous_exploration_development_environment.git)

[Sophus](https://github.com/strasdat/Sophus.git)

[Vikit](https://github.com/xuankuzcr/rpg_vikit.git )

[ROS Robot Localization Pkg](https://github.com/cra-ros-pkg/robot_localization.git)