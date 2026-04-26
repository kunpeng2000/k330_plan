cmake_minimum_required(VERSION 3.0.2)
project(init_relocalizer)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
  pcl_conversions
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES online_relocalizer
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_eigen pcl_conversions
#  DEPENDS PCL Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_definitions(${PCL_DEFINITIONS})


add_executable(re_loc 
  src/re_loc.cpp 
  # src/global_reloc.cpp
)
add_executable(save_data 
  src/save_pcd_odom.cpp 
)
add_executable(pub_pcd 
  src/pub_pcd.cpp 
)

target_link_libraries(re_loc
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(save_data
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(pub_pcd
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)