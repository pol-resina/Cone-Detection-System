cmake_minimum_required(VERSION 3.0.2)
project(ftfcd)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_ros
  std_msgs
  geometry_msgs
  sensor_msgs
  as_msgs
  tf
)

generate_messages(
  DEPENDENCIES
  as_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dbscone
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ftfcd
  include/Modules

  src/main.cpp

  src/Modules/Manager.cpp
  src/Modules/Ransac.cpp
  src/Modules/Clustering.cpp

  src/Utils/Cluster.cpp
  src/Utils/Dbscan.cpp
  src/Utils/OctreeGenerator.cpp
)

add_dependencies(${PROJECT_NAME} as_msgs_generate_messages_cpp)

target_link_libraries(ftfcd ${catkin_LIBRARIES})