#ifndef MANAGER_HPP
#define MANAGER_HPP

// #include <as_msgs/ObservationArray.h>
// #include <geometry_msgs/PoseArray.h>
// #include <nav_msgs/Odometry.h>
// #include <pcl/filters/crop_box.h>
// #include <pcl/octree/octree_search.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <ros/ros.h>

#include "sensor_msgs/PointCloud2.h"
#include "Modules/Config.hpp"
#include "Modules/Ransac.hpp"
#include "Utils/Dbscan.hpp"

class Manager {
  private:
    ros::Publisher pubGround;
    ros::Publisher pubClusters;
    
    Ransac ransac;

    bool publish_debug_;

    void publishGround(sensor_msgs::PointCloud2 msg);

    void publishClusters(std::vector<dbScanSpace::cluster> clusters);

  public:
    // Constructor
    Manager(ros::NodeHandle &nh);

    // Callbacks
    void velodyneCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
};

#endif // MANAGER_HPP
