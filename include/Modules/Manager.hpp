#ifndef MANAGER_HPP
#define MANAGER_HPP

// #include <as_msgs/ObservationArray.h>
// #include <geometry_msgs/PoseArray.h>
// #include <nav_msgs/Odometry.h>
// #include <pcl/filters/crop_box.h>
// #include <pcl/octree/octree_search.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <ros/ros.h>

// #include "Objects/Params.hpp"

#include "sensor_msgs/PointCloud2.h"
#include "Modules/Config.hpp"
#include "Modules/Ransac.hpp"

class Manager {
  private:
    ros::Publisher pubGround;
    
    Ransac ransac;

    bool publish_debug_;

    void publishGround(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &ground) const;

  public:
    // Constructor
    Manager(ros::NodeHandle &nh);

    // Callbacks
    void velodyneCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
};

#endif // MANAGER_HPP