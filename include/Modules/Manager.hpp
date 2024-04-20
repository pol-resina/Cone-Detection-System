#ifndef MANAGER_HPP
#define MANAGER_HPP

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
