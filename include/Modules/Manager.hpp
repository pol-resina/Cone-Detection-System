#ifndef MANAGER_HPP
#define MANAGER_HPP

#include <as_msgs/ObservationArray.h>

#include "sensor_msgs/PointCloud2.h"
#include "Modules/Config.hpp"
#include "Modules/Ransac.hpp"
#include "Utils/Dbscan.hpp"
#include "Modules/Clustering.hpp"

class Manager {
  private:
    ros::Publisher pubGround;
    ros::Publisher pubClusters;
    ros::Publisher pubObs;
    
    Ransac ransac;
    Clustering clustering;

    bool publish_debug_;

    std_msgs::Header header_;


    /* PUBLISHERS  */
    void publishGround(sensor_msgs::PointCloud2 &msg);
    void publishClusters(const std::vector<dbScanSpace::cluster> &clusters);
    void publishClusters2(const pcl::PointCloud<pcl::PointXYZI> &clusters);
    void publishObservations(std::vector<dbScanSpace::cluster> &clusters);

  public:
    // Constructor
    Manager(ros::NodeHandle &nh);

    // Callbacks
    void velodyneCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
};

#endif // MANAGER_HPP
