#ifndef MANAGER_HPP
#define MANAGER_HPP

#include <as_msgs/ObservationArray.h>

#include "sensor_msgs/PointCloud2.h"
#include "Modules/Config.hpp"
#include "Modules/Ransac.hpp"
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
    void publishClusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &clusters);
    void publishObservations(as_msgs::ObservationArray &obs_vector);

    /* SAVE TIME*/
    void saveTime(const std::string &filename, std::chrono::duration<double> elapsed);

  public:
    // Constructor
    Manager(ros::NodeHandle &nh);

    // Callbacks
    void velodyneCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
};

#endif // MANAGER_HPP
