#ifndef MANAGER_HPP
#define MANAGER_HPP

#include <as_msgs/ObservationArray.h>

#include "sensor_msgs/PointCloud2.h"
#include "Modules/Config.hpp"
#include "Modules/Ransac.hpp"
#include "Modules/Clustering.hpp"
#include "nav_msgs/Odometry.h"

#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class Manager {
  private:
    ros::Publisher pubGround;
    ros::Publisher pubClusters;
    ros::Publisher pubObs;
    ros::Publisher pubVelComp;
    ros::Publisher pubPreRANSAC;  // to debug
    
    Ransac ransac;
    Clustering clustering;

    bool publish_debug_;

    std_msgs::Header header1_, header2_;

    /* PUBLISHERS  */
    void publishGround(sensor_msgs::PointCloud2 &msg);
    void publishCompensatedVelodyne(sensor_msgs::PointCloud2 &msg);
    void publishClusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &clusters);
    void publishObservations(as_msgs::ObservationArray &obs_vector);

    /* SAVE TIME*/
    void saveTime(const std::string &filename, std::chrono::duration<double> elapsed);

  public:
    // Constructor
    Manager(ros::NodeHandle &nh);

    // Callbacks
    void velodyneCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
    void limoveloCallback(const nav_msgs::Odometry::ConstPtr& odom_msg, const sensor_msgs::PointCloud2::ConstPtr& pcl_msg);
};

#endif // MANAGER_HPP
