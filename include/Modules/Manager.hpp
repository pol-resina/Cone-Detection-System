#ifndef MANAGER_HPP
#define MANAGER_HPP

#include <as_msgs/ObservationArray.h>

#include "sensor_msgs/PointCloud2.h"
#include "Modules/Config.hpp"
#include "Modules/Ransac.hpp"
#include "Modules/Clustering.hpp"
#include "Modules/Compensator.hpp"
#include "../src/Objects/Buffer.cpp"
#include "nav_msgs/Odometry.h"

#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <sensor_msgs/Imu.h>

class Manager {
  private:
    ros::Publisher pubGround;
    ros::Publisher pubClusters;
    ros::Publisher pubObs;
    ros::Publisher pubVelComp;
    ros::Publisher pubPreRANSAC;  // to debug
    
    Ransac ransac;
    Clustering clustering;
    Compensator compensator;

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

    // Buffer IMUs
    Buffer<IMU> BUFFER_I;

    // Callbacks
    void velodyneCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
    void IMUCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);
    void limoveloCallback(const nav_msgs::Odometry::ConstPtr& odom_msg, const sensor_msgs::PointCloud2::ConstPtr& pcl_msg);
};

#endif // MANAGER_HPP
