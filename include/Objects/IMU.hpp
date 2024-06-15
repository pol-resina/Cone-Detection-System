#ifndef IMU_HPP
#define IMU_HPP

#include "Utils/Common.hpp"
#include "Modules/Config.hpp"
#include "Utils/Utils.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>  // For Quaternion
#include <sensor_msgs/Imu.h>  // ROS message header for sensor_msgs::Imu

class IMU {
    public:
        Eigen::Vector3f a;
        Eigen::Vector3f w;
        Eigen::Quaternionf q;
        TimeType time;

        IMU();
        IMU(const sensor_msgs::Imu::ConstPtr& msg);

        IMU(const sensor_msgs::Imu& imu);
        IMU (const Eigen::Vector3f& a, const Eigen::Vector3f& w, double time);
        IMU (double time);
        bool has_orientation();
};

#endif