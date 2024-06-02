#include "Utils/Common.hpp"

#include <deque>
#include <stdexcept>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Geometry>  // For Quaternion
#include <sensor_msgs/Imu.h>  // ROS message header for sensor_msgs::Imu

// extern struct Params Config;

template <typename ContentType>
class Buffer {
    public:
        std::deque<ContentType> content;
        Buffer();

        void push(const ContentType& cnt);
        // void pop_front();
        // void pop_back();    
        // ContentType front();
        // ContentType back();
        // bool empty();
        // int size();
        // void clear();
        // void clear(TimeType t);
};

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
