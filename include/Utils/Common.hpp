#ifndef COMMON_H
#define COMMON_H
// Libraries
#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <chrono>
// Data Structures
#include <deque>
#include <vector>
// ROS messages
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
// PCL Library
#define PCL_NO_PRECOMPILE
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace LIDAR_TYPE {
    const std::string Velodyne = "velodyne";
    const std::string Hesai = "hesai";
    const std::string Ouster = "ouster";
    const std::string Custom = "custom";
}

struct InitializationParams {
    std::vector<double> times;
    std::vector<double> deltas;
};

namespace velodyne_ros {   // util
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      float time;
      uint16_t ring;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

namespace hesai_ros {
    struct Point {
        PCL_ADD_POINT4D
        uint8_t intensity;
        double timestamp;
        uint16_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;
}

namespace full_info {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      PCL_ADD_RGB;
      float intensity;
      float range;
      double timestamp;
      uint16_t ring;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

namespace custom {
    // Example: point with lots of fields
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        PCL_ADD_RGB;
        float intensity;
        float range;
        double timestamp;
        uint16_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

namespace ouster_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      std::uint32_t t;
      std::uint16_t reflectivity;
      std::uint8_t  ring;
      std::uint32_t range;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,          // util
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (std::uint16_t, ring, ring)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(hesai_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (std::uint8_t, intensity, intensity)
    (double, timestamp, timestamp)
    (std::uint16_t, ring, ring)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(full_info::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, r, r)
    (float, g, g)
    (float, b, b)
    (float, intensity, intensity)
    (float, range, range)
    (double, timestamp, timestamp)
    (std::uint16_t, ring, ring)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint32_t, range, range)
)

// Example: point with lots of fields
POINT_CLOUD_REGISTER_POINT_STRUCT(custom::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, r, r)
    (float, g, g)
    (float, b, b)
    (float, intensity, intensity)
    (float, range, range)
    (double, timestamp, timestamp)
    (std::uint16_t, ring, ring)
)

typedef sensor_msgs::PointCloud2::ConstPtr PointCloud_msg;
typedef sensor_msgs::Imu::ConstPtr IMU_msg;
typedef double TimeType;

class Point;
class IMU;
class State;
class RotTransl;
typedef std::deque<Point> Points;
typedef std::vector<Point, Eigen::aligned_allocator<Point>> PointVector;
typedef std::deque<IMU> IMUs;
typedef std::deque<State> States;

#endif