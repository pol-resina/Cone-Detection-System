#ifndef MANAGER_HPP
#define MANAGER_HPP

#include <as_msgs/ObservationArray.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <pcl/filters/crop_box.h>
#include <pcl/octree/octree_search.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

#include "Objects/Params.hpp"

class Manager {
  private:
    Manager();
    ~Manager();

    ros::Publisher pubGround_;

    Params::Manager params_;

    void publishGround(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &ground) const;

  public:
    static Manager& getInstance() {
      static Manager instance;
      return instance;
    }

    Manager(Manager const&) = delete;
    void operator=(Manager const&) = delete;

    void init(const Params &params,
              const ros::Publisher &pubGround);

    void velodyneCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);

    void saveRawpoints(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud) const;

    void run();
};

#endif // MANAGER_HPP