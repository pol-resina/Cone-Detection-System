#include "Modules/Manager.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h> 
#include <random>
#include <string>

extern struct Params Config;

int i=0;

Manager::Manager(ros::NodeHandle &nh): ransac(Config){
    pubGround = nh.advertise<sensor_msgs::PointCloud2>(Config.common.topics.output.ground, 20);
    pubClusters = nh.advertise<sensor_msgs::PointCloud2>(Config.common.topics.output.clusters, 20);
    this->publish_debug_ = Config.manager.publish_debug;
}

void Manager::velodyneCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    if (cloud_msg == nullptr) return;

    auto start = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground(new pcl::PointCloud<pcl::PointXYZI>);
    sensor_msgs::PointCloud2 msg;
    ransac.removeGround(cloud, msg, no_ground);

    if (publish_debug_) {
      this->publishGround(msg);
    }
    
    std::vector<htr::Point3D> groupA;
    dbScanSpace::dbscan dbscan;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloudRGB->width = no_ground->width;
    cloudRGB->height = no_ground->height;
    cloudRGB->is_dense = no_ground->is_dense;
    cloudRGB->points.resize(cloudRGB->width * cloudRGB->height);

    for (size_t i = 0; i < no_ground->points.size(); ++i)
    {
      pcl::PointXYZI point_i = no_ground->points[i];

      // Create a corresponding RGB point
      pcl::PointXYZRGB point_rgb;
      point_rgb.x = point_i.x;
      point_rgb.y = point_i.y;
      point_rgb.z = point_i.z;

      // Convert intensity to RGB color
      // For simplicity, let's assume intensity directly maps to grayscale
      // You can replace this with a more sophisticated mapping if needed
      point_rgb.r = 255;
      point_rgb.g = 255;
      point_rgb.b = 255;

      // Add the RGB point to the RGB point cloud
      cloudRGB->points[i] = point_rgb;
    }

    
    dbscan.init(groupA, cloudRGB, Config.dbscan.octreeResolution, Config.dbscan.eps, Config.dbscan.minPtsAux, Config.dbscan.minPts);
    dbscan.generateClusters_fast();

    this->publishClusters(dbscan.getClusters());

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Elapsed time: " << elapsed.count() * 1000 << "ms" << std::endl;
}

void Manager::publishClusters(std::vector<dbScanSpace::cluster> clusters){

  pcl::PointCloud<pcl::PointXYZ>::Ptr global_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  for (auto &cluster : clusters){
    for (auto &pointCluster : cluster.clusterPoints){
      pcl::PointXYZ point;
      point.x = pointCluster.x;
      point.y = pointCluster.y;
      point.z = pointCluster.z;

      global_cloud->points.push_back(point);
    }
  }

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*global_cloud, output);

  output.header.frame_id = "global";
  output.header.stamp = ros::Time::now();
  this->pubClusters.publish(output);
}

void Manager::publishGround(sensor_msgs::PointCloud2 msg){
    msg.header.frame_id = "global";
    msg.header.stamp = ros::Time::now();
    this->pubGround.publish(msg);
}