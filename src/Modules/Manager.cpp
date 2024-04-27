#include "Modules/Manager.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h> 
#include <random>
#include <string>

extern struct Params Config;

Manager::Manager(ros::NodeHandle &nh): ransac(Config), clustering(Config){
  pubGround = nh.advertise<sensor_msgs::PointCloud2>(Config.common.topics.output.ground, 20);
  pubClusters = nh.advertise<sensor_msgs::PointCloud2>(Config.common.topics.output.clusters, 20);
  pubObs = nh.advertise<as_msgs::ObservationArray>(Config.common.topics.output.observations, 20);
  this->publish_debug_ = Config.manager.publish_debug;
}

void Manager::velodyneCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    this->header_ = cloud_msg->header;

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
    
    // generate clusters
    auto clusters = clustering.generateClusters(no_ground, false);
    // publish clusters
    this->publishClusters(clusters);
    //publish observations
    this->publishObservations(clusters);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Elapsed time: " << elapsed.count() * 1000 << "ms" << std::endl;
}

void Manager::publishClusters(const std::vector<dbScanSpace::cluster> &clusters){

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  std::cout << "size: " << clusters.size() << std::endl;
  for (auto &cluster : clusters){
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 255);

    int random_number1 = dis(gen);
    int random_number2 = dis(gen);
    int random_number3 = dis(gen);
    for (auto &pointCluster : cluster.clusterPoints){
      pcl::PointXYZRGB point;
      point.x = pointCluster.x;
      point.y = pointCluster.y;
      point.z = pointCluster.z;

      point.r = random_number1;
      point.b = random_number2;
      point.g = random_number3;

      global_cloud->points.push_back(point);
    }
  }

  sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*global_cloud, *output);

  output->header.frame_id = "velodyne";
  output->header.stamp = ros::Time::now();
  this->pubClusters.publish(output);
}

void Manager::publishGround(sensor_msgs::PointCloud2 &msg){
  msg.header.frame_id = "velodyne";
  msg.header.stamp = ros::Time::now();
  this->pubGround.publish(msg);
}

void Manager::publishObservations(std::vector<dbScanSpace::cluster> &clusters){
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  as_msgs::ObservationArray obs_vector;
  for (dbScanSpace::cluster c: clusters){
    as_msgs::Observation obs;
    c.calculateCentroid();
    obs.centroid.x = c.centroid3D.x;
    obs.centroid.y = c.centroid3D.y;
    obs.centroid.z = c.centroid3D.z;
    for (pcl::mod_pointXYZ p: c.clusterPoints){
      pcl::PointXYZI point;
      point.x = p.x;
      point.y = p.y;
      point.z = p.z;
      cloud->points.push_back(point);
    }
    obs_vector.observations.push_back(obs);
    cloud->clear();
  }

  obs_vector.header = this->header_;
  obs_vector.header.stamp = ros::Time::now();
  this->pubObs.publish(obs_vector);
}
