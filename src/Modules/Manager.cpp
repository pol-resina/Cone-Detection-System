#include "Modules/Manager.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h> 
#include <random>
#include <string>

extern struct Params Config;
bool clear_csv = false;


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

  if (!clear_csv) {
    std::ofstream file;
    file.open("/home/pol/Desktop/timeransac.csv", std::ofstream::out | std::ofstream::trunc);
    file.close();
  }
  auto end1 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed1 = end1 - start;
  this->saveTime("/home/pol/Desktop/timeransac.csv", elapsed1);

  if (publish_debug_) {
    this->publishGround(msg);
  }
  
  // generate clusters
  // First approach
  // auto clusters = clustering.generateClusters(no_ground, false);
  // Second approach
  std::vector<std::vector<int>> cluster_index;
  clustering.dbscan(no_ground, cluster_index, Config.dbscan.eps, Config.dbscan.minPts, Config.dbscan.maxPoints);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud_cluster->width = no_ground->width;
  cloud_cluster->height = no_ground->height;
  cloud_cluster->resize(no_ground->size());

  as_msgs::ObservationArray obs_vector;

  for (size_t i = 0; i < cluster_index.size(); i++){
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 255);

    int random_number1 = dis(gen);
    int random_number2 = dis(gen);
    int random_number3 = dis(gen);

    auto scx = 0.0;
    auto scy = 0.0;
    auto scz = 0.0;

    as_msgs::Observation obs;
    for (size_t j = 0; j < cluster_index[i].size(); j++){
      pcl::PointXYZRGB point;
      point.x = no_ground->points[cluster_index[i][j]].x;
      point.y = no_ground->points[cluster_index[i][j]].y;
      point.z = no_ground->points[cluster_index[i][j]].z;

      scx += point.x;
      scy += point.y;
      scz += point.z;

      point.r = random_number1;
      point.b = random_number2;
      point.g = random_number3;

      cloud_cluster->points[cluster_index[i][j]] = point;
    }
    obs.centroid.x = scx/cluster_index[i].size();
    obs.centroid.y = scy/cluster_index[i].size();
    obs.centroid.z = scz/cluster_index[i].size();

    obs_vector.observations.push_back(obs);
  }

  // publish clusters
  this->publishClusters(cloud_cluster);
  //publish observations
  this->publishObservations(obs_vector);

  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  std::cout << "Elapsed time: " << elapsed.count() * 1000 << "ms" << std::endl;
  
  if (!clear_csv) {
    std::ofstream file;
    file.open("/home/pol/Desktop/timeelapsed.csv", std::ofstream::out | std::ofstream::trunc);
    file.close();
    clear_csv = true;
  }
  this->saveTime("/home/pol/Desktop/timeelapsed.csv", elapsed);
}


void Manager::publishClusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &clusters){
  sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*clusters, *output);

  output->header.frame_id = "velodyne";
  output->header.stamp = ros::Time::now();
  this->pubClusters.publish(output);
}


void Manager::publishGround(sensor_msgs::PointCloud2 &msg){
  msg.header.frame_id = "velodyne";
  msg.header.stamp = ros::Time::now();
  this->pubGround.publish(msg);
}

void Manager::publishObservations(as_msgs::ObservationArray &obs_vector){
  obs_vector.header = this->header_;
  obs_vector.header.stamp = ros::Time::now();
  this->pubObs.publish(obs_vector);
}

void Manager::saveTime(const std::string &filename, std::chrono::duration<double> elapsed) {
  std::ofstream file;
  file.open(filename, std::ios::app);
  file << elapsed.count() * 1000 << std::endl;
  file.close();
}

// void Manager::publishClusters(const std::vector<dbScanSpace::cluster> &clusters){

//   pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

//   std::cout << "size: " << clusters.size() << std::endl;
//   for (auto &cluster : clusters){
//     std::random_device rd;
//     std::mt19937 gen(rd());
//     std::uniform_int_distribution<> dis(0, 255);

//     int random_number1 = dis(gen);
//     int random_number2 = dis(gen);
//     int random_number3 = dis(gen);
//     for (auto &pointCluster : cluster.clusterPoints){
//       pcl::PointXYZRGB point;
//       point.x = pointCluster.x;
//       point.y = pointCluster.y;
//       point.z = pointCluster.z;

//       point.r = random_number1;
//       point.b = random_number2;
//       point.g = random_number3;

//       global_cloud->points.push_back(point);
//     }
//   }

//   sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2);
//   pcl::toROSMsg(*global_cloud, *output);

//   output->header.frame_id = "velodyne";
//   output->header.stamp = ros::Time::now();
//   this->pubClusters.publish(output);
// }

// void Manager::publishObservations(std::vector<dbScanSpace::cluster> &clusters){
//   pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
//   as_msgs::ObservationArray obs_vector;
//   for (dbScanSpace::cluster c: clusters){
//     as_msgs::Observation obs;
//     c.calculateCentroid();
//     obs.centroid.x = c.centroid3D.x;
//     obs.centroid.y = c.centroid3D.y;
//     obs.centroid.z = c.centroid3D.z;
//     for (pcl::mod_pointXYZ p: c.clusterPoints){
//       pcl::PointXYZI point;
//       point.x = p.x;
//       point.y = p.y;
//       point.z = p.z;
//       cloud->points.push_back(point);
//     }
//     obs_vector.observations.push_back(obs);
//     cloud->clear();
//   }

//   obs_vector.header = this->header_;
//   obs_vector.header.stamp = ros::Time::now();
//   this->pubObs.publish(obs_vector);
// }