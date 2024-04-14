#include "Modules/Manager.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h> // for pcl::copyPointCloud
#include <random>

extern struct Params Config;

float colors[] = {
    255, 0,   0,    // red 		1
    0,   255, 0,    // green		2
    0,   0,   255,  // blue		3
    255, 255, 0,    // yellow		4
    0,   255, 255,  // light blue	5
    255, 0,   255,  // magenta     6
    255, 255, 255,  // white		7
    255, 128, 0,    // orange		8
    255, 153, 255,  // pink		9
    51,  153, 255,  //			10
    153, 102, 51,   //			11
    128, 51,  153,  //			12
    153, 153, 51,   //			13
    163, 38,  51,   //			14
    204, 153, 102,  //		15
    204, 224, 255,  //		16
    128, 179, 255,  //		17
    206, 255, 0,    //			18
    255, 204, 204,  //			19
    204, 255, 153,  //			20

};  // 20x3=60 color elements


Manager::Manager(ros::NodeHandle &nh): ransac(Config){
    pubGround = nh.advertise<sensor_msgs::PointCloud2>(Config.common.topics.output.ground, 20);
    pubClusters = nh.advertise<sensor_msgs::PointCloud2>(Config.common.topics.output.clusters, 20);
    this->publish_debug_ = Config.manager.publish_debug;
}

void Manager::velodyneCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    if (cloud_msg == nullptr) return; // NO data

    auto start = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    sensor_msgs::PointCloud2 msg;
    ransac.removeGround(cloud, msg);

    if (publish_debug_) {
      this->publishGround(msg);
    }
    
    /*
    std::vector<htr::Point3D> groupA;
    dbScanSpace::dbscan dbscan;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloudRGB->width = cloud->width;
    cloudRGB->height = cloud->height;
    cloudRGB->is_dense = cloud->is_dense;
    cloudRGB->points.resize(cloudRGB->width * cloudRGB->height);

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
      pcl::PointXYZI point_i = cloud->points[i];

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

    this->publishClusters(dbscan.getClusters());
    */

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Elapsed time: " << elapsed.count() * 1000 << "ms" << std::endl;
}

void Manager::publishClusters(std::vector<dbScanSpace::cluster> clusters){

  std::random_device seeder;
  std::ranlux48 gen(seeder());
  std::uniform_int_distribution<int> uniform_0_255(0, 255);

  int j = 0;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  for (auto &cluster : clusters){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    uint8_t r;
    uint8_t g;
    uint8_t b;

    if (j < 60){
      r = (uint8_t)colors[j];
      g = (uint8_t)colors[j+1];
      b = (uint8_t)colors[j+2];
    }
    else{
      r = (uint8_t)uniform_0_255(gen);
      g = (uint8_t)uniform_0_255(gen);
      b = (uint8_t)uniform_0_255(gen);
    }

    for (auto &pointCluster : cluster.clusterPoints){
      pcl::PointXYZRGB point;
      point.x = pointCluster.x;
      point.y = pointCluster.y;
      point.z = pointCluster.z;

      uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
      point.rgb = *reinterpret_cast<float *>(&rgb);

      cluster_rgb->points.push_back(point);
      global_cloud->points.push_back(point); // Add point to global cloud
    }

    j += 3;
  }

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*global_cloud, output);
  output.header.frame_id = "global";
  output.header.stamp = ros::Time::now();
  this->pubGround.publish(output);
}

void Manager::publishGround(sensor_msgs::PointCloud2 msg){
    msg.header.frame_id = "global";
    msg.header.stamp = ros::Time::now();
    this->pubGround.publish(msg);
}