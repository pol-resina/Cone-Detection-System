#include "Modules/Manager.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/common/centroid.h> // compute3DCentroid
#include <random>
#include <string>

extern struct Params Config;
bool clear_csv = false;


Manager::Manager(ros::NodeHandle &nh): ransac(Config), clustering(Config), compensator(Config){
  pubGround = nh.advertise<sensor_msgs::PointCloud2>(Config.common.topics.output.ground, 20);
  pubClusters = nh.advertise<sensor_msgs::PointCloud2>(Config.common.topics.output.clusters, 20);
  pubObs = nh.advertise<as_msgs::ObservationArray>(Config.common.topics.output.observations, 20);
  this->publish_debug_ = Config.manager.publish_debug;
  pubPreRANSAC = nh.advertise<sensor_msgs::PointCloud2>("/ftfcd/debug/preRANSAC", 20);
  pubVelComp = nh.advertise<sensor_msgs::PointCloud2>("/AS/P/ftfcd/compensatedVelodyne", 20);

  this->accumulator = &Accumulator::getInstance();
}

void Manager::velodyneCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
  if (cloud_msg == nullptr || cloud_msg->data.empty()) return;

  auto start = std::chrono::high_resolution_clock::now(); // start timer
  this->header1_ = cloud_msg->header;



  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*cloud_msg, *cloud);

  pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground(new pcl::PointCloud<pcl::PointXYZI>);
  sensor_msgs::PointCloud2 msg;
  compensator.compensate(cloud);
  ransac.removeGround(cloud, msg, no_ground, pubPreRANSAC);

  // // save time ransac
  // if (!clear_csv) {
  //   std::ofstream file;
  //   file.open("/home/pol/Desktop/timeransac.csv", std::ofstream::out | std::ofstream::trunc);
  //   file.close();
  // }
  // auto end1 = std::chrono::high_resolution_clock::now();
  // std::chrono::duration<double> elapsed1 = end1 - start;
  // this->saveTime("/home/pol/Desktop/timeransac.csv", elapsed1);
  // // save time ransac

  // publish ground
  if (publish_debug_) {
    this->publishGround(msg);
  }
  
  // Clustering
  std::vector<std::vector<int>> cluster_index;
  clustering.dbscan(no_ground, cluster_index);

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

    as_msgs::Observation obs;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);

    float min_x, max_x, min_y, max_y, min_z, max_z;
    min_x = max_x = no_ground->points[cluster_index[i][0]].x;
    min_y = max_y = no_ground->points[cluster_index[i][0]].y;
    min_z = max_z = no_ground->points[cluster_index[i][0]].z;

    for (size_t j = 0; j < cluster_index[i].size(); j++){
      pcl::PointXYZRGB point;
      point.x = no_ground->points[cluster_index[i][j]].x;
      point.y = no_ground->points[cluster_index[i][j]].y;
      point.z = no_ground->points[cluster_index[i][j]].z;

      if (point.x < min_x) min_x = point.x;
      if (point.x > max_x) max_x = point.x;
      if (point.y < min_y) min_y = point.y;
      if (point.y > max_y) max_y = point.y;
      if (point.z < min_z) min_z = point.z;
      if (point.z > max_z) max_z = point.z;
      
      point.r = random_number1;
      point.b = random_number2;
      point.g = random_number3;

      cluster->points.push_back(point);
    }

    // CLASSIFICATION
    float dist_x = max_x - min_x;
    float dist_y = max_y - min_y;
    float dist_z = max_z - min_z;
    if (cluster->size() > Config.dbscan.maxPts) continue;
    if (dist_z > Config.dbscan.classification.distX) continue;
    if (dist_x > Config.dbscan.classification.distY) continue;
    if (dist_y > Config.dbscan.classification.distX) continue;
    // END CLASSIFICATION

    *cloud_cluster += *cluster;
    

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster, centroid);

    geometry_msgs::Point centroid_msg;
    centroid_msg.x = centroid[0];
    centroid_msg.y = centroid[1];
    centroid_msg.z = centroid[2];

    obs.centroid = centroid_msg;

    sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*cluster, *output);

    obs.cloud = *output;

    obs_vector.observations.push_back(obs);
  }

  // publish clusters
  this->publishClusters(cloud_cluster);
  //publish observations
  this->publishObservations(obs_vector);

  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  std::cout << "Elapsed time: " << elapsed.count() * 1000 << "ms" << std::endl;
  
  // if (!clear_csv) {
  //   std::ofstream file;
  //   file.open("/home/pol/Desktop/timeelapsed.csv", std::ofstream::out | std::ofstream::trunc);
  //   file.close();
  //   clear_csv = true;
  // }
  // this->saveTime("/home/pol/Desktop/timeelapsed.csv", elapsed);
}

void Manager::IMUCallback(const sensor_msgs::Imu::ConstPtr& imu_msg){
  // IMU imu(imu_msg);
  // this->BUFFER_I.push(imu);
  accumulator->receive_imu(imu_msg);
}

void Manager::limoveloCallback(const nav_msgs::Odometry::ConstPtr& odom_msg, const sensor_msgs::PointCloud2::ConstPtr& pcl_msg) {
    if (pcl_msg == nullptr || pcl_msg->data.empty()) return;

    this->header2_ = pcl_msg->header;

    geometry_msgs::Point odom_position = odom_msg->pose.pose.position;
    geometry_msgs::Quaternion odom_orientation = odom_msg->pose.pose.orientation;

    // Convert the geometry_msgs::Quaternion to Eigen::Quaternionf
    Eigen::Quaternionf quat(odom_orientation.w, odom_orientation.x, odom_orientation.y, odom_orientation.z);

    // Convert Eigen::Quaternionf to Eigen::Matrix3f
    Eigen::Matrix3f rotation_matrix = quat.toRotationMatrix();

    // Calculate the inverse of the rotation matrix (transpose for orthogonal matrices)
    Eigen::Matrix3f rotation_matrix_inv = rotation_matrix.transpose();

    // Create the translation vector
    Eigen::Vector3f translation_vector(odom_position.x, odom_position.y, odom_position.z);

    // Calculate the inverse translation
    Eigen::Vector3f translation_vector_inv = -rotation_matrix_inv * translation_vector;

    // Convert ROS PointCloud2 to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    pcl::fromROSMsg(*pcl_msg, pcl_cloud);

    // Create an Eigen 4x4 transformation matrix for the inverse transformation
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    // Set the inverse rotation part
    transform.block<3,3>(0,0) = rotation_matrix_inv;

    // Set the inverse translation part
    transform.block<3,1>(0,3) = translation_vector_inv;

    // Apply the inverse transformation to the point cloud
    pcl::PointCloud<pcl::PointXYZI> transformed_cloud;
    pcl::transformPointCloud(pcl_cloud, transformed_cloud, transform);

    // Convert the transformed PCL point cloud back to ROS PointCloud2
    sensor_msgs::PointCloud2 transformed_cloud_msg;
    pcl::toROSMsg(transformed_cloud, transformed_cloud_msg);
    transformed_cloud_msg.header = this->header2_; // Ensure the header is set correctly

    // Publish the transformed point cloud
    if (publish_debug_) {
        this->publishCompensatedVelodyne(transformed_cloud_msg);
    }
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
  msg.header.stamp = header1_.stamp;
  this->pubGround.publish(msg);
}

void Manager::publishCompensatedVelodyne(sensor_msgs::PointCloud2 &msg){
  msg.header.frame_id = "velodyne";
  msg.header.stamp = header2_.stamp;
  this->pubVelComp.publish(msg);
}

void Manager::publishObservations(as_msgs::ObservationArray &obs_vector){
  obs_vector.header = this->header1_;
  // obs_vector.header.stamp = ros::Time::now();
  this->pubObs.publish(obs_vector);
}

void Manager::saveTime(const std::string &filename, std::chrono::duration<double> elapsed) {
  std::ofstream file;
  file.open(filename, std::ios::app);
  file << elapsed.count() * 1000 << std::endl;
  file.close();
}