#include "Modules/Clustering.hpp"

std::vector<dbScanSpace::cluster> Clustering::generateClusters(const pcl::PointCloud<pcl::PointXYZI>::Ptr &no_ground, bool fast){
    
    dbScanSpace::dbscan dbscan;
    std::vector<htr::Point3D> groupA;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloudRGB->width = no_ground->width;
    cloudRGB->height = no_ground->height;
    cloudRGB->is_dense = no_ground->is_dense;
    cloudRGB->points.resize(cloudRGB->width * cloudRGB->height);

    for (size_t i = 0; i < no_ground->points.size(); ++i)
    {
      pcl::PointXYZI point_i = no_ground->points[i];

      pcl::PointXYZRGB point_rgb;
      point_rgb.x = point_i.x;
      point_rgb.y = point_i.y;
      point_rgb.z = point_i.z;

      point_rgb.r = 255;
      point_rgb.g = 255;
      point_rgb.b = 255;

      cloudRGB->points[i] = point_rgb;
    }


    dbscan.init(groupA, cloudRGB, octreeResolution_, eps_, minPtsAux_, minPts_);

    if (fast) dbscan.generateClusters_fast();
    else dbscan.generateClusters();

    return dbscan.getClusters();
}

bool Clustering::dbscan(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, 
            std::vector<std::vector<int>> &cluster_index, 
            const double &eps, const int &size, const int &maxPoints){

  if (!cloud->size()) return false;
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud(cloud);
  std::vector<bool> cloud_processed(cloud->size(), false);

  for (size_t i = 0; i < cloud->points.size(); i++){
    if (cloud_processed[i]) continue;

    std::vector<int> seed_queue;

    std::vector<int> indices_cloud;
    std::vector<float> dists_cloud;
    if (kdtree.radiusSearch(cloud->points[i], eps, indices_cloud, dists_cloud) >= size) {
      seed_queue.push_back(i);
      cloud_processed[i] = true;
    }
    else continue;

    int seed_index = 0;
    while (seed_index < seed_queue.size()){
      std::vector<int> indices;
      std::vector<float> dists;
      if (kdtree.radiusSearch(cloud->points[seed_queue[seed_index]], eps, indices, dists) < size){
        ++seed_index;
        continue;
      }
      for (size_t j = 0; j < indices.size(); j++){
        if (cloud_processed[indices[j]]){
          continue;
        }
        else{
          seed_queue.push_back(indices[j]);
          cloud_processed[indices[j]] = true;
        }
      }
      ++seed_index;

      if (seed_queue.size() > maxPoints) break;
    }
    if (seed_queue.size() <= maxPoints) cluster_index.push_back(seed_queue);    
  }

  if (cluster_index.size()) return true;
  else return false;
}