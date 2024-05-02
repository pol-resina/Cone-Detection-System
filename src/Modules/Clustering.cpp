#include "Modules/Clustering.hpp"

bool Clustering::dbscan(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, 
            std::vector<std::vector<int>> &cluster_index){

  if (!cloud->size()) return false;
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud(cloud);
  std::vector<bool> cloud_processed(cloud->size(), false);

  for (size_t i = 0; i < cloud->points.size(); i++){
    if (cloud_processed[i]) continue;

    std::vector<int> seed_queue;

    std::vector<int> indices_cloud;
    std::vector<float> dists_cloud;
    if (kdtree.radiusSearch(cloud->points[i], eps_, indices_cloud, dists_cloud) >= minPts_) {
      seed_queue.push_back(i);
      cloud_processed[i] = true;
      for (size_t j = 0; j < indices_cloud.size(); j++){
        if (cloud_processed[indices_cloud[j]]){
          continue;
        }
        else{
          seed_queue.push_back(indices_cloud[j]);
          cloud_processed[indices_cloud[j]] = true;
        }
      }
    }
    else continue;

    int seed_index = 1;
    while (seed_index < seed_queue.size()){
      std::vector<int> indices;
      std::vector<float> dists;
      if (kdtree.radiusSearch(cloud->points[seed_queue[seed_index]], eps_, indices, dists) < minPts_){
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

      if (seed_queue.size() > maxPoints_) break; //this condition improves the time
    }
    cluster_index.push_back(seed_queue);    
  }

  if (cluster_index.size()) return true;
  else return false;
}

