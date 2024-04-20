#include "Modules/Clustering.hpp"


std::vector<dbScanSpace::cluster> Clustering::clusterize_dbScan(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud){
    std::vector<htr::Point3D> groupA;
    dbScanSpace::dbscan dbscan;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloudRGB->width = input_cloud->width;
    cloudRGB->height = input_cloud->height;
    cloudRGB->is_dense = input_cloud->is_dense;
    cloudRGB->points.resize(cloudRGB->width * cloudRGB->height);

    for (size_t i = 0; i < input_cloud->points.size(); ++i)
    {
      pcl::PointXYZI point_i = input_cloud->points[i];

      pcl::PointXYZRGB point_rgb;
      point_rgb.x = point_i.x;
      point_rgb.y = point_i.y;
      point_rgb.z = point_i.z;

      point_rgb.r = 255;
      point_rgb.g = 255;
      point_rgb.b = 255;

      cloudRGB->points[i] = point_rgb;
    }

    dbscan.init(groupA, cloudRGB, this->octreeResolution_, this->eps_, this->minPtsAux_, this->minPts_);
    dbscan.generateClusters_fast();

    return dbscan.getClusters();
}