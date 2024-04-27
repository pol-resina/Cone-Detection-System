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