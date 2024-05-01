#ifndef CLUSTERING_HPP
#define CLUSTERING_HPP

#include "Modules/Config.hpp"

#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>

class Clustering {
    private:
    // Private Atributes
    float eps_;
    int minPts_, maxPoints_;
    
    public:
    // Constructor
    Clustering(Params Config){
        this->eps_ = Config.dbscan.eps;
        this->minPts_ = Config.dbscan.minPts;
        this->maxPoints_ = Config.dbscan.maxPoints;
    }

    bool dbscan(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, 
                std::vector<std::vector<int>> &cluster_index);

};

#endif // CLUSTERING_HPP