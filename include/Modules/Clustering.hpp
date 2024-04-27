#ifndef CLUSTERING_HPP
#define CLUSTERING_HPP


#include "Utils/Dbscan.hpp"
#include "Modules/Config.hpp"

class Clustering {
    private:
    // Private Atributes
    float eps_;
    int minPts_, minPtsAux_, octreeResolution_;
    
    public:
    // Constructor
    Clustering(Params Config){
        this->eps_ = Config.dbscan.eps;
        this->minPts_ = Config.dbscan.minPts;
        this->minPtsAux_ = Config.dbscan.minPtsAux;
        this->octreeResolution_ = Config.dbscan.octreeResolution;
    }

    std::vector<dbScanSpace::cluster> generateClusters(const pcl::PointCloud<pcl::PointXYZI>::Ptr &no_ground, bool fast);
    
};

#endif // CLUSTERING_HPP