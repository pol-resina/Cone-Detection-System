#ifndef CLUSTERING_HPP
#define CLUSTERING_HPP

#include "Modules/Config.hpp"
#include "Utils/Dbscan.hpp"

#include <vector>

class Clustering{
    private:
        /* -------------------------- Private Attributes -------------------------- */
        int octreeResolution_;
        int eps_;
        int minPtsAux_;
        int minPts_;
        /* -------------------------- Private Methods --------------------------- */

    public:
        Clustering(Params Config){
            this->octreeResolution_ = Config.dbscan.octreeResolution;
            this->eps_ = Config.dbscan.eps;
            this->minPtsAux_ = Config.dbscan.minPtsAux;
            this->minPts_ = Config.dbscan.minPts;
        }

        std::vector<dbScanSpace::cluster> clusterize_dbScan(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud);

};
#endif // CLUSTERING_HPP