#ifndef RANSAC_HPP
#define RANSAC_HPP

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include "Modules/Config.hpp"

#include "sensor_msgs/PointCloud2.h"

#include <iostream>


class Ransac {
  private:
    /* -------------------------- Private Attributes -------------------------- */
    float minz_, maxz_;
    int max_iterations_;
    double dist_threshold_, plane_angle_;
    bool vis_outliers_;

    /* -------------------------- Private Methods --------------------------- */

  public:
      
    Ransac(Params Config){
      this->minz_ = Config.ransac.minz;
      this->maxz_ =  Config.ransac.maxz;
      this->max_iterations_ = Config.ransac.max_iterations;
      this->dist_threshold_ = Config.ransac.dist_threshold;
      this->plane_angle_ = Config.ransac.plane_angle;
      this->vis_outliers_ = Config.ransac.vis_outliers;
    }
    //Ransac(){}
    void removeGround(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, 
                      sensor_msgs::PointCloud2 &no_ground_msg);

};

#endif