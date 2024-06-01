#ifndef COMPENSATOR_HPP
#define COMPENSATOR_HPP

#include "Modules/Config.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <iostream>


class Compensator {
  private:
    /* -------------------------- Private Attributes -------------------------- */

    /* -------------------------- Private Methods --------------------------- */

  public:
    //Constructor
    Compensator(Params Config){
    }

    /* -------------------------- Public Methods --------------------------- */
    void compensate(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);

};

#endif