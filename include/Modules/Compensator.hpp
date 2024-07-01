#ifndef COMPENSATOR_HPP
#define COMPENSATOR_HPP

#include "Modules/Config.hpp"
#include "Utils/Common.hpp"
#include "Modules/Accumulator.hpp"

#include "Modules/PointCloudProcessor.hpp"
#include "Objects/State.hpp"
#include "Objects/Point.hpp"
#include "Objects/IMU.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

#include <iostream>
#include <cassert>


class Compensator {
  private:
    /* -------------------------- Private Methods --------------------------- */
      State get_t2(const States&, double t2);
      States upsample(const States&, const IMUs&);

      Points voxelgrid_downsample(const Points&);
      // pcl::PointCloud<full_info::Point> voxelgrid_downsample_PCL(const Points&);
      // Points onion_downsample(const Points&);

  public:
    Compensator() {};

    // Main constructor
    Points compensate(double t1, double t2);
    Points compensate(const States& states, const State& Xt2, const Points& points);
    
    States path(double t1, double t2);
    Points downsample(const Points&);
    // pcl::PointCloud<full_info::Point> downsample_PCL(const Points&);

};

#endif