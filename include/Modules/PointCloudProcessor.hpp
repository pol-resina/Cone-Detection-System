#ifndef POINTCLOUDPROCESSOR_HPP
#define POINTCLOUDPROCESSOR_HPP

// #include "Utils/Common.hpp"
// #include "Modules/Config.hpp"
// #include "Utils/Utils.hpp"

#include "Objects/Point.hpp"

class PointCloudProcessor {
    
    // Given a PointCloud, process it to push to the buffer

    public:
        PointCloudProcessor() = default;
        Points msg2points(const PointCloud_msg&);
        Points downsample(const Points&);
        Points sort_points(const Points&);
        void fill(pcl::PointCloud<full_info::Point>& pcl, const Points& points);

    private:
        // Velodyne specific
            Points velodynemsg2points(const PointCloud_msg&);
            double get_begin_time(const pcl::PointCloud<velodyne_ros::Point>&);
        
        // HESAI specific
            Points hesaimsg2points(const PointCloud_msg&);
            double get_begin_time(const pcl::PointCloud<hesai_ros::Point>&);
        
        // Ouster specific
            Points oustermsg2points(const PointCloud_msg&);
            double get_begin_time(const pcl::PointCloud<ouster_ros::Point>&);

        // Custom specific
            Points custommsg2points(const PointCloud_msg&);
            double get_begin_time(const pcl::PointCloud<custom::Point>&);
        
        template <typename PointType> Points to_points(const typename pcl::PointCloud<PointType>&);

        Points temporal_downsample(const Points&);
        static bool time_sort(const Point&, const Point&);
};

#endif