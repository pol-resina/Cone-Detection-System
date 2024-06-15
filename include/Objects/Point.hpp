#ifndef POINT_HPP
#define POINT_HPP

#include "Utils/Common.hpp"
#include "Modules/Config.hpp"
#include "Utils/Utils.hpp"

class Point {
    public:
        float x;
        float y;
        float z;
        TimeType time;
        float intensity;
        float range;

        Point();

        Point(const full_info::Point& p);
        Point(const Eigen::Matrix<float, 3, 1>& p);
        
        // Delegate constructor (Eigen + attributes)
        Point(const Eigen::Matrix<float, 3, 1>& p, const Point& attributes);
        
        // HESAI specific
            Point(const hesai_ros::Point& p);
            Point(const hesai_ros::Point& p, double time_offset);

        // Velodyne specific
            Point(const velodyne_ros::Point& p);
            Point(const velodyne_ros::Point& p, double time_offset);
        
        // Ouster specific
            Point(const ouster_ros::Point& p);
            Point(const ouster_ros::Point& p, double time_offset);

        // Custom specific
            Point(const custom::Point& p);
            Point(const custom::Point& p, double time_offset);

        full_info::Point toPCL() const;
        Eigen::Matrix<float, 3, 1> toEigen() const;

        float norm() const;
        Eigen::Vector3d cross(const Eigen::Vector3d& v);

        friend Point operator*(const Eigen::Matrix<float, 3, 3>&, const Point&);
        friend Point operator+(const Point& p, const Eigen::Matrix<float, 3, 1> v);
        friend Point operator-(const Point& p, const Eigen::Matrix<float, 3, 1> v);
        friend std::ostream& operator<< (std::ostream& out, const Point& p);

    private:
        template <typename PointType>
        void set_XYZ(const PointType& p);
        void set_XYZ(const Eigen::Matrix<float, 3, 1>& p);

        template <typename PointType>
        void set_attributes(const PointType& p);

        // Ouster specific
        void set_attributes(const ouster_ros::Point& p);
        
        // Point::set_attributes(const custom::Point& p);

        void pass_attributes(const Point& attributes);
};

#endif