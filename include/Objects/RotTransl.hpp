#ifndef ROTTRANSL_HPP
#define ROTTRANSL_HPP

// #include "Utils/Common.hpp"
// #include "Modules/Config.hpp"
// #include "Utils/Utils.hpp"

#include "Objects/Point.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>  // For Quaternion

class RotTransl {
    public:
        Eigen::Matrix3f R;
        Eigen::Vector3f t;

        // RotTransl(const State& S);
        RotTransl(const Eigen::Matrix3f& dR, const Eigen::Vector3f& dt);
        RotTransl inv();

        friend RotTransl operator* (const RotTransl&, const RotTransl&);
        friend Point operator* (const RotTransl&, const Point& p);
        friend Points operator* (const RotTransl&, const Points&);
};

#endif