#ifndef STATE_HPP
#define STATE_HPP

// #include "Utils/Common.hpp"
// #include "Modules/Config.hpp"
// #include "Utils/Utils.hpp"

#include "Objects/RotTransl.hpp"
#include "Objects/Point.hpp"
#include "Objects/IMU.hpp"

#include <geometry_msgs/PoseStamped.h>

// #include "Modules/Accumulator.hpp"

// #include <deque>
// #include <Eigen/Core>
// #include <Eigen/Geometry>  // For Quaternion

class State {
    public:
        // State
        Eigen::Matrix3f R;
        Eigen::Vector3f pos;
        Eigen::Vector3f vel;
        Eigen::Vector3f bw;
        Eigen::Vector3f ba;
        Eigen::Vector3f g;

        // Offsets
        Eigen::Matrix3f RLI;
        Eigen::Vector3f tLI;

        // Last controls
        TimeType time;
        Eigen::Vector3f a;
        Eigen::Vector3f w;

        // Noises
        Eigen::Vector3f nw;
        Eigen::Vector3f na;
        Eigen::Vector3f nbw;
        Eigen::Vector3f nba;

        State();
        State(const state_msg& msg);
        State(double time);
        // State(const state_ikfom& s, double time);            state ikFoM, no cal, es part del kalman

        RotTransl I_Rt_L() const;
        RotTransl inv() const;

        void operator+= (const IMU& imu);
        friend Point operator* (const State& X, const Point& p);
        friend RotTransl operator* (const State& X, const RotTransl& RT);
        friend Points operator* (const State& X, const Points& points);
    private:
        // When propagating, we set noises = 0
        void propagate_f(IMU imu, float dt);
        void update(IMU imu);  
};

#endif