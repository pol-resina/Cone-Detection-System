#include "Objects/Params.hpp"

Params::Params(const ros::NodeHandle &nh) {
    
    // COMMON
    nh.param<std::string>("/ftfcd/common/topics/input/points", common.topics.input.points, "/velodyne_points");
    nh.param<std::string>("/ftfcd/common/topics/output/ground", common.topics.output.ground, "/ftfcd/ground");

    // MANAGER
    nh.param<bool>("/ftfcd/manager/publish_debug", manager.publish_debug, true);
}