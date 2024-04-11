#ifndef PARAMS_HPP
#define PARAMS_HPP

#include <ros/ros.h>

#include <string>
#include <vector>

struct Params {

    Params(const ros::NodeHandle &nh);

    struct Common {
        struct {
            struct {
                std::string points;
            } input;
            struct {
                std::string ground;
            } output;
        } topics;
    } common;

    struct Manager {
        bool publish_debug;
    } manager;
};

#endif // PARAMS_HPP
