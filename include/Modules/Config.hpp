#ifndef CONFIG_HPP
#define CONFIG_HPP

#include "Utils/Common.hpp"

#include <string>
#include <vector>

struct Params{

    struct Ransac {
        bool ground_segmentation_mode;
        bool publish_ground;
        bool publish_obstacle;
        int minz;
        int maxz;
        int max_x;
        int max_y;
        int min_y;
        double x_car_limit_p;
        double x_car_limit_n;
        double y_car_limit_p;
        double y_car_limit_n;
        int max_iterations;
        double dist_threshold;
        double plane_angle; 
        bool vis_outliers;
    } ransac;

    struct Dbscan {
        int eps;
        int minPts;
        int maxPts;
        struct Classification {
            double distX;
            double distY;
            double distZ;
        } classification;

    } dbscan;


    struct Common {
        struct {
            struct {
                std::string points;
            } input;
            struct {
                std::string ground;
                std::string clusters;
                std::string observations;
            } output;
        } topics;
    } common;

    struct Manager {
        bool publish_debug;
    } manager;

    struct State {
        std::vector<float> initial_gravity;
        std::vector<float> I_Rotation_L;
        std::vector<float> I_Translation_L;
    } state;

    struct Point {
        bool offset_beginning;
        double full_rotation_time;
    } point;

    struct Accumulator {
        double full_rotation_time;
        double real_time_delay;
        double imu_rate;
        InitializationParams Initialization;
        int MAX_POINTS2MATCH;
    } accumulator;

    struct Lidar {
        int downsample_rate;
        double min_dist;
        std::string LiDAR_type;
        bool stamp_beginning;
    }lidar;
};

#endif
