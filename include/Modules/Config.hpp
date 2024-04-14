#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <string>
#include <vector>

struct Params{

    struct Ransac {
        bool ground_segmentation_mode;
        bool publish_ground;
        bool publish_obstacle;
        int minz;
        int maxz;
        int max_iterations;
        double dist_threshold;
        double plane_angle; 
        bool vis_outliers;
    } ransac;

    struct Dbscan {
        int octreeResolution;
        int eps;
        int minPtsAux;
        int minPts;
    } dbscan;


    struct Common {
        struct {
            struct {
                std::string points;
            } input;
            struct {
                std::string ground;
                std::string clusters;
            } output;
        } topics;
    } common;

    struct Manager {
        bool publish_debug;
    } manager;
};

#endif