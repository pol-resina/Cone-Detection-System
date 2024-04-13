#include "Modules/Ransac.hpp"
#include <ros/ros.h>  //llibreria de ros!

void Ransac::removeGround(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, sensor_msgs::PointCloud2 &no_ground_msg){
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr preprocCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PassThrough<pcl::PointXYZI> pass;

    pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(this->minz_, this->maxz_);
    pass.filter(*preprocCloud);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    Eigen::Vector3f axis = Eigen::Vector3f(0.0, 0.0, 1.0);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(this->max_iterations_);
    seg.setDistanceThreshold(this->dist_threshold_);
    seg.setAxis(axis);
    seg.setEpsAngle(this->plane_angle_);

    seg.setInputCloud(preprocCloud);
    seg.segment(*inliers, *coefficients);

    if(inliers->indices.size()!=0){
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(preprocCloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*no_ground);
    }

    pcl::toROSMsg(*no_ground,no_ground_msg);
}
