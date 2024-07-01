#include "Modules/Ransac.hpp"
#include <ros/ros.h> 

void Ransac::removeGround(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, sensor_msgs::PointCloud2 &no_ground_msg,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr &no_ground, ros::Publisher pubPreRANSAC){
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr preprocCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PassThrough<pcl::PointXYZI> pass;

    pcl::SACSegmentation<pcl::PointXYZI> seg;

    // Convert inputCloud to PointXYZ type
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, *inputCloud);
    
    pcl::CropBox<pcl::PointXYZ> cropBoxFilter;
    cropBoxFilter.setInputCloud(inputCloud);
    cropBoxFilter.setMin(Eigen::Vector4f(-0.5, -0.5, -0.5, 1.0));
    cropBoxFilter.setMax(Eigen::Vector4f(0.5, 0.5, 0.5, 1.0));
    cropBoxFilter.setNegative(true); // Set to negative mode to keep points outside the box
    cropBoxFilter.filter(*inputCloud);
    
    // pass.setInputCloud(cloud);
    // pass.setFilterFieldName("x");
    // pass.setFilterLimitsNegative(true);
    // pass.setFilterLimits(this->x_car_limit_n_, this->x_car_limit_p_);
    // pass.setFilterFieldName("y");
    // pass.setFilterLimits(this->y_car_limit_n_, this->y_car_limit_p_);
    // pass.filter(*cloud);

    // pass.setInputCloud(cloud);
    // pass.setFilterFieldName("y");
    // pass.setFilterLimits(this->y_car_limit_n_, this->y_car_limit_p_);
    // pass.filter(*cloud);
    
    // Convert filteredCloud back to PointXYZI type
    pcl::copyPointCloud(*inputCloud, *cloud);
    
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimitsNegative(false);
    pass.setFilterLimits(0, this->max_x_);
    pass.filter(*cloud);

    // pass.setInputCloud(cloud);
    // pass.setFilterFieldName("y");
    // pass.setFilterLimits(this->min_y_, this->max_y_);
    // pass.filter(*cloud);

    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(this->minz_, this->maxz_);
    pass.filter(*preprocCloud);
    std::cout << "preprocCloud size: " << preprocCloud->points.size() << std::endl;

    pcl::copyPointCloud(*preprocCloud, *inputCloud);
    int index = 0;
    cropBoxFilter.setInputCloud(inputCloud);
    cropBoxFilter.setNegative(true);
    while (index < inputCloud->points.size()){
        if(inputCloud->points[index].z > 0.5){
            int i;    // --> comentar la busqueda del nou index si es va mes rapid sense ell
            pcl::PointXYZ point_1; point_1.x = 0; point_1.y = 0; point_1.z = 0;
            for(i = 0; i < index; i++) if (std::abs(inputCloud->points[index - i - 1].x - inputCloud->points[index].x) > 1 || 
                                           std::abs(inputCloud->points[index - i - 1].y - inputCloud->points[index].y) > 1){
                point_1 = inputCloud->points[index - i - 1];
                break;
            }
            // pass.setFilterFieldName("x");
            // pass.setFilterLimits(preprocCloud->points[index].x - 0.2, preprocCloud->points[index].x + 0.2);
            // pass.filter(*preprocCloud);

            // pass.setFilterFieldName("y");
            // pass.setFilterLimits(preprocCloud->points[index].y - 0.2, preprocCloud->points[index].y + 0.2);
            // pass.filter(*preprocCloud);

            cropBoxFilter.setInputCloud(inputCloud);
            cropBoxFilter.setMin(Eigen::Vector4f(inputCloud->points[index].x - 1, inputCloud->points[index].y - 1, -4, 1.0));
            cropBoxFilter.setMax(Eigen::Vector4f(inputCloud->points[index].x + 1, inputCloud->points[index].y + 1, 4, 1.0));
            cropBoxFilter.filter(*inputCloud);

            index = index - i - 1;
            while((point_1.x != inputCloud->points[index].x ||
                   point_1.y != inputCloud->points[index].y ||
                   point_1.z != inputCloud->points[index].z) &&
                   index > 0) index--;
        } else index++;
    }
    pcl::copyPointCloud(*inputCloud, *preprocCloud);

    sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*preprocCloud,*output);
    output->header.frame_id = "velodyne";
    pubPreRANSAC.publish(output);

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

