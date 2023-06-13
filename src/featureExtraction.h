#pragma once

#include "parameters.h"
#include "utility/utility.h"
#include "lvi_sam_localization/cloud_info.h"

struct smoothness_t{ 
    float value;
    size_t ind;
};

struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};

class FeatureExtraction
{
public:
    FeatureExtraction();
    void initializationValue(); 
    void laserCloudInfoHandler(const lvi_sam_localization::cloud_infoConstPtr& msgIn);
    void calculateSmoothness();
    void markOccludedPoints();
    void extractFeatures();
    void freeCloudInfoMemory();
    void publishFeatureCloud();

private:
    ros::NodeHandle nh;

    ros::Subscriber subLaserCloudInfo;

    ros::Publisher pubLaserCloudInfo;
    ros::Publisher pubCornerPoints;
    ros::Publisher pubSurfacePoints;

    pcl::PointCloud<PointType>::Ptr extractedCloud;     // 上一节点传来的去畸变的点云
    pcl::PointCloud<PointType>::Ptr cornerCloud;        // 角点点云
    pcl::PointCloud<PointType>::Ptr surfaceCloud;       // 面点点云

    pcl::VoxelGrid<PointType> downSizeFilter;           // 对面点进行下采样，因为面点很多

    lvi_sam_localization::cloud_info cloudInfo;                      // 点云信息
    std_msgs::Header cloudHeader;                       // ROS消息头

    std::vector<smoothness_t> cloudSmoothness;          // 记录每一个点的曲率和原索引
    float *cloudCurvature;                              // 记录每个点的曲率
    int *cloudNeighborPicked;                           // 0为有效点，1为无效点
    int *cloudLabel;                                    // 标签为1为角点，-1表示面点

};