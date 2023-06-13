#pragma once

#include "parameters.h"
#include "utility/utility.h"
#include "lvi_sam_location_ros/cloud_info.h"

const int queueLength = 2000;   // 存储一帧激光数据之间的IMU数据的队列长度

class ImageProjection
{
public:
    ImageProjection();
    void allocateMemory();
    void resetParameters();
    void imageHandler(const sensor_msgs::ImageConstPtr &imageMsg);
    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg);
    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg);
    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);
    bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);
    bool deskewInfo();
    void imuDeskewInfo();
    void odomDeskewInfo();
    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur);
    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur);
    PointType deskewPoint(PointType *point, double relTime);
    bool getSyncImage();
    void projectPointCloud();
    void cloudExtraction();
    void publishClouds();
    void judgeEnvironmentStatus();

private:
    ros::NodeHandle nh;

    std::mutex imageLock;
    std::mutex imuLock;
    std::mutex odoLock;

    ros::Publisher pubExtractedCloud;
    ros::Publisher pubLaserCloudInfo;

    ros::Subscriber subImage;
    std::deque<sensor_msgs::Image> imageQueue;          // 图像消息队列

    ros::Subscriber subImu;     
    std::deque<sensor_msgs::Imu> imuQueue;              // imu消息队列

    ros::Subscriber subOdom;
    std::deque<nav_msgs::Odometry> odomQueue;           // imu里程计消息队列

    ros::Subscriber subLaserCloud;
    std::deque<sensor_msgs::PointCloud2> cloudQueue;    // 存储订阅的点云

    sensor_msgs::PointCloud2 currentCloudMsg;           // 当前正在处理的点云，ros格式

    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    int imuPointerCur;                                  // 指向imu相关数据数组的最后一个数据
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;     // 当前正在处理的原始点云，PCL格式
    // pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
    pcl::PointCloud<PointType>::Ptr   fullCloud;        // 原始点云转换的完整点云，经过运动补偿
    pcl::PointCloud<PointType>::Ptr   extractedCloud;   // 去掉了无效点的点云

    int deskewFlag;                                     // 是否要去畸变
    cv::Mat rangeMat;

    bool odomDeskewFlag;                                // 是否可以用odom来做运动补偿
    float odomIncreX;                                   // 通过imu里程计得到的帧起始和结束在x方向上的相对运动
    float odomIncreY;                                   // 通过imu里程计得到的帧起始和结束在y方向上的相对运动
    float odomIncreZ;                                   // 通过imu里程计得到的帧起始和结束在z方向上的相对运动

    lvi_sam_location_ros::cloud_info cloudInfo;         // 点云信息
    double timeScanCur;                                 // 当前帧的起始时间
    double timeScanEnd;                                 // 当前帧的结束时间
    std_msgs::Header cloudHeader;      

    pcl::PointCloud<PointType>::Ptr judgeCloud;         // 最终判断环境密度的点云(yabao)
    std::string envDensity;                             // 环境密度(yabao)
    ros::Publisher pubEnvDensityInfo;
    ros::Publisher pubEnvDensityCloud;
};