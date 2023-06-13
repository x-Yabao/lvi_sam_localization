#pragma once

#include "parameters.h"
#include "utility/utility.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

using gtsam::symbol_shorthand::X;   // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V;   // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B;   // Bias  (ax,ay,az,gx,gy,gz)

class TransformFusion
{
public:
    ros::NodeHandle nh;

    std::mutex mtx;

    ros::Subscriber subImuOdometry;                 // 订阅预积分节点的增量位姿
    ros::Subscriber subLaserOdometry;               // 订阅地图优化节点的全局位姿

    ros::Publisher pubImuOdometry;
    ros::Publisher pubImuPath;

    Eigen::Affine3f lidarOdomAffine;                // 从地图优化节点得到的全局位姿
    Eigen::Affine3f imuOdomAffineFront;
    Eigen::Affine3f imuOdomAffineBack;

    tf::TransformListener tfListener;
    tf::StampedTransform lidar2Baselink;

    double lidarOdomTime = -1;                      // 最新的lidar里程记时间戳
    deque<nav_msgs::Odometry> imuOdomQueue;

    TransformFusion();
    Eigen::Affine3f odom2affine(nav_msgs::Odometry odom);
    void lidarOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg);
    void imuOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg);
    
};

class IMUPreintegration
{
public:
    ros::NodeHandle nh;

    std::mutex mtx;

    ros::Subscriber subImu;             //订阅imu信息
    ros::Subscriber subOdometry;        //订阅lidar odometry信息（增量信息）
    ros::Publisher pubImuOdometry;      //发布imu odometry信息，用于imageProjection节点给点云提供初始化位姿

    bool systemInitialized = false;

    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;         // 位姿置信度
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;          // 速度置信度
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;         // 零偏置信度
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
    gtsam::Vector noiseModelBetweenBias;

    gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;

    std::deque<sensor_msgs::Imu> imuQueOpt;
    std::deque<sensor_msgs::Imu> imuQueImu;

    gtsam::Pose3 prevPose_;     
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;

    gtsam::NavState prevStateOdom;                  // 最新的经过优化的里程计状态
    gtsam::imuBias::ConstantBias prevBiasOdom;      // 最新的经过优化的零偏

    bool doneFirstOpt = false;
    double lastImuT_imu = -1;
    double lastImuT_opt = -1;

    gtsam::ISAM2 optimizer;                         // 优化器
    gtsam::NonlinearFactorGraph graphFactors;
    gtsam::Values graphValues;

    const double delta_t = 0;

    int key = 1;

    // 外参
    gtsam::Pose3 imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));     
    gtsam::Pose3 lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));

    IMUPreintegration(); 
    void resetOptimization();
    void resetParams();
    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg);
    bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur);
    void imuHandler(const sensor_msgs::Imu::ConstPtr& imu_raw);
    
};