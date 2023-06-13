#pragma once

#include "parameters.h"
#include "utility/utility.h"
#include "lvi_sam_localization/cloud_info.h"

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
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <gtsam/nonlinear/ISAM2.h>

using namespace gtsam;

using symbol_shorthand::X;      // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V;      // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B;      // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::G;      // GPS pose

class mapOptimization
{
public:
    mapOptimization();
    void allocateMemory();
    void resetLIO();
    void laserCloudInfoHandler(const lvi_sam_localization::cloud_infoConstPtr& msgIn);
    void gpsHandler(const nav_msgs::Odometry::ConstPtr& gpsMsg);
    void pointAssociateToMap(PointType const * const pi, PointType * const po);
    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn);
    gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint);
    gtsam::Pose3 trans2gtsamPose(float transformIn[]);
    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint);
    Eigen::Affine3f trans2Affine3f(float transformIn[]);
    PointTypePose trans2PointTypePose(float transformIn[]);
    void updateInitialGuess();
    void extractForLoopClosure();
    void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract);
    void extractSurroundingKeyFrames();
    void downsampleCurrentScan();
    void updatePointAssociateToMap();
    void cornerOptimization();
    void surfOptimization();
    void combineOptimizationCoeffs();
    bool LMOptimization(int iterCount);
    void scan2MapOptimization();
    void transformUpdate();
    float constraintTransformation(float value, float limit);
    bool saveFrame();
    void addOdomFactor();
    void saveKeyFramesAndFactor();
    void updatePath(const PointTypePose& pose_in);
    void publishOdometry();
    void publishFrames();
    void cloudGlobalLoad();
    void globalLocalizeThread();
    void ICPLocalizeInitialize();
    void ICPscanMatchGlobal();
    void keyFramesLoad();
    void initialpose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg);

public:
    // gtsam
    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2 *isam;
    Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance;

    ros::NodeHandle nh;

    ros::Publisher pubLaserCloudSurround;
    ros::Publisher pubOdomAftMappedROS;
    ros::Publisher pubKeyPoses;
    ros::Publisher pubPath;

    ros::Publisher pubHistoryKeyFrames;
    ros::Publisher pubIcpKeyFrames;
    ros::Publisher pubRecentKeyFrames;
    ros::Publisher pubRecentKeyFrame;
    ros::Publisher pubCloudRegisteredRaw;

    ros::Subscriber subLaserCloudInfo;
    ros::Subscriber subGPS;

    std::deque<nav_msgs::Odometry> gpsQueue;
    lvi_sam_localization::cloud_info cloudInfo;

    vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
    vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;

    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;        //gc: can be used to illustrate the path of odometry // keep
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;    //gc: can be used to illustrate the path of odometry //keep
    
    //addded**********************************by gc
    std::mutex mtxWin;
    std::vector<PointType> win_cloudKeyPoses3D;         // 里程计地图窗口
    std::vector<PointTypePose> win_cloudKeyPoses6D;

    std::vector<pcl::PointCloud<PointType>::Ptr> win_cornerCloudKeyFrames;
    std::vector<pcl::PointCloud<PointType>::Ptr> win_surfCloudKeyFrames;
    //added***********************************by gc

    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;   // corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;     // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS; // downsampled corner featuer set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS;   // downsampled surf featuer set from odoOptimization

    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    std::vector<PointType> laserCloudOriCornerVec;          // corner point holder for parallel computation
    std::vector<PointType> coeffSelCornerVec;
    std::vector<bool> laserCloudOriCornerFlag;
    std::vector<PointType> laserCloudOriSurfVec;            // surf point holder for parallel computation
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

    //pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
    //pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

    pcl::PointCloud<PointType>::Ptr latestKeyFrameCloud;
    pcl::PointCloud<PointType>::Ptr nearHistoryKeyFrameCloud;

    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization

    ros::Time timeLaserInfoStamp;
    double timeLaserCloudInfoLast;

    float transformTobeMapped[6];

    std::mutex mtx;

    double timeLastProcessing = -1;

    bool isDegenerate = false;
    Eigen::Matrix<float, 6, 6> matP;

    int winSize = 30;
    int laserCloudCornerFromMapDSNum = 0;
    int laserCloudSurfFromMapDSNum = 0;
    int laserCloudCornerLastDSNum = 0;
    int laserCloudSurfLastDSNum = 0;

    bool aLoopIsClosed = false;
    int imuPreintegrationResetId = 0;

    nav_msgs::Path globalPath;

    Eigen::Affine3f transPointAssociateToMap;

    /*************added by gc*****************/
    pcl::PointCloud<PointType>::Ptr cloudGlobalMap;             // 全局地图
    pcl::PointCloud<PointType>::Ptr cloudGlobalMapDS;           // 全局地图降采样
    pcl::PointCloud<PointType>::Ptr cloudScanForInitialize;

    ros::Subscriber subIniPoseFromRviz;
    ros::Publisher pubLaserCloudInWorld;
    ros::Publisher pubMapWorld;
    //ros::Publisher fortest_publasercloudINWorld;

    float transformInTheWorld[6];   // the pose in the world（the prebuilt map）
    float tranformOdomToWorld[6];
    int globalLocaSkipFrames = 3;
    int frameNum = 1;
    tf::TransformBroadcaster tfOdom2Map;
    std::mutex mtxtranformOdomToWorld;
    std::mutex mtx_general;
    bool globalLocalizeInitialiized = false;

    ros::Subscriber subImu;

    enum InitializedFlag
    {
        NonInitialized,
        Initializing,
        Initialized
    };
    
    InitializedFlag initializedFlag;

    geometry_msgs::PoseStamped poseOdomToMap;
    ros::Publisher pubOdomToMapPose;
    /*************added by gc******************/
};