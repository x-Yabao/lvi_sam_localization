#pragma once

#include "parameters.h"
#include "utility/utility.h"
#include "lvi_sam_localization/cloud_info.h"
#include "MultiMap.h"
#include "keyframe.h"

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
    void initialpose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg);

    // 地图管理和服务接口(yabao)
    int loadFolderMap();
    int saveBinaryMap();
    int loadBinaryMap();

private:
    // 视觉重定位功能实现
    int visualRelocate();
    int loadQueryPicture(bool relocate_test);
    int detectLoop(KeyFrame* keyframe, int frame_index);
    KeyFrame* getKeyFrame(int index);

    // 激光重定位功能实现
    int lidarRelocate();
    int loadQueryCloud(bool relocate_test);
    void loopFindNearKeyframesByIndex(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum);
    void loopFindNearKeyframesByPose(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const PointType& pose, const int& searchNum);
    
    // 点云配准验证
    int refineRelocateResult();
    
    // 重定位初始化
    void relocateInitialize();

public:
    // gtsam
    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2 *isam;
    Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance;

    ros::NodeHandle nh;

    ros::Publisher pubKeyPoses;
    ros::Publisher pubLaserCloudSurround;
    ros::Publisher pubOdomAftMappedROS;
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

    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;        
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;    
    
    //addded by gc
    std::mutex mtxWin;
    std::vector<PointType> win_cloudKeyPoses3D;         // 里程计地图窗口
    std::vector<PointTypePose> win_cloudKeyPoses6D;

    std::vector<pcl::PointCloud<PointType>::Ptr> win_cornerCloudKeyFrames;
    std::vector<pcl::PointCloud<PointType>::Ptr> win_surfCloudKeyFrames;
    //added by gc

    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;   // corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;     // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS; // downsampled corner featuer set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS;   // downsampled surf featuer set from odoOptimization
    
    pcl::PointCloud<PointType>::Ptr laserCloudLast;         // 当前点云（yabao)
    pcl::PointCloud<PointType>::Ptr laserCloudLastDS;       // 当前点云下采样(yabao)
    cv::Mat currentPicture;                                 // 当前图片（yabao)
    bool imageAvailable;                                    // 图像是否可用（yabao)

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

    pcl::PointCloud<PointType>::Ptr latestKeyFrameCloud;        // added by gc
    pcl::PointCloud<PointType>::Ptr nearHistoryKeyFrameCloud;   // added by gc              

    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization

    ros::Time timeLaserInfoStamp;       // ros::time 类型的时间
    double timeLaserCloudInfoLast;      // double 类型的时间

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

    /* added by gc */
    pcl::PointCloud<PointType>::Ptr cloudGlobalMap;             // 全局地图
    pcl::PointCloud<PointType>::Ptr cloudGlobalMapDS;           // 全局地图降采样
    pcl::PointCloud<PointType>::Ptr cloudScanForInitialize;     // 用于初始化的点云

    ros::Subscriber subIniPoseFromRviz;
    ros::Publisher pubLaserCloudInWorld;                        // 发布当前点云在世界坐标系下的位置
    ros::Publisher pubMapWorld;                                 // 发布全局地图
    //ros::Publisher fortest_publasercloudINWorld;

    float transformInTheWorld[6];                               // the pose in the world（the prebuilt map）
    float tranformOdomToWorld[6];
    int frameNum = 1;
    tf::TransformBroadcaster tfOdom2Map;
    std::mutex mtxtranformOdomToWorld;
    std::mutex mtx_general;

    enum InitializedFlag
    {
        NonInitialized,
        Initializing,
        Initialized
    };
    
    InitializedFlag initializedFlag;

    geometry_msgs::PoseStamped poseOdomToMap;
    ros::Publisher pubOdomToMapPose;
    /* added by gc */

/************************** added by yabao *****************************/
public:
    MultiMap* map;                                  // 先验地图
    std::string binaryMapFile;

    /************重定位数据结构**************/
    pcl::PointCloud<PointType>::Ptr queryCloud;     // 重定位点云
    KeyFrame* queryPicture;					        // 重定位图片关键帧 

    // 重定位结果
    Eigen::Vector3d tmp_relocation_T;               // 粗重定位平移(lidar坐标系)
    Eigen::Matrix3d tmp_relocation_R;               // 粗重定位旋转(lidar坐标系)
    PointTypePose tmp_relocation_pose;              // 粗重定位位姿
    int relocation_lidar_index = -1;                // index为-1时根据位置提取局部地图，其他时根据index提取局部地图
    Eigen::Vector3d relocation_T;                   // 精重定位平移
    Eigen::Matrix3d relocation_R;                   // 精重定位旋转

    // 定位结果    
    Eigen::Vector3d location_T;                     // 最终定位位移(可通过getLocationResult()获得)
    Eigen::Matrix3d location_R;                     // 最终定位旋转
    Eigen::Affine3f lastTransformation;             // 最终定位结果（精重定位完成后会赋值）
    
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeLidarKeyPoses;
};