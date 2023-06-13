#pragma once

#include "parameters.h"
#include "lvi_sam_localization/cloud_info.h"
#include "MultiMap.h"
#include "keyframe.h"

enum class LocationStatus { INITING, TRACKING_GOOD, TRACKING_BAD, LOST };

class MapOptimization
{
public:
    MapOptimization();
    void allocateMemory();

    // 定位和重定位接口
    void laserCloudInfoHandler(const lvi_sam_localization::cloud_infoConstPtr& msgIn);
    void relocate();
    int relocate_MODULE_TEST();
    void locate();
    void visualizeGlobalMapThread();

    // 地图管理和服务接口
    int loadFolderMap();
    int saveBinaryMap();
    int loadBinaryMap();
    void speedUpExtractSurroundingKeyFrames();

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

    // 定位功能实现
    void updateInitialGuess();
    void extractNearby();
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
    void publishOdometry();
    

    // tool
    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint);
    Eigen::Affine3f trans2Affine3f(float transformIn[]);
    PointTypePose trans2PointTypePose(float transformIn[]);
    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn);
    void pointAssociateToMap(PointType const * const pi, PointType * const po);

    void updateTrajectoryInfo();
    void updateParameters();            // 更新参数,用于实时参赛整定
    

public:
    MultiMap* map;                                  // 先验地图
    std::string binaryMapFile;

private:
    ros::NodeHandle nh;

    ros::Subscriber subCloud;
    ros::Publisher pubLaserOdometryGlobal;
    ros::Publisher pubLaserOdometryIncremental;     // 发布里程计（scan2map之后的，不包括回环）

    lvi_sam_localization::cloud_info cloudInfo;                     // 点云信息

    /************定位数据结构**************/
    // 用于优化过程
    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    // 用于优化过程
    std::vector<PointType> laserCloudOriCornerVec;                  // corner point holder for parallel computation
    std::vector<PointType> coeffSelCornerVec;
    std::vector<bool> laserCloudOriCornerFlag;
    std::vector<PointType> laserCloudOriSurfVec;                    // surf point holder for parallel computation
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;

    // 当前角点点云，面点点云和去畸变点云
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;           // corner feature set from odoOptimization，cloudinfo中提取的
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;             // surf feature set from odoOptimization，cloudinfo中提取的
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS;         // downsampled corner featuer set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS;           // downsampled surf featuer set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudLast;                 // 当前点云（yabao)
    pcl::PointCloud<PointType>::Ptr laserCloudLastDS;               // 当前点云下采样(yabao)
    cv::Mat currentPicture;                                         // 当前图片（yabao)
    bool imageAvailable;                                            // 图像是否可用（yabao) 

    // 局部地图
    std::map<int, std::pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> laserCloudMapContainer;  // 局部地图的一个容器
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;        // 角点局部地图
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;          // 面点局部地图
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;      // 角点局部地图的下采样后的点云
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;        // 面点局部地图的下采样后的点云

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;           // 角点局部地图的kdtree
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;             // 面点局部地图的kdtree

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeLidarKeyPoses;

    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses;    // for surrounding key poses of scan-to-map optimization
    
    ros::Time timeLaserInfoStamp;           // ros::time 类型的时间
    double timeLaserInfoCur;                // double 类型的时间
    double timeLastProcessing = -1;         // 上一帧的时间

    float transformTobeMapped[6];           // 估计的位姿

    std::mutex mtx;

    bool isDegenerate = false;
    cv::Mat matP;

    int laserCloudCornerFromMapDSNum = 0;   // 当前局部地图下采样后的角点数目
    int laserCloudSurfFromMapDSNum = 0;     // 当前局部地图下采样后的面点数目
    int laserCloudCornerLastDSNum = 0;      // 当前帧下采样后的角点的数目
    int laserCloudSurfLastDSNum = 0;        // 当前帧下采样后的面点数目

    Eigen::Affine3f transPointAssociateToMap;
    Eigen::Affine3f incrementalOdometryAffineFront; // scan matching前后位姿，用于计算增量里程计
    Eigen::Affine3f incrementalOdometryAffineBack;

    /************重定位数据结构**************/
    pcl::PointCloud<PointType>::Ptr queryCloud;     // 重定位点云
    KeyFrame* queryPicture;					        // 重定位图片关键帧

    /************处理结果**************/
    // 重定位结果
    bool just_relocate;                             // 是否刚进行完重定位
    int relocate_cnt;                               // 重定位次数
    int relocate_success_cnt;                       // 重定位成功次数

    Eigen::Vector3d tmp_relocation_T;               // 重定位粗位姿平移(lidar坐标系)
    Eigen::Matrix3d tmp_relocation_R;               // 重定位粗位姿旋转(lidar坐标系)
    PointTypePose tmp_relocation_pose;              // 重定位粗位姿
    int relocation_lidar_index;                     // index为-1时根据位置提取局部地图，其他时根据index提取局部地图
    Eigen::Vector3d relocation_T;                   // 重定位精位姿平移(可通过getRelocationResult()获得)
    Eigen::Matrix3d relocation_R;                   // 重定位精位姿旋转

    // 定位结果    
    // PointTypePose tmp_pose;                      // 估计位姿，用于提取局部地图(用transformTobeMapped代替)
    Eigen::Vector3d location_T;                     // 最终定位位移(可通过getLocationResult()获得)
    Eigen::Matrix3d location_R;                     // 最终定位旋转
    Eigen::Affine3f lastTransformation;             // 最终定位结果（精重定位完成后会赋值）
    
    Eigen::Affine3f transIncreConstSpeed;           // 匀速运动模型相关  
    double time_diff;                               // 匀速运动模型对应时间     


    // 系统状态量
    double last_score;                              // 上一帧配准得分
    double last_process_time;                       // 上一帧处理时间(s)
    LocationStatus status;                          // 定位系统状态

    /**************************轨迹**************************/
    std::vector<double> traj_timestamps;
    std::vector<double> traj_scores;
    std::vector<double> traj_process_times;
    std::vector<LocationStatus> traj_status;
    std::vector<Eigen::Vector3d> traj_Ts;
    std::vector<Eigen::Matrix3d> traj_Rs;
   

};