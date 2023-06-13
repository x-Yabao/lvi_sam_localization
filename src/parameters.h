#pragma once

// 标准库
#include <vector>
#include <map>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

// 引入第三方库
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/crop_box.h> 
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <glog/logging.h>


// #define SHOW_RELOCATE             //if show the relocation result, please cancle comment
// #define NCLT                      //if use NCLT dataset, please cancle comment

/*************************** lidar parameters ***************************/

// Topics
extern std::string pointCloudTopic;
extern std::string imuTopic;
extern std::string odomTopic;

// Frames
extern std::string lidarFrame;
extern std::string baselinkFrame;
extern std::string odometryFrame;
extern std::string mapFrame;

// Sensor Settings
extern int N_SCAN;
extern int Horizon_SCAN;
extern float ang_res_y;
extern int downsampleRate;                           
extern float lidarMinRange;                          
extern float lidarMaxRange; 

// IMU Settings
extern float imuAccNoise;
extern float imuGyrNoise;
extern float imuAccBiasN;
extern float imuGyrBiasN;
extern float imuGravity;
extern float imuRPYWeight;

extern std::vector<double> extRotV;
extern std::vector<double> extRPYV;
extern std::vector<double> extTransV;
extern Eigen::Matrix3d extRot;
extern Eigen::Matrix3d extRPY;
extern Eigen::Vector3d extTrans;
extern Eigen::Quaterniond extQRPY;
        
// LOAM feature threshold
extern float edgeThreshold;
extern float surfThreshold;
extern int edgeFeatureMinValidNum;
extern int surfFeatureMinValidNum;

// voxel filter paprams
extern float odometrySurfLeafSize;
extern float mappingCornerLeafSize;
extern float mappingSurfLeafSize ;

// robot motion constraint
extern float z_tollerance;
extern float rotation_tollerance;

// CPU Params
extern int numberOfCores;
extern double mappingProcessInterval;

// Surrounding map
extern float surroundingkeyframeAddingDistThreshold;
extern float surroundingkeyframeAddingAngleThreshold;
extern float surroundingKeyframeDensity;
extern float surroundingKeyframeSearchRadius;

// Loop closure
extern bool  loopClosureEnableFlag;
extern float loopClosureFrequency;
extern int   surroundingKeyframeSize;
extern float historyKeyframeSearchRadius;
extern float historyKeyframeSearchTimeDiff;
extern int   historyKeyframeSearchNum;
extern float historyKeyframeFitnessScore;

// algorithm
extern int RELOCATION_USE_ICP;
extern int LIDAR_RELOCATION_USE_SC;


/*************************** camera parameters ***************************/

// topics
extern std::string image_topic;

// path
extern std::string VOC_PATH;
extern std::string BRIEF_PATTERN_FILE;

// image
extern int DEBUG_IMAGE;
extern int COL;
extern int ROW;

extern cv::Mat K_camera;
extern cv::Mat distCoef;

extern double C_TX_L;
extern double C_TY_L;
extern double C_TZ_L;
extern double C_RX_L;
extern double C_RY_L;
extern double C_RZ_L;

// extrinsic
extern std::vector<double> ticV;
extern std::vector<double> qicV;
extern Eigen::Vector3d tic;
extern Eigen::Matrix3d qic;


/*************************** function parameters ***************************/

extern std::string POSE_GRAPH_SAVE_PATH;
extern std::string loading_path;
extern std::string binarymapName;
extern int lidar_keyFrame_num;

extern std::string QUERY_PATH;
extern std::string QUERY_LIDAR_PATH;
extern std::string QUERY_IMAGE_PATH;

extern std::string DATASET_PATH;
extern int START_FRAME;
extern int END_FRAME;

extern int RELOCATE_METHOD;
extern std::string ENV_LIDAR_PATH;
extern float DENSE_DIS;
extern float DENSE_RATIO;
extern float OPEN_DIS;
extern float OPEN_RATIO;
extern int USE_TUNNING;


/*************************** function ***************************/

void readLidarParameters();
void readCameraParameters();
void readFunctionParameters();
void readParameters();


/*************************** 类型定义 ***************************/

typedef pcl::PointXYZI PointType;

struct PointXYZIR
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIR,  
                                   (float, x, x) (float, y, y) (float, z, z) 
                                   (float, intensity, intensity) (uint16_t, ring, ring)
)

struct PointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,
                                    (float, x, x) (float, y, y) (float, z, z) 
                                    (float, intensity, intensity) (uint16_t, ring, ring) (float, time, time)
)

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;                  
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     
} EIGEN_ALIGN16;                       

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                    (float, x, x) (float, y, y) (float, z, z)
                                    (float, intensity, intensity)
                                    (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                    (double, time, time))

typedef PointXYZIRPYT  PointTypePose;

// 环境密度：空旷/稀疏/密集
enum EnvironmentDensityType {OPEN, SPARSE, DENSE};
// 环境状态：未知/室内/室外
enum EnvironmentStatusType {UNKNOWN, INDOOR, OURDOOR};