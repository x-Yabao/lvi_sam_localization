#include "parameters.h"


/*************************** lidar parameters ***************************/

// Topics
std::string pointCloudTopic;
std::string imuTopic;
std::string odomTopic;

// Frames
std::string lidarFrame;
std::string baselinkFrame;
std::string odometryFrame;
std::string mapFrame;

// Sensor Settings
int N_SCAN;
int Horizon_SCAN;
float ang_res_y;
int downsampleRate;                           
float lidarMinRange;                          
float lidarMaxRange; 

// IMU Settings
float imuAccNoise;
float imuGyrNoise;
float imuAccBiasN;
float imuGyrBiasN;
float imuGravity;
float imuRPYWeight;

std::vector<double> extRotV;
std::vector<double> extRPYV;
std::vector<double> extTransV;
Eigen::Matrix3d extRot;
Eigen::Matrix3d extRPY;
Eigen::Vector3d extTrans;
Eigen::Quaterniond extQRPY;
    
// LOAM feature threshold
float edgeThreshold;
float surfThreshold;
int edgeFeatureMinValidNum;
int surfFeatureMinValidNum;

// voxel filter paprams
float odometrySurfLeafSize;
float mappingCornerLeafSize;
float mappingSurfLeafSize ;

// CPU Params
int numberOfCores;
double mappingProcessInterval;

// Surrounding map
float surroundingKeyframeDensity;
float surroundingKeyframeSearchRadius;

// Loop closure
bool  loopClosureEnableFlag;
float loopClosureFrequency;
int   surroundingKeyframeSize;
float historyKeyframeSearchRadius;
float historyKeyframeSearchTimeDiff;
int   historyKeyframeSearchNum;
float historyKeyframeFitnessScore;

// algorithm
int RELOCATION_USE_ICP;
int LIDAR_RELOCATION_USE_SC;


/*************************** camera parameters ***************************/

// topics
std::string image_topic;

// path
std::string VOC_PATH;
std::string BRIEF_PATTERN_FILE;

// image
int DEBUG_IMAGE;
int COL;
int ROW;

cv::Mat K_camera;
cv::Mat distCoef;

double C_TX_L;
double C_TY_L;
double C_TZ_L;
double C_RX_L;
double C_RY_L;
double C_RZ_L;

// extrinsic
std::vector<double> ticV;
std::vector<double> qicV;
Eigen::Vector3d tic;
Eigen::Matrix3d qic;


/*************************** function parameters ***************************/

std::string POSE_GRAPH_SAVE_PATH;
std::string loading_path;
std::string binarymapName;
int lidar_keyFrame_num;

std::string QUERY_PATH;
std::string QUERY_LIDAR_PATH;
std::string QUERY_IMAGE_PATH;

std::string DATASET_PATH;
int START_FRAME;
int END_FRAME;

int RELOCATE_METHOD;
std::string ENV_LIDAR_PATH;
float DENSE_DIS;
float DENSE_RATIO;
float OPEN_DIS;
float OPEN_RATIO;
int USE_TUNNING;


/*************************** function ***************************/

void readLidarParameters()
{
    ros::param::get("pointCloudTopic", pointCloudTopic);
    ros::param::get("imuTopic", imuTopic);
    ros::param::get("odomTopic", odomTopic);

    ros::param::get("lidarFrame", lidarFrame);
    ros::param::get("baselinkFrame", baselinkFrame);
    ros::param::get("odometryFrame", odometryFrame);
    ros::param::get("mapFrame", mapFrame);

    ros::param::get("N_SCAN", N_SCAN);
    ros::param::get("Horizon_SCAN", Horizon_SCAN);
    ros::param::get("ang_res_y", ang_res_y);
    ros::param::get("downsampleRate", downsampleRate);
    ros::param::get("lidarMinRange", lidarMinRange);
    ros::param::get("lidarMaxRange", lidarMaxRange);

    ros::param::get("imuAccNoise", imuAccNoise);
    ros::param::get("imuGyrNoise", imuGyrNoise);
    ros::param::get("imuAccBiasN", imuAccBiasN);
    ros::param::get("imuGyrBiasN", imuGyrBiasN);
    ros::param::get("imuGravity", imuGravity);
    ros::param::get("imuRPYWeight", imuRPYWeight);

    ros::param::get("extrinsicRot", extRotV);
    ros::param::get("extrinsicRPY", extRPYV);
    ros::param::get("extrinsicTrans", extTransV);
    extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
    extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
    extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
    extQRPY = Eigen::Quaterniond(extRPY);

    ros::param::get("edgeThreshold", edgeThreshold);
    ros::param::get("surfThreshold", surfThreshold);
    ros::param::get("edgeFeatureMinValidNum", edgeFeatureMinValidNum);
    ros::param::get("surfFeatureMinValidNum", surfFeatureMinValidNum);

    ros::param::get("odometrySurfLeafSize", odometrySurfLeafSize);
    ros::param::get("mappingCornerLeafSize", mappingCornerLeafSize);
    ros::param::get("mappingSurfLeafSize", mappingSurfLeafSize);

    ros::param::get("numberOfCores", numberOfCores);
    ros::param::get("mappingProcessInterval", mappingProcessInterval);

    ros::param::get("surroundingKeyframeDensity", surroundingKeyframeDensity);
    ros::param::get("surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius);

    ros::param::get("loopClosureEnableFlag", loopClosureEnableFlag);
    ros::param::get("loopClosureFrequency", loopClosureFrequency);
    ros::param::get("surroundingKeyframeSize", surroundingKeyframeSize);
    ros::param::get("historyKeyframeSearchRadius", historyKeyframeSearchRadius);
    ros::param::get("historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff);
    ros::param::get("historyKeyframeSearchNum", historyKeyframeSearchNum);
    ros::param::get("historyKeyframeFitnessScore", historyKeyframeFitnessScore);

    ros::param::get("RELOCATION_USE_ICP", RELOCATION_USE_ICP);
    ros::param::get("LIDAR_RELOCATION_USE_SC", LIDAR_RELOCATION_USE_SC);

}

void readCameraParameters()
{
    ros::param::get("image_topic", image_topic);

    ros::param::get("VOC_PATH", VOC_PATH);
    ros::param::get("BRIEF_PATTERN_FILE", BRIEF_PATTERN_FILE);

    ros::param::get("DEBUG_IMAGE", DEBUG_IMAGE);
    ros::param::get("COL", COL);
    ros::param::get("ROW", ROW);


    cv::Mat _distCoef(4, 1, CV_32F);
    float _k1, _k2, _p1, _p2;
    ros::param::get("Camera_k1", _k1);
    ros::param::get("Camera_k2", _k2);
    ros::param::get("Camera_p1", _p1);
    ros::param::get("Camera_p2", _p2);
    _distCoef.at<float>(0) = _k1;
    _distCoef.at<float>(1) = _k2;
    _distCoef.at<float>(2) = _p1;
    _distCoef.at<float>(3) = _p2;
    _distCoef.copyTo(distCoef);

    cv::Mat _K = cv::Mat::eye(3, 3, CV_32F);
    float _fx, _fy, _cx, _cy;
    ros::param::get("Camera_fx", _fx);
    ros::param::get("Camera_fy", _fy);
    ros::param::get("Camera_cx", _cx);
    ros::param::get("Camera_cy", _cy);
    _K.at<float>(0, 0) = _fx;
    _K.at<float>(1, 1) = _fy;
    _K.at<float>(0, 2) = _cx;
    _K.at<float>(1, 2) = _cy;
    _K.copyTo(K_camera);


    ros::param::get("lidar_to_cam_tx", C_TX_L);
    ros::param::get("lidar_to_cam_ty", C_TY_L);
    ros::param::get("lidar_to_cam_tz", C_TZ_L);
    ros::param::get("lidar_to_cam_rx", C_RX_L);    // roll
    ros::param::get("lidar_to_cam_ry", C_RY_L);    // pitch
    ros::param::get("lidar_to_cam_rz", C_RZ_L);    // yaw

    ros::param::get("extrinsicTranslation", ticV);
    ros::param::get("extrinsicRotation", qicV);
    tic = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(ticV.data(), 3, 1);
    qic = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(qicV.data(), 3, 3);
    Eigen::Quaterniond Q(qic);
    qic = Q.normalized();

}

void readFunctionParameters()
{
    ros::param::get("POSE_GRAPH_SAVE_PATH", POSE_GRAPH_SAVE_PATH);
    ros::param::get("loading_path", loading_path);
    ros::param::get("binarymapName", binarymapName);
    ros::param::get("lidar_keyFrame_num", lidar_keyFrame_num);

    ros::param::get("QUERY_PATH", QUERY_PATH);
    ros::param::get("QUERY_LIDAR_PATH", QUERY_LIDAR_PATH);
    ros::param::get("QUERY_IMAGE_PATH", QUERY_IMAGE_PATH);

    ros::param::get("DATASET_PATH", DATASET_PATH);
    ros::param::get("START_FRAME", START_FRAME);
    ros::param::get("END_FRAME", END_FRAME);

    ros::param::get("RELOCATE_METHOD", RELOCATE_METHOD);
    ros::param::get("ENV_LIDAR_PATH", ENV_LIDAR_PATH);
    ros::param::get("DENSE_DIS", DENSE_DIS);
    ros::param::get("DENSE_RATIO", DENSE_RATIO);
    ros::param::get("OPEN_DIS", OPEN_DIS);
    ros::param::get("OPEN_RATIO", OPEN_RATIO);
    ros::param::get("USE_TUNNING", USE_TUNNING);

}

void readParameters()
{
    readLidarParameters();
    //std::cout << "Read lidar parameters finish."  << std::endl;
    readCameraParameters();
    //std::cout << "Read camera parameters finish."  << std::endl;
    readFunctionParameters();
    //std::cout << "Read function parameters finish."  << std::endl;
}
