#pragma once

#include "ThirdParty/lidar/Scancontext.h"
#include "ThirdParty/camera/DBoW/DBoW2.h"
#include "ThirdParty/camera/DVision/DVision.h"
#include "utility/utility.h"
#include "utility/tictoc.h"
#include "parameters.h"
#include "keyframe.h"
#include "serialize.h"

/*
typedef pcl::PointXYZI PointType;

//A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;                  // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
} EIGEN_ALIGN16;                        // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

typedef PointXYZIRPYT  PointTypePose;
*/


class MultiMap
{
public:
    MultiMap();
    void allocateMemory();

    // 地图加载
    int loadMultiMap();          
    int loadLidarMap();
    int loadVisualMap();

    int testMap();
    int buildNCLTMap();
    
private:
    int loadCloudKeyFrames();   // 加载点云关键帧
    int loadCornerAndSurf();    // 加载角点和平面点
    int loadSCbyScans();        // 加载Scancontext描述子
    int loadTrajectory();       // 加载轨迹
    int loadTransformations();  // 加载六自由度位姿

    int loadPoseGraph();
    void loadKeyFrame(KeyFrame* cur_kf);
	void loadVocabulary(std::string voc_path);

private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version);

private:
    friend class mapOptimization;
    /************************ lidar information ****************************/
    std::vector<pcl::PointCloud<PointType>::Ptr> cloudKeyFrames;
    std::vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;     
    std::vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;

    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;

    SCManager scManager;

    /************************ camera information ****************************/
	std::list<KeyFrame*> keyframelist;	// 关键帧地图
	std::mutex m_keyframelist;

	int global_index;					// 关键帧索引

	BriefDatabase db;					// 地图词袋
	BriefVocabulary* voc;				// 字典

};


