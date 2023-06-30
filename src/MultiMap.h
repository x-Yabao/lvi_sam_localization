#pragma once

#include "ThirdParty/lidar/Scancontext.h"
#include "ThirdParty/camera/DBoW/DBoW2.h"
#include "ThirdParty/camera/DVision/DVision.h"
#include "utility/utility.h"
#include "utility/tictoc.h"
#include "parameters.h"
#include "keyframe.h"
#include "serialize.h"


class MultiMap
{
public:
    MultiMap();
    void allocateMemory();

    // 地图加载
    int loadMultiMap();          

    // 获取局部地图
    void extractSurroundingKeyFrames(pcl::PointCloud<PointType>::Ptr &nearKeyframes, const PointType &pose);
    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn);

private:
    int loadLidarMap();         // 加载激光地图
    int loadCloudKeyFrames();   // 加载点云关键帧
    int loadCornerAndSurf();    // 加载角点和平面点
    int loadSCbyScans();        // 加载Scancontext描述子
    int loadTrajectory();       // 加载轨迹
    int loadTransformations();  // 加载六自由度位姿
    
    int loadVisualMap();        // 加载视觉地图
    int loadPoseGraph();
    void loadKeyFrame(KeyFrame* cur_kf);
	void loadVocabulary(std::string voc_path);

private:
    // serialization
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version);

    friend class mapOptimization;
    /************************ LiDAR information ****************************/
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

    /************************ 用于提取局部地图 ****************************/
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; 
};


