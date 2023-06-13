#include "mapOptimization.h"

MapOptimization::MapOptimization()
{
    status = LocationStatus::INITING;
    just_relocate = true;
    relocate_cnt = 0;
    relocate_success_cnt = 0;

    binaryMapFile = binarymapName;
    queryCloud.reset(new pcl::PointCloud<PointType>());

    subCloud = nh.subscribe<lvi_sam_location_ros::cloud_info>("lio_sam/feature/cloud_info", 100, &MapOptimization::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
    pubLaserOdometryGlobal = nh.advertise<nav_msgs::Odometry>("lio_sam/mapping/odometry", 1);
    pubLaserOdometryIncremental = nh.advertise<nav_msgs::Odometry>("lio_sam/mapping/odometry_incremental", 1);

    // 体素滤波设置珊格大小
    downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
    downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
    downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
    downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity); // for surrounding key poses of scan-to-map optimization

    allocateMemory();
}

// 预先分配内存
void MapOptimization::allocateMemory()
{
    laserCloudOri.reset(new pcl::PointCloud<PointType>());
    coeffSel.reset(new pcl::PointCloud<PointType>());

    laserCloudOriCornerVec.resize(N_SCAN * Horizon_SCAN);
    coeffSelCornerVec.resize(N_SCAN * Horizon_SCAN);
    laserCloudOriCornerFlag.resize(N_SCAN * Horizon_SCAN);
    laserCloudOriSurfVec.resize(N_SCAN * Horizon_SCAN);
    coeffSelSurfVec.resize(N_SCAN * Horizon_SCAN);
    laserCloudOriSurfFlag.resize(N_SCAN * Horizon_SCAN);

    std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
    std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);

    laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());   // corner feature set from odoOptimization
    laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());     // surf feature set from odoOptimization
    laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled corner featuer set from odoOptimization
    laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>());   // downsampled surf featuer set from odoOptimization
    laserCloudLast.reset(new pcl::PointCloud<PointType>());
    laserCloudLastDS.reset(new pcl::PointCloud<PointType>());

    laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
    laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

    kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

    kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeLidarKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

    for (int i = 0; i < 6; ++i)
    {
        transformTobeMapped[i] = 0;
    }

    matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));
}

void MapOptimization::laserCloudInfoHandler(const lvi_sam_location_ros::cloud_infoConstPtr &msgIn)
{
    // extract time stamp
    // 提取当前时间戳
    timeLaserInfoStamp = msgIn->header.stamp;
    timeLaserInfoCur = msgIn->header.stamp.toSec();

    // extract info and feature cloud
    // 提取cloudinfo中的角点和面点
    cloudInfo = *msgIn;
    imageAvailable = cloudInfo.imageAvailable;
    pcl::fromROSMsg(msgIn->cloud_corner, *laserCloudCornerLast);
    pcl::fromROSMsg(msgIn->cloud_surface, *laserCloudSurfLast);
    pcl::fromROSMsg(msgIn->cloud_deskewed, *laserCloudLast); // yabao

    if (imageAvailable)
    {
        cv_bridge::CvImageConstPtr ptr;
        ptr = cv_bridge::toCvCopy(msgIn->image, sensor_msgs::image_encodings::MONO8);
        currentPicture = ptr->image;
    }

    // std::lock_guard<std::mutex> lock(mtx);

    if (timeLaserInfoCur - timeLastProcessing >= mappingProcessInterval)
    {

        std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();

        if(USE_TUNNING)
            updateParameters();

        switch (status)
        {
        case LocationStatus::INITING:
        case LocationStatus::LOST:
            relocate();
            break;
        case LocationStatus::TRACKING_GOOD:
        case LocationStatus::TRACKING_BAD:
            locate();
            break;
        }

        std::chrono::time_point<std::chrono::system_clock> end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        last_process_time = elapsed_seconds.count();

        updateTrajectoryInfo();
        timeLastProcessing = timeLaserInfoCur;
    }
    else
    {
        ROS_INFO("Throw this point cloud.");
    }
}

void MapOptimization::relocate()
{
    relocate_cnt++; // 记录系统运行以来的重定位次数

    if (1 && imageAvailable)
    {
        // 视觉激光融合重定位
        // 1.加载重定位帧（赋值queryCloud和queryPicture)
        loadQueryPicture(false);
        loadQueryCloud(false);
        // 2.粗重定位
        if (visualRelocate() != 0)
        {
            if (lidarRelocate() != 0)
            {
                std::cout << "Visual-LiDAR fusion rough relocation failure!" << std::endl;
                status = LocationStatus::LOST;
                return;
            }
        }
    }
    else
    {
        // 激光重定位
        loadQueryCloud(false);
        if (lidarRelocate() != 0)
        {
            std::cout << "Image unavailable, LiDAR rough relocation failure!" << std::endl;
            status = LocationStatus::LOST;
            return;
        }
    }

    // 3.精重定位
    if (refineRelocateResult() != 0)
    {
        std::cout << "Refine rough result failure!" << std::endl;
        status = LocationStatus::LOST;
        return;
    }

    status = LocationStatus::TRACKING_GOOD;
    just_relocate = true;
    relocate_success_cnt++; // 记录系统运行以来的重定位成功次数

    return;
}

// return  0:粗定位成功，验证成功
// return -1:粗定位失败
// return -2:粗定位成功，验证失败
int MapOptimization::relocate_MODULE_TEST()
{
    // 1.加载重定位帧（赋值queryCloud和queryPicture)
    loadQueryPicture(true);
    loadQueryCloud(true);
    // 2.粗重定位
    // 激光视觉融合重定位
    if (RELOCATE_METHOD == 0)
    {
        std::cout << "Start visual-LiDAR fusion rough relocation." << std::endl;
        if (visualRelocate() != 0)
        {
            if (lidarRelocate() != 0)
            {
                std::cout << "Visual-LiDAR fusion rough relocation failure!" << std::endl;
                status = LocationStatus::LOST;
                return -1;
            }
        }
    }
    else if (RELOCATE_METHOD == 1)
    {
        std::cout << "Start visual only rough relocation." << std::endl;
        if (visualRelocate() != 0)
        {
            std::cout << "Visual only rough relocation failure!" << std::endl;
            status = LocationStatus::LOST;
            return -1;
        }
    }
    else if (RELOCATE_METHOD == 2)
    {
        std::cout << "Start lidar only rough relocation." << std::endl;
        if (lidarRelocate() != 0)
        {
            std::cout << "lidar only rough relocation failure!" << std::endl;
            status = LocationStatus::LOST;
            return -1;
        }
    }
    else
    {
        std::cout << "Unkonw relocation method!" << std::endl;
        status = LocationStatus::LOST;
        return -1;
    }

    // 3.精重定位
    if (refineRelocateResult() != 0)
    {
        std::cout << "Refine relocation result failure!" << std::endl;
        status = LocationStatus::LOST;
        return -2;
    }
    status = LocationStatus::TRACKING_GOOD;
    just_relocate = true;
    return 0;
}

/*********************** visual relocation *******************************/

int MapOptimization::visualRelocate()
{
    // 1.词袋重定位
    int loop_index = detectLoop(queryPicture, queryPicture->index);
    if (loop_index != -1)
    {
        KeyFrame *old_kf = getKeyFrame(loop_index);

#ifdef NCLT
        // NCLT视觉重定位不做几何验证
        printf(" %d detect loop with %d \n", queryPicture->index, loop_index);
        cv::Mat query_img = queryPicture->image;
        cv::Mat old_img = old_kf->image;
        cv::Mat show_img;
        cv::hconcat(query_img, old_img, show_img);
        // cv::imshow("query picture - old picture", show_img);
        // cv::waitKey();
        std::cout << "visual coarse relocate success!" << std::endl;
        return 0;
#endif

        // 2.几何验证
        // 视觉几何验证似乎不是必要的，因为后面有激光的验证了
        Eigen::Vector3d queryPicture_T;
        Eigen::Matrix3d queryPicture_R;
        // 为什么改成了old_kf->findConnection()而不是queryPicture->findConnection()
        // 因为调用findConnection()的keyframe需要具有地图点，后面会进行pnp
        if (old_kf->findConnection(queryPicture, queryPicture_T, queryPicture_R))
        {

            // // 3.获取视觉粗定位结果(基于视觉路径的方式)
            // // 可以把相机坐标系转化为激光坐标系，更准确
            // //CameraFrame2LidarFrame(queryPicture_T, queryPicture_R, tmp_relocation_T, tmp_relocation_R);
            // tmp_relocation_T = queryPicture_T;
            // tmp_relocation_R = queryPicture_R;
            // Eigen::Vector3d tmp_relocation_YPR = Utility::R2ypr(tmp_relocation_R);
            // tmp_relocation_pose.x = tmp_relocation_T.x();
            // tmp_relocation_pose.y = tmp_relocation_T.y();
            // tmp_relocation_pose.z = tmp_relocation_T.z();
            // tmp_relocation_pose.yaw = tmp_relocation_YPR.x() / 180 * M_PI;
            // tmp_relocation_pose.pitch = tmp_relocation_YPR.y() / 180 * M_PI;
            // tmp_relocation_pose.roll = tmp_relocation_YPR.z() / 180 * M_PI;
            // relocation_lidar_index = -1;

            // 3.获取视觉粗定位结果(基于时间戳的方式)
            pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D(new pcl::PointCloud<PointTypePose>());
            *copy_cloudKeyPoses6D = *(map->cloudKeyPoses6D);
            double cloud_time, image_time;
            image_time = old_kf->time_stamp;
            int loopKeyPre = 0;
            // 找到第一个时间戳大于image的lidar，当然还有更精确的方法
            for (int i = 0; i < copy_cloudKeyPoses6D->size(); i++)
            {
                cloud_time = copy_cloudKeyPoses6D->points[i].time;
                if (cloud_time >= image_time)
                {
                    loopKeyPre = i;
                    break;
                }
            }

            tmp_relocation_pose = copy_cloudKeyPoses6D->points[loopKeyPre];
            relocation_lidar_index = loopKeyPre;
            tmp_relocation_T(0) = tmp_relocation_pose.x;
            tmp_relocation_T(1) = tmp_relocation_pose.y;
            tmp_relocation_T(2) = tmp_relocation_pose.z;
            Eigen::Vector3d tmp_relocation_YPR;
            tmp_relocation_YPR << tmp_relocation_pose.yaw / M_PI * 180,
                tmp_relocation_pose.pitch / M_PI * 180,
                tmp_relocation_pose.roll / M_PI * 180;
            tmp_relocation_R = Utility::ypr2R(tmp_relocation_YPR);

            ROS_INFO("Visual rough relocation success!");
            std::cout << "T is: " << tmp_relocation_T.transpose() << std::endl;
            std::cout << "YPR is: " << tmp_relocation_YPR.transpose() << std::endl;
            std::cout << "image time stamp: " << fixed << setprecision(5) << image_time << std::endl; // 不要科学计数法
            std::cout << "lidar time stamp: " << fixed << setprecision(5) << cloud_time << std::endl; // 不要科学计数法

            return 0;
        }
    }
    ROS_INFO("Visual rough relocation failure!");
    return -1;
}

// relocate_test = true:测试重定位模块，图片从路径加载
// relocate_test = false:正常运行
int MapOptimization::loadQueryPicture(bool relocate_test)
{
    double time_stamp = 0;
    int frame_index = map->keyframelist.size();
    Vector3d T = Vector3d::Zero();
    Matrix3d R = Matrix3d::Zero();
    vector<cv::Point3f> point_3d;
    vector<cv::Point2f> point_2d_uv;
    vector<cv::Point2f> point_2d_normal;
    vector<double> point_id;
    int sequence = 1;

    cv::Mat image;
    if (relocate_test)
        image = cv::imread(QUERY_IMAGE_PATH, 0);
    else
        image = currentPicture.clone();

    if (image.empty())
    {
        std::cout << "Load the query picture from: " << QUERY_IMAGE_PATH << " failed!" << std::endl;
        return -1;
    }

#ifdef NCLT
    cv::transpose(image, image);
    cv::flip(image, image, 1); // 90度旋转
#endif

    queryPicture = new KeyFrame(time_stamp, frame_index, T, R,
                                image, point_3d, point_2d_uv, point_2d_normal, point_id, sequence);
    std::cout << "Load the query picture success!" << std::endl;
    return 0;
}

int MapOptimization::detectLoop(KeyFrame *keyframe, int frame_index)
{
    DBoW2::QueryResults ret;

    map->db.query(keyframe->brief_descriptors, ret, 4, frame_index);

    bool find_loop = false;

    // a good match with its nerghbour
    // (adjust parameters)
    if (ret.size() >= 1 && ret[0].Score > 0.05)
        for (unsigned int i = 1; i < ret.size(); i++)
        {
            if (ret[i].Score > 0.015)
                find_loop = true;
        }

    if (find_loop)
    {
        int relo_index;
        relo_index = ret[0].Id;
        return relo_index;
    }
    else
        return -1;
}

KeyFrame *MapOptimization::getKeyFrame(int index)
{
    // unique_lock<mutex> lock(m_keyframelist);
    list<KeyFrame *>::iterator it = map->keyframelist.begin();
    for (; it != map->keyframelist.end(); it++)
    {
        if ((*it)->index == index)
            break;
    }
    if (it != map->keyframelist.end())
        return *it;
    else
        return NULL;
}

/*********************** lidar relocation *******************************/

int MapOptimization::lidarRelocate()
{
    // scancontext重定位
    if (LIDAR_RELOCATION_USE_SC)
    {
        // 1.调用scancontext接口
        auto detectResult = map->scManager.detectLoopClosureID(); // first: nn index, second: yaw diff
        int loopKeyPre = detectResult.first;
        float yaw_diff_rad = detectResult.second;
        // 2.移除scancontext中关于queryCloud的内容(不管成功失败都要移除)
        map->scManager.removeLastScancontextAndKeys();

        if (loopKeyPre == -1)
        {
            ROS_INFO("LiDAR rough relocation failure, couldn't find SC loop!");
            return -1;
        }
        if (loopKeyPre >= map->cloudKeyPoses6D->size())
        {
            std::cout << "Wrong LiDAR index!";
            return -1;
        }
        ROS_INFO("LiDAR rough relocation success, SC loop found: %d!", loopKeyPre);

#ifdef NCLT
        // NCLT数据集只用SC重定位，不取粗位姿了
        return 0;
#endif

        // 3.得到粗位姿，以找到的lidar帧作为粗位姿，并在yaw上做一点处理
        tmp_relocation_pose = map->cloudKeyPoses6D->points[loopKeyPre];
        tmp_relocation_pose.yaw = tmp_relocation_pose.yaw - yaw_diff_rad;
        relocation_lidar_index = loopKeyPre;
        tmp_relocation_T(0) = tmp_relocation_pose.x;
        tmp_relocation_T(1) = tmp_relocation_pose.y;
        tmp_relocation_T(2) = tmp_relocation_pose.z;
        Eigen::Vector3d tmp_relocation_YPR;
        tmp_relocation_YPR << tmp_relocation_pose.yaw / M_PI * 180,
            tmp_relocation_pose.pitch / M_PI * 180,
            tmp_relocation_pose.roll / M_PI * 180;
        tmp_relocation_R = Utility::ypr2R(tmp_relocation_YPR);

        std::cout << "T is: " << tmp_relocation_T.transpose() << std::endl;
        std::cout << "YPR is: " << tmp_relocation_YPR.transpose() << std::endl;
    }
    else
    {
    }

    return 0;
}

int MapOptimization::loadQueryCloud(bool relocate_test)
{
    if (relocate_test)
    {
        std::cout << "Loading the query cloud." << std::endl;
        std::string file_path = QUERY_LIDAR_PATH;
        if (file_path.substr(file_path.size() - 3, 3) == "pcd")
        {
            if (pcl::io::loadPCDFile(file_path, *queryCloud) == -1)
            {
                std::cout << "Load pcd file failed." << std::endl;
                return -1;
            }
        }
        else if (file_path.substr(file_path.size() - 3, 3) == "bin")
        {
            if (_loadBINFile(file_path, *queryCloud) == -1)
            {
                std::cout << "Load bin file failed." << std::endl;
                return -1;
            };
        }
        else
        {
            std::cout << "Didn't support thid format." << std::endl;
            return -1;
        }
        std::cout << "Load the query scan success: " << file_path << std::endl;
    }
    else
    {
        *queryCloud = *laserCloudLast;
    }

    map->scManager.makeAndSaveScancontextAndKeys(*queryCloud);

    return 0;
}

void MapOptimization::loopFindNearKeyframesByIndex(pcl::PointCloud<PointType>::Ptr &nearKeyframes, const int &key, const int &searchNum)
{
    // extract near keyframes
    nearKeyframes->clear();
    int cloudSize = map->cloudKeyPoses6D->size();
    for (int i = -searchNum; i <= searchNum; ++i)
    {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= cloudSize)
            continue;
        // *nearKeyframes += *transformPointCloud(cornerCloudKeyFrames[keyNear], &cloudKeyPoses6D->points[keyNear]);
        // *nearKeyframes += *transformPointCloud(surfCloudKeyFrames[keyNear],   &cloudKeyPoses6D->points[keyNear]);
        *nearKeyframes += *transformPointCloud(map->cloudKeyFrames[keyNear], &map->cloudKeyPoses6D->points[keyNear]);
    }

    if (nearKeyframes->empty())
        return;

    // downsample near keyframes
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
}

void MapOptimization::loopFindNearKeyframesByPose(pcl::PointCloud<PointType>::Ptr &nearKeyframes, const PointType &pose, const int &searchNum)
{
    // extract near keyframes
    nearKeyframes->clear();
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    kdtreeLidarKeyPoses->setInputCloud(map->cloudKeyPoses3D); // 可以移除去，一次性添加
    kdtreeLidarKeyPoses->nearestKSearch(pose, 2 * searchNum, pointSearchInd, pointSearchSqDis);
    for (int i = 0; i < pointSearchInd.size(); i++)
    {
        int index = pointSearchInd[i];
        // *nearKeyframes += *transformPointCloud(cornerCloudKeyFrames[keyNear], &cloudKeyPoses6D->points[keyNear]);
        // *nearKeyframes += *transformPointCloud(surfCloudKeyFrames[keyNear],   &cloudKeyPoses6D->points[keyNear]);
        *nearKeyframes += *transformPointCloud(map->cloudKeyFrames[index], &map->cloudKeyPoses6D->points[index]);
    }

    if (nearKeyframes->empty())
        return;

    // downsample near keyframes
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
}

int MapOptimization::refineRelocateResult()
{
    // extract cloud
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());

    // *cureKeyframeCloud = *queryCloud;
    *cureKeyframeCloud = *transformPointCloud(queryCloud, &tmp_relocation_pose);
    if (relocation_lidar_index != -1)
        loopFindNearKeyframesByIndex(prevKeyframeCloud, relocation_lidar_index, historyKeyframeSearchNum);
    else
    {
        PointType pose;
        pose.x = tmp_relocation_pose.x;
        pose.y = tmp_relocation_pose.y;
        pose.z = tmp_relocation_pose.z;
        loopFindNearKeyframesByPose(prevKeyframeCloud, pose, historyKeyframeSearchNum);
    }

    if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
    {
        std::cout << "cloud is too small" << std::endl;
        return -1;
    }

    if (RELOCATION_USE_ICP)
    {
        // ICP Settings
        static pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius * 2); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        // icp.setRANSACIterations(10);

        // Align clouds
        icp.setInputSource(cureKeyframeCloud);
        icp.setInputTarget(prevKeyframeCloud);
        pcl::PointCloud<PointType>::Ptr result(new pcl::PointCloud<PointType>());
        icp.align(*result);

        // icp得分实际算出的是点云对之间的“平均距离”，即得分越小越好
        if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
        {
            ROS_INFO("RELOCATION: ICP fitness test failed (%f > %f)", icp.getFitnessScore(), historyKeyframeFitnessScore);
            return -1;
        }
        else
        {
            ROS_INFO("RELOCATION: ICP fitness test passed (%f <= %f)", icp.getFitnessScore(), historyKeyframeFitnessScore);
        }

        // 计算重定位结果位姿
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = icp.getFinalTransformation();
        // transform from world origin to wrong pose
        Eigen::Affine3f tWrong = pclPointToAffine3f(tmp_relocation_pose);
        // transform from world origin to corrected pose
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;
        std::cout << "refine relocate result success, the result is: " << std::endl;
        // std::cout << tCorrect.matrix() << std::endl;

        // 获取重定位结果
        lastTransformation = tCorrect;
        relocation_T = tCorrect.translation().cast<double>();
        relocation_R = tCorrect.rotation().cast<double>();
        location_T = relocation_T; // 重定位结果也要赋值给定位
        location_R = relocation_R;
        Eigen::Vector3d tmp_YPR;
        tmp_YPR = Utility::R2ypr(relocation_R);
        std::cout << "T is: " << relocation_T.transpose() << std::endl;
        // std::cout << "R is: " << std::endl << relocation_R << std::endl;
        std::cout << "YPR is: " << tmp_YPR.transpose() << std::endl;

// 可视化
#ifdef SHOW_RELOCATE
        showLidarResult(prevKeyframeCloud, cureKeyframeCloud, "before icp refine");
        showLidarResult(prevKeyframeCloud, result, "after icp refine");
#endif

        // pcl::PointCloud<PointType>::Ptr result_test(new pcl::PointCloud<PointType>());
        // pcl::transformPointCloud(*queryCloud, *result_test, tCorrect.matrix());
        // showLidarResult(result_test, result, "test the calculation of final pose");
    }
    else
    {
        // NDT Settings
        static pcl::NormalDistributionsTransform<PointType, PointType> ndt;

        ndt.setMaximumIterations(60);       // 设置最大迭代次数（算法终止条件）
        ndt.setTransformationEpsilon(0.01); // 设置NDT迭代收敛阈值（算法终止条件）
        ndt.setStepSize(0.5);               // 设置搜索步长
        ndt.setResolution(2.5);             // 设置NDT网格分辨率，如果初始误差大于NDT分辨率，会导致NDT无法收敛

        // Align clouds
        ndt.setInputSource(cureKeyframeCloud);
        ndt.setInputTarget(prevKeyframeCloud);
        pcl::PointCloud<PointType>::Ptr result(new pcl::PointCloud<PointType>());
        ndt.align(*result);

        if (ndt.hasConverged() == false || ndt.getFitnessScore() > historyKeyframeFitnessScore)
        {
            ROS_INFO("RELOCATION: NDT fitness test failed (%f > %f)", ndt.getFitnessScore(), historyKeyframeFitnessScore);
            return -1;
        }
        else
        {
            ROS_INFO("RELOCATION: NDT fitness test passed (%f <= %f)", ndt.getFitnessScore(), historyKeyframeFitnessScore);
        }

        // 计算重定位结果位姿
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = ndt.getFinalTransformation();
        // transform from world origin to wrong pose
        Eigen::Affine3f tWrong = pclPointToAffine3f(tmp_relocation_pose);
        // transform from world origin to corrected pose
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;
        std::cout << "refine relocate result success, the result is: " << std::endl;
        // std::cout << tCorrect.matrix() << std::endl;

        // 获取重定位结果
        lastTransformation = tCorrect;
        relocation_T = tCorrect.translation().cast<double>();
        relocation_R = tCorrect.rotation().cast<double>();
        location_T = relocation_T; // 重定位结果也要赋值给定位
        location_R = relocation_R;
        Eigen::Vector3d tmp_YPR;
        tmp_YPR = Utility::R2ypr(relocation_R);
        std::cout << "T is: " << relocation_T.transpose() << std::endl;
        // std::cout << "R is: " << std::endl << relocation_R << std::endl;
        std::cout << "YPR is: " << tmp_YPR.transpose() << std::endl;

// 可视化
#ifdef SHOW_RELOCATE
        showLidarResult(prevKeyframeCloud, cureKeyframeCloud, "before ndt refine");
        showLidarResult(prevKeyframeCloud, result, "after ndt refine");
#endif
    }

    return 0;
}

/*********************** location *******************************/

void MapOptimization::locate()
{
    // 更新当前匹配结果的初始位姿
    updateInitialGuess();

    // 提取当前帧相关的关键帧并且构建点云局部地图
    TicToc t_ext;
    extractSurroundingKeyFrames();
    // ROS_INFO("extract local map cost: %f", t_ext.toc());

    // 对当前帧进行下采样
    downsampleCurrentScan();

    // 对点云配准进行优化问题构建求解
    TicToc t_opt;
    scan2MapOptimization();
    // ROS_INFO("scan to map cost: %f", t_opt.toc());

    // 将lidar里程记信息发送出去
    publishOdometry();
    // // 发送可视化点云信息
    // publishFrames();
}

// 作为基于优化方式的点云匹配，初始值是非常重要的，一个好的初始值会帮助优化问题快速收敛且避免局部最优解的情况
void MapOptimization::updateInitialGuess()
{
    // save current transformation before any processing
    // transformTobeMapped是上一帧优化后的最佳位姿
    incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);

    static Eigen::Affine3f lastImuTransformation;
    // initialization
    if (just_relocate) {
        just_relocate = false;
        transIncreConstSpeed = Eigen::Affine3f::Identity();

        pcl::getTranslationAndEulerAngles(lastTransformation,
                                          transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                          transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

        // 保存磁力计得到的位姿，平移置0
        lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
        ROS_INFO("Initial guess: relocation.");
        return;
    }

    // // use imu pre-integration estimation for pose guess
    // static bool lastImuPreTransAvailable = false;
    // static Eigen::Affine3f lastImuPreTransformation;
    // // 如果有预积分节点提供的里程记
    // if (0 && cloudInfo.odomAvailable == true)
    // {
    //     // 将提供的初值转成eigen的数据结构保存下来
    //     Eigen::Affine3f transBack = pcl::getTransformation(cloudInfo.initialGuessX,    cloudInfo.initialGuessY,     cloudInfo.initialGuessZ,
    //                                                         cloudInfo.initialGuessRoll, cloudInfo.initialGuessPitch, cloudInfo.initialGuessYaw);
    //     // 这个标志位表示是否收到过第一帧预积分里程记信息
    //     if (lastImuPreTransAvailable == false)
    //     {
    //         // 将当前里程记结果记录下来
    //         lastImuPreTransformation = transBack;
    //         // 收到第一个里程记数据以后这个标志位就是true
    //         lastImuPreTransAvailable = true;
    //     } else {
    //         // 计算上一个里程记的结果和当前里程记结果之间的delta pose
    //         Eigen::Affine3f transIncre = lastImuPreTransformation.inverse() * transBack;
    //         Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
    //         // 将这个增量加到上一帧最佳位姿上去，就是当前帧位姿的一个先验估计位姿
    //         Eigen::Affine3f transFinal = transTobe * transIncre;
    //         // 将eigen变量转成欧拉角和平移的形式
    //         pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
    //                                                         transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

    //         // 同理，把当前帧的值保存下来
    //         lastImuPreTransformation = transBack;
    //         // 虽然有里程记信息，仍然需要把imu磁力计得到的旋转记录下来
    //         lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
    //         ROS_INFO("Initial guess: odometry.");
    //         return;
    //     }
    // }

    // // use imu incremental estimation for pose guess (only rotation)
    // // 如果没有里程记信息，就是用imu的旋转信息来更新，因为单纯使用imu无法得到靠谱的平移信息，因此，平移直接置0
    // if (0 && cloudInfo.imuAvailable == true)
    // {
    //     // 初值计算方式和上面相同，只不过注意平移置0
    //     Eigen::Affine3f transBack = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);
    //     Eigen::Affine3f transIncre = lastImuTransformation.inverse() * transBack;

    //     Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
    //     Eigen::Affine3f transFinal = transTobe * transIncre;
    //     pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
    //                                                     transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

    //     lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
    //     ROS_INFO("Initial guess: imu.");
    //     return;
    // }

    // 旋转用imu,平移用匀速运动模型来估计
    if (cloudInfo.imuAvailable == true)
    {
        double ratio = (timeLaserInfoCur - timeLastProcessing) / time_diff;
        Eigen::Vector3d tmp_linear = transIncreConstSpeed.translation().cast<double>() * ratio;

        Eigen::Affine3f transBack = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);
        Eigen::Affine3f transIncre = lastImuTransformation.inverse() * transBack;

        Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
        Eigen::Affine3f transFinal = transTobe * transIncre *
                    pcl::getTransformation(tmp_linear.x(), tmp_linear.y(), tmp_linear.z(), 0, 0, 0);

        pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                                        transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

        lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
        ROS_INFO("Initial guess: imu and const speed.");
        return;
    }

    // 匀速运动模型(比imu好用)
    // 还可以优化，加入时间因素，这里默认每帧时间间隔相等
    if (1)
    {
        // double ratio = (timeLaserInfoCur - timeLastProcessing) / time_diff;
        // Eigen::Vector3d tmp_linear = transIncreConstSpeed.translation().cast<double>() * ratio;
        // Eigen::Vector3d tmp_angular = transIncreConstSpeed.rotation().eulerAngles(2, 1, 0).cast<double>() * ratio;
        // std::cout << "ratio: " << ratio << std::endl;
        // std::cout << "tmp_linear: " << tmp_linear.transpose() << std::endl;
        // std::cout << "tmp_angular: " << tmp_angular.transpose() << std::endl;    // 有时接近pi，有时接近0

        Eigen::Affine3f transGuess, transIncre;
        transIncre = transIncreConstSpeed; // 默认每帧时间间隔相等
        // transIncre = pcl::getTransformation(tmp_linear.x(), tmp_linear.y(), tmp_linear.z(),
        //                                     tmp_angular.z(), tmp_angular.y(), tmp_angular.x());
        transGuess = lastTransformation * transIncre;
        pcl::getTranslationAndEulerAngles(transGuess,
                                          transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                          transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

        // lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);
        ROS_INFO("Initial guess: const speed.");
        return;
    }
}

void MapOptimization::extractNearby()
{
    pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointType>());
    std::vector<int> pointSearchInd;     // 保存kdtree提取出来的元素的索引
    std::vector<float> pointSearchSqDis; // 保存距离查询位置的距离的数组

    // extract all the nearby key poses and downsample them
    kdtreeSurroundingKeyPoses->setInputCloud(map->cloudKeyPoses3D); // create kd-tree
    // 根据预测的位置，提取一定距离内的关键帧
    PointType search_pose;
    search_pose.x = transformTobeMapped[3];
    search_pose.y = transformTobeMapped[4];
    search_pose.z = transformTobeMapped[5];
    kdtreeSurroundingKeyPoses->radiusSearch(search_pose, (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis);

    // 根据查询的结果，把这些点的位置存进一个点云结构中
    for (int i = 0; i < (int)pointSearchInd.size(); ++i)
    {
        int id = pointSearchInd[i];
        surroundingKeyPoses->push_back(map->cloudKeyPoses3D->points[id]);
    }

    // 避免关键帧过多，因此做一个下采样
    downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
    downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);
    // 确认每个下采样后的点的索引，就使用一个最近邻搜索，其索引赋值给这个点的intensity数据位
    for (auto &pt : surroundingKeyPosesDS->points)
    {
        kdtreeSurroundingKeyPoses->nearestKSearch(pt, 1, pointSearchInd, pointSearchSqDis);
        pt.intensity = map->cloudKeyPoses3D->points[pointSearchInd[0]].intensity;
    }

    // // also extract some latest key frames in case the robot rotates in one position
    // int numPoses = cloudKeyPoses3D->size();
    // // 刚刚是提取了一些空间上比较近的关键帧，然后再提取一些时间上比较近的关键帧
    // for (int i = numPoses-1; i >= 0; --i)
    // {
    //     // 最近十秒的关键帧也保存下来
    //     if (timeLaserInfoCur - cloudKeyPoses6D->points[i].time < 10.0)
    //         surroundingKeyPosesDS->push_back(cloudKeyPoses3D->points[i]);
    //     else
    //         break;
    // }

    // 根据筛选出来的关键帧进行局部地图构建
    extractCloud(surroundingKeyPosesDS);
}

void MapOptimization::extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract)
{
    // fuse the map
    // 分别存储角点和面点相关的局部地图
    laserCloudCornerFromMap->clear();
    laserCloudSurfFromMap->clear();
    for (int i = 0; i < (int)cloudToExtract->size(); ++i)
    {
        // // 简单校验一下关键帧距离不能太远，这个实际上不太会触发
        // if (pointDistance(cloudToExtract->points[i], cloudKeyPoses3D->back()) > surroundingKeyframeSearchRadius)
        //     continue;
        // 取出提出出来的关键帧的索引
        int thisKeyInd = (int)cloudToExtract->points[i].intensity;
        // 如果这个关键帧对应的点云信息已经存储在一个地图容器里
        if (laserCloudMapContainer.find(thisKeyInd) != laserCloudMapContainer.end())
        {
            // transformed cloud available
            // 直接从容器中取出来加到局部地图中
            *laserCloudCornerFromMap += laserCloudMapContainer[thisKeyInd].first;
            *laserCloudSurfFromMap += laserCloudMapContainer[thisKeyInd].second;
        }
        else
        {
            // transformed cloud not available
            // 如果这个点云没有实现存取，那就通过该帧对应的位姿，把该帧点云从当前帧的位姿转到世界坐标系下
            pcl::PointCloud<PointType> laserCloudCornerTemp = *transformPointCloud(map->cornerCloudKeyFrames[thisKeyInd], &map->cloudKeyPoses6D->points[thisKeyInd]);
            pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud(map->surfCloudKeyFrames[thisKeyInd], &map->cloudKeyPoses6D->points[thisKeyInd]);
            // 点云转换之后加到局部地图中
            *laserCloudCornerFromMap += laserCloudCornerTemp;
            *laserCloudSurfFromMap += laserCloudSurfTemp;
            // 把转换后的面点和角点存进这个容器中，方便后续直接加入点云地图，避免点云转换的操作，节约时间
            laserCloudMapContainer[thisKeyInd] = std::make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
        }
    }

    TicToc t_ds;
    // extractSurroundingKeyFrames()函数中，最花时间的就是这个下采样的过程
    // Downsample the surrounding corner key frames (or map)
    // 将提取的关键帧的点云转到世界坐标系下后，避免点云过度密集，因此对面点和角点的局部地图做一个下采样的过程
    downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
    downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
    laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->size();
    // Downsample the surrounding surf key frames (or map)
    downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
    downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
    laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();
    ROS_DEBUG("downsize local map cost: %f", t_ds.toc());

    // yabao
    // // clear map cache if too large
    // // 如果这个局部地图容器过大，就clear一下，避免占用内存过大
    // if (laserCloudMapContainer.size() > 1000)
    //     laserCloudMapContainer.clear();
    
}

void MapOptimization::extractSurroundingKeyFrames()
{
    // 如果当前没有关键帧，就return了
    if (map->cloudKeyPoses3D->points.empty() == true)
        return;

    // if (loopClosureEnableFlag == true)
    // {
    //     extractForLoopClosure();
    // } else {
    //     extractNearby();
    // }

    extractNearby();
}

void MapOptimization::downsampleCurrentScan()
{
    // Downsample cloud from current scan
    // 当前帧的角点和面点分别进行下采样，也就是为了减少计算量
    laserCloudCornerLastDS->clear();
    downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
    downSizeFilterCorner.filter(*laserCloudCornerLastDS);
    laserCloudCornerLastDSNum = laserCloudCornerLastDS->size();

    laserCloudSurfLastDS->clear();
    downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
    downSizeFilterSurf.filter(*laserCloudSurfLastDS);
    laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();
}

void MapOptimization::updatePointAssociateToMap()
{
    // 将欧拉角转成eigen的对象
    transPointAssociateToMap = trans2Affine3f(transformTobeMapped);
}

void MapOptimization::cornerOptimization()
{
    updatePointAssociateToMap();
    // 使用openmp并行加速
    #pragma omp parallel for num_threads(numberOfCores)
    // 遍历当前帧的角点
    for (int i = 0; i < laserCloudCornerLastDSNum; i++)
    {
        PointType pointOri, pointSel, coeff;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        pointOri = laserCloudCornerLastDS->points[i];
        // 将该点从当前帧通过初始的位姿转换到地图坐标系下去
        pointAssociateToMap(&pointOri, &pointSel);
        // 在角点地图里寻找距离当前点比较近的5个点
        kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

        cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));
        // 计算找到的点中距离当前点最远的点，如果距离太大那说明这个约束不可信，就跳过  
        if (pointSearchSqDis[4] < 1.0) {
            float cx = 0, cy = 0, cz = 0;
            // 计算协方差矩阵
            // 首先计算均值
            for (int j = 0; j < 5; j++) {
                cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
                cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
                cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
            }
            cx /= 5; cy /= 5;  cz /= 5;

            float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
            for (int j = 0; j < 5; j++) {
                float ax = laserCloudCornerFromMapDS->points[pointSearchInd[j]].x - cx;
                float ay = laserCloudCornerFromMapDS->points[pointSearchInd[j]].y - cy;
                float az = laserCloudCornerFromMapDS->points[pointSearchInd[j]].z - cz;

                a11 += ax * ax; a12 += ax * ay; a13 += ax * az;
                a22 += ay * ay; a23 += ay * az;
                a33 += az * az;
            }
            a11 /= 5; a12 /= 5; a13 /= 5; a22 /= 5; a23 /= 5; a33 /= 5;

            matA1.at<float>(0, 0) = a11; matA1.at<float>(0, 1) = a12; matA1.at<float>(0, 2) = a13;
            matA1.at<float>(1, 0) = a12; matA1.at<float>(1, 1) = a22; matA1.at<float>(1, 2) = a23;
            matA1.at<float>(2, 0) = a13; matA1.at<float>(2, 1) = a23; matA1.at<float>(2, 2) = a33;
            // 特征值分解
            cv::eigen(matA1, matD1, matV1);
            // 这是线特征性，要求最大特征值大于3倍的次大特征值
            if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {

                float x0 = pointSel.x;
                float y0 = pointSel.y;
                float z0 = pointSel.z;
                // 特征向量对应的就是直线的方向向量
                // 通过点的均值往两边拓展，构成一个线的两个端点
                float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                float z2 = cz - 0.1 * matV1.at<float>(0, 2);
                // 下面是计算点到线的残差和垂线方向（及雅克比方向）
                float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                                + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                                + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)) * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

                float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

                float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                            + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

                float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                            - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                            + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;   // 

                float ld2 = a012 / l12;     // 点到直线的距离
                // 一个简单的核函数，残差越大权重降低
                float s = 1 - 0.9 * fabs(ld2);

                coeff.x = s * la;
                coeff.y = s * lb;
                coeff.z = s * lc;   // 雅克比方向
                coeff.intensity = s * ld2;  // 残差
                // 如果残差小于10cm，就认为是一个有效的约束
                if (s > 0.1) {
                    laserCloudOriCornerVec[i] = pointOri;
                    coeffSelCornerVec[i] = coeff;
                    laserCloudOriCornerFlag[i] = true;
                }
            }
        }
    }
}

void MapOptimization::surfOptimization()
{
    updatePointAssociateToMap();

    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < laserCloudSurfLastDSNum; i++)
    {
        PointType pointOri, pointSel, coeff;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        // 同样找5个面点
        pointOri = laserCloudSurfLastDS->points[i];
        pointAssociateToMap(&pointOri, &pointSel); 
        kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

        Eigen::Matrix<float, 5, 3> matA0;
        Eigen::Matrix<float, 5, 1> matB0;
        Eigen::Vector3f matX0;
        // 平面方程Ax + By + Cz + 1 = 0
        matA0.setZero();
        matB0.fill(-1);
        matX0.setZero();
        // 同样最大距离不能超过1m
        if (pointSearchSqDis[4] < 1.0) {
            for (int j = 0; j < 5; j++) {
                matA0(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                matA0(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                matA0(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
            }
            // 求解Ax = b这个超定方程
            matX0 = matA0.colPivHouseholderQr().solve(matB0);
            // 求出来x的就是这个平面的法向量
            float pa = matX0(0, 0);
            float pb = matX0(1, 0);
            float pc = matX0(2, 0);
            float pd = 1;

            float ps = sqrt(pa * pa + pb * pb + pc * pc);
            // 归一化，将法向量模长统一为1
            pa /= ps; pb /= ps; pc /= ps; pd /= ps;

            bool planeValid = true;
            for (int j = 0; j < 5; j++) {
                // 每个点代入平面方程，计算点到平面的距离，如果距离大于0.2m认为这个平面曲率偏大，就是无效的平面
                if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                            pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                            pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2) {
                    planeValid = false;
                    break;
                }
            }
            // 如果通过了平面的校验
            if (planeValid) {
                // 计算当前点到平面的距离
                float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;
                // 分母不是很明白，为了更多的面点用起来？
                float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
                        + pointSel.y * pointSel.y + pointSel.z * pointSel.z));

                coeff.x = s * pa;
                coeff.y = s * pb;
                coeff.z = s * pc;
                coeff.intensity = s * pd2;

                if (s > 0.1) {
                    laserCloudOriSurfVec[i] = pointOri;
                    coeffSelSurfVec[i] = coeff;
                    laserCloudOriSurfFlag[i] = true;
                }
            }
        }
    }
}

// 将角点约束和面点约束统一到一起
void MapOptimization::combineOptimizationCoeffs()
{
    // combine corner coeffs
    for (int i = 0; i < laserCloudCornerLastDSNum; ++i)
    {
        // 只有标志位为true的时候才是有效约束
        if (laserCloudOriCornerFlag[i] == true)
        {
            laserCloudOri->push_back(laserCloudOriCornerVec[i]);
            coeffSel->push_back(coeffSelCornerVec[i]);
        }
    }
    // combine surf coeffs
    for (int i = 0; i < laserCloudSurfLastDSNum; ++i)
    {
        if (laserCloudOriSurfFlag[i] == true)
        {
            laserCloudOri->push_back(laserCloudOriSurfVec[i]);
            coeffSel->push_back(coeffSelSurfVec[i]);
        }
    }
    // reset flag for next iteration
    // 标志位清零
    std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
    std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
}

bool MapOptimization::LMOptimization(int iterCount)
{
    // 原始的loam代码是将lidar坐标系转到相机坐标系，这里把原先loam中的代码拷贝了过来，但是为了坐标系的统一，就先转到相机系优化，然后结果转回lidar系
    // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
    // lidar <- camera      ---     camera <- lidar
    // x = z                ---     x = y
    // y = x                ---     y = z
    // z = y                ---     z = x
    // roll = yaw           ---     roll = pitch
    // pitch = roll         ---     pitch = yaw
    // yaw = pitch          ---     yaw = roll

    // lidar -> camera
    // 将lidar系转到相机系
    float srx = sin(transformTobeMapped[1]);
    float crx = cos(transformTobeMapped[1]);
    float sry = sin(transformTobeMapped[2]);
    float cry = cos(transformTobeMapped[2]);
    float srz = sin(transformTobeMapped[0]);
    float crz = cos(transformTobeMapped[0]);

    int laserCloudSelNum = laserCloudOri->size();
    if (laserCloudSelNum < 50)
    {
        return false;
    }

    cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

    PointType pointOri, coeff;

    for (int i = 0; i < laserCloudSelNum; i++)
    {
        // 首先将当前点以及点到线（面）的单位向量转到相机系
        // lidar -> camera
        pointOri.x = laserCloudOri->points[i].y;
        pointOri.y = laserCloudOri->points[i].z;
        pointOri.z = laserCloudOri->points[i].x;
        // lidar -> camera
        coeff.x = coeffSel->points[i].y;
        coeff.y = coeffSel->points[i].z;
        coeff.z = coeffSel->points[i].x;
        coeff.intensity = coeffSel->points[i].intensity;
        // in camera
        // 相机系下的旋转顺序是Y - X - Z对应lidar系下Z -Y -X
        float arx = (crx * sry * srz * pointOri.x + crx * crz * sry * pointOri.y - srx * sry * pointOri.z) * coeff.x + (-srx * srz * pointOri.x - crz * srx * pointOri.y - crx * pointOri.z) * coeff.y + (crx * cry * srz * pointOri.x + crx * cry * crz * pointOri.y - cry * srx * pointOri.z) * coeff.z;

        float ary = ((cry * srx * srz - crz * sry) * pointOri.x + (sry * srz + cry * crz * srx) * pointOri.y + crx * cry * pointOri.z) * coeff.x + ((-cry * crz - srx * sry * srz) * pointOri.x + (cry * srz - crz * srx * sry) * pointOri.y - crx * sry * pointOri.z) * coeff.z;

        float arz = ((crz * srx * sry - cry * srz) * pointOri.x + (-cry * crz - srx * sry * srz) * pointOri.y) * coeff.x + (crx * crz * pointOri.x - crx * srz * pointOri.y) * coeff.y + ((sry * srz + cry * crz * srx) * pointOri.x + (crz * sry - cry * srx * srz) * pointOri.y) * coeff.z;
        // lidar -> camera
        // 这里就是把camera转到lidar了
        matA.at<float>(i, 0) = arz;
        matA.at<float>(i, 1) = arx;
        matA.at<float>(i, 2) = ary;
        matA.at<float>(i, 3) = coeff.z;
        matA.at<float>(i, 4) = coeff.x;
        matA.at<float>(i, 5) = coeff.y;
        matB.at<float>(i, 0) = -coeff.intensity;
    }
    // 构造JTJ以及-JTe矩阵
    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    // 求解增量
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

    if (iterCount == 0)
    {
        // 检查一下是否有退化的情况
        cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));
        // 对JTJ进行特征值分解
        cv::eigen(matAtA, matE, matV);
        matV.copyTo(matV2);

        isDegenerate = false;
        float eignThre[6] = {100, 100, 100, 100, 100, 100};
        for (int i = 5; i >= 0; i--)
        {
            // 特征值从小到大遍历，如果小于阈值就认为退化
            if (matE.at<float>(0, i) < eignThre[i])
            {
                // 对应的特征向量全部置0
                for (int j = 0; j < 6; j++)
                {
                    matV2.at<float>(i, j) = 0;
                }
                isDegenerate = true;
            }
            else
            {
                break;
            }
        }
        matP = matV.inv() * matV2;
    }
    // 如果发生退化，就对增量进行修改，退化方向不更新
    if (isDegenerate)
    {
        cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
        matX.copyTo(matX2);
        matX = matP * matX2;
    }
    // 增量更新
    transformTobeMapped[0] += matX.at<float>(0, 0);
    transformTobeMapped[1] += matX.at<float>(1, 0);
    transformTobeMapped[2] += matX.at<float>(2, 0);
    transformTobeMapped[3] += matX.at<float>(3, 0);
    transformTobeMapped[4] += matX.at<float>(4, 0);
    transformTobeMapped[5] += matX.at<float>(5, 0);
    // 计算更新的旋转和平移大小
    float deltaR = sqrt(
        pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
        pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
        pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
    float deltaT = sqrt(
        pow(matX.at<float>(3, 0) * 100, 2) +
        pow(matX.at<float>(4, 0) * 100, 2) +
        pow(matX.at<float>(5, 0) * 100, 2));
    // 旋转和平移增量足够小，认为优化问题收敛了
    if (deltaR < 0.05 && deltaT < 0.05)
    {
        return true; // converged
    }
    // 否则继续优化
    return false; // keep optimizing
}

void MapOptimization::scan2MapOptimization()
{
    int iterCount_ = 0; // yabao

    // 如果没有关键帧，那也没办法做当前帧到局部地图的匹配
    if (map->cloudKeyPoses3D->points.empty())
        return;
    // 判断当前帧的角点数和面点数是否足够
    if (laserCloudCornerLastDSNum > edgeFeatureMinValidNum && laserCloudSurfLastDSNum > surfFeatureMinValidNum)
    {
        // 分别把角点面点局部地图构建kdtree
        kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
        kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

        // 迭代求解
        for (int iterCount = 0; iterCount < 50; iterCount++)
        {
            laserCloudOri->clear();
            coeffSel->clear();

            cornerOptimization();
            surfOptimization();

            combineOptimizationCoeffs();

            if (LMOptimization(iterCount) == true)
                break;

            iterCount_++;
        }

        // 优化问题结束
        transformUpdate();
    }
    else
    {
        ROS_WARN("Not enough features! Only %d edge and %d planar features available.", laserCloudCornerLastDSNum, laserCloudSurfLastDSNum);
    }

    ROS_INFO("\033[1;32m----> iter time: %d.\033[0m", iterCount_); // yabao
    if (iterCount_ == 50)
    {
        status = LocationStatus::LOST;
        return;
    }
    else if (iterCount_ > 30)
    {
        status = LocationStatus::TRACKING_BAD;
    }
    else
    {
        status = LocationStatus::TRACKING_GOOD;
    }

    Eigen::Affine3f tCorrect = trans2Affine3f(transformTobeMapped);
    transIncreConstSpeed = lastTransformation.inverse() * tCorrect;
    time_diff = timeLaserInfoCur - timeLastProcessing;
    lastTransformation = tCorrect;

    location_T = tCorrect.translation().cast<double>();
    location_R = tCorrect.rotation().cast<double>();
    Eigen::Vector3d tmp_YPR;
    tmp_YPR = Utility::R2ypr(location_R);
    std::cout << "T is: " << location_T.transpose() << std::endl;
    std::cout << "YPR is: " << tmp_YPR.transpose() << std::endl;
}

// 把结果和imu进行一些加权融合
void MapOptimization::transformUpdate()
{
    // // 可以获取九轴imu的世界系下的姿态
    // if (cloudInfo.imuAvailable == true)
    // {
    //     // 因为roll 和 pitch原则上全程可观，因此这里把lidar推算出来的姿态和磁力计结果做一个加权平均
    //     // 首先判断车翻了没有，车翻了好像做slam也没有什么意义了，当然手持设备可以pitch很大，这里主要避免插值产生的奇异
    //     if (std::abs(cloudInfo.imuPitchInit) < 1.4)
    //     {
    //         double imuWeight = imuRPYWeight;
    //         tf::Quaternion imuQuaternion;
    //         tf::Quaternion transformQuaternion;
    //         double rollMid, pitchMid, yawMid;

    //         // slerp roll
    //         // lidar匹配获得的roll角转成四元数
    //         transformQuaternion.setRPY(transformTobeMapped[0], 0, 0);
    //         // imu获得的roll角
    //         imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
    //         // 使用四元数球面插值
    //         tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
    //         // 插值结果作为roll的最终结果
    //         transformTobeMapped[0] = rollMid;

    //         // 下面pitch角同理
    //         // slerp pitch
    //         transformQuaternion.setRPY(0, transformTobeMapped[1], 0);
    //         imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
    //         tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
    //         transformTobeMapped[1] = pitchMid;
    //     }
    // }
    // // 对roll pitch和z进行一些约束，主要针对室内2d场景下，已知2d先验可以加上这些约束
    // // transformTobeMapped[0] = constraintTransformation(transformTobeMapped[0], rotation_tollerance);
    // // transformTobeMapped[1] = constraintTransformation(transformTobeMapped[1], rotation_tollerance);
    // // transformTobeMapped[5] = constraintTransformation(transformTobeMapped[5], z_tollerance);
    // 最终的结果也可以转成eigen的结构
    incrementalOdometryAffineBack = trans2Affine3f(transformTobeMapped);
}

float MapOptimization::constraintTransformation(float value, float limit)
{
    if (value < -limit)
        value = -limit;
    if (value > limit)
        value = limit;

    return value;
}

void MapOptimization::publishOdometry()
{
    // Publish odometry for ROS (global)
    // 发送当前帧的位姿
    nav_msgs::Odometry laserOdometryROS;
    laserOdometryROS.header.stamp = timeLaserInfoStamp;
    laserOdometryROS.header.frame_id = odometryFrame;
    laserOdometryROS.child_frame_id = "odom_mapping";
    laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
    laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
    laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
    laserOdometryROS.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
    pubLaserOdometryGlobal.publish(laserOdometryROS);

    // Publish TF
    // 发送lidar在odom坐标系下的tf
    static tf::TransformBroadcaster br;
    tf::Transform t_odom_to_lidar = tf::Transform(tf::createQuaternionFromRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]),
                                                    tf::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
    tf::StampedTransform trans_odom_to_lidar = tf::StampedTransform(t_odom_to_lidar, timeLaserInfoStamp, odometryFrame, lidarFrame);
    br.sendTransform(trans_odom_to_lidar);

    // Publish odometry for ROS (incremental)
    // 发送增量位姿变换
    // 这里主要用于给imu预积分模块使用，需要里程计是平滑的
    static bool lastIncreOdomPubFlag = false;
    static nav_msgs::Odometry laserOdomIncremental; // incremental odometry msg
    static Eigen::Affine3f increOdomAffine; // incremental odometry in affine
    // 该标志位处理一次后始终为true
    if (lastIncreOdomPubFlag == false)
    {
        lastIncreOdomPubFlag = true;
        // 记录当前位姿
        laserOdomIncremental = laserOdometryROS;
        increOdomAffine = trans2Affine3f(transformTobeMapped);
    } else {
        // 上一帧的最佳位姿和当前帧最佳位姿（scan matching之后，而不是根据回环或者gps调整之后的位姿）之间的位姿增量
        Eigen::Affine3f affineIncre = incrementalOdometryAffineFront.inverse() * incrementalOdometryAffineBack;
        // 位姿增量叠加到上一帧位姿上
        increOdomAffine = increOdomAffine * affineIncre;
        float x, y, z, roll, pitch, yaw;
        // 分解成欧拉角+平移向量
        pcl::getTranslationAndEulerAngles (increOdomAffine, x, y, z, roll, pitch, yaw);
        // 如果有imu信号，同样对roll和pitch做插值
        // if (cloudInfo.imuAvailable == true)
        // {
        //     if (std::abs(cloudInfo.imuPitchInit) < 1.4)
        //     {
        //         double imuWeight = 0.1;
        //         tf::Quaternion imuQuaternion;
        //         tf::Quaternion transformQuaternion;
        //         double rollMid, pitchMid, yawMid;

        //         // slerp roll
        //         transformQuaternion.setRPY(roll, 0, 0);
        //         imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
        //         tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
        //         roll = rollMid;

        //         // slerp pitch
        //         transformQuaternion.setRPY(0, pitch, 0);
        //         imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
        //         tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
        //         pitch = pitchMid;
        //     }
        // }
        laserOdomIncremental.header.stamp = timeLaserInfoStamp;
        laserOdomIncremental.header.frame_id = odometryFrame;
        laserOdomIncremental.child_frame_id = "odom_mapping";
        laserOdomIncremental.pose.pose.position.x = x;
        laserOdomIncremental.pose.pose.position.y = y;
        laserOdomIncremental.pose.pose.position.z = z;
        laserOdomIncremental.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        // 协方差这一位作为是否退化的标志位
        if (isDegenerate)
            laserOdomIncremental.pose.covariance[0] = 1;
        else
            laserOdomIncremental.pose.covariance[0] = 0;
    }
    pubLaserOdometryIncremental.publish(laserOdomIncremental);  //发送给imu预积分节点
}

Eigen::Affine3f MapOptimization::pclPointToAffine3f(PointTypePose thisPoint)
{
    return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
}

Eigen::Affine3f MapOptimization::trans2Affine3f(float transformIn[])
{
    return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
}

PointTypePose MapOptimization::trans2PointTypePose(float transformIn[])
{
    PointTypePose thisPose6D;
    thisPose6D.x = transformIn[3];
    thisPose6D.y = transformIn[4];
    thisPose6D.z = transformIn[5];
    thisPose6D.roll = transformIn[0];
    thisPose6D.pitch = transformIn[1];
    thisPose6D.yaw = transformIn[2];
    return thisPose6D;
}

pcl::PointCloud<PointType>::Ptr MapOptimization::transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
    // 使用openmp进行并行加速
    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        // 每个点都施加RX+t这样一个过程
        cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
        cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
        cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }
    return cloudOut;
}

void MapOptimization::pointAssociateToMap(PointType const *const pi, PointType *const po)
{
    po->x = transPointAssociateToMap(0, 0) * pi->x + transPointAssociateToMap(0, 1) * pi->y + transPointAssociateToMap(0, 2) * pi->z + transPointAssociateToMap(0, 3);
    po->y = transPointAssociateToMap(1, 0) * pi->x + transPointAssociateToMap(1, 1) * pi->y + transPointAssociateToMap(1, 2) * pi->z + transPointAssociateToMap(1, 3);
    po->z = transPointAssociateToMap(2, 0) * pi->x + transPointAssociateToMap(2, 1) * pi->y + transPointAssociateToMap(2, 2) * pi->z + transPointAssociateToMap(2, 3);
    po->intensity = pi->intensity;
}

void MapOptimization::updateTrajectoryInfo()
{
    traj_timestamps.push_back(timeLaserInfoCur);
    traj_scores.push_back(last_score);
    traj_process_times.push_back(last_process_time);
    traj_status.push_back(status);
    traj_Ts.push_back(location_T);
    traj_Rs.push_back(location_R);
}

void MapOptimization::updateParameters()
{
    nh.param<float>("surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);
    ROS_INFO("now surroundingKeyframeSearchRadius is: %f", surroundingKeyframeSearchRadius);
}

// 全局可视化线程
void MapOptimization::visualizeGlobalMapThread()
{
    ros::Rate rate(1.0);
    while (ros::ok())
    {
        rate.sleep();
        // publishGlobalMap();
    }

    // 当ros被杀死之后
    // 执行保存轨迹功能
    std::cout << "Saving the trajectory now!" << std::endl;
    std::string filename("/home/yabao/trajectory.txt");
    std::ofstream traj_file;
    traj_file.open(filename.c_str());
    traj_file << fixed;

    for (int i = 0; i < traj_status.size(); i++)
    {
        if (traj_status[i] == LocationStatus::TRACKING_GOOD ||
            traj_status[i] == LocationStatus::TRACKING_BAD)
        {
            Eigen::Quaterniond q(traj_Rs[i]);

            traj_file << traj_timestamps[i] << " " << setprecision(7) << traj_Ts[i](0) << " " << traj_Ts[i](1) << " " << traj_Ts[i](2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
        }
    }
    traj_file.close();

    // 平均每帧的处理时间
    double sum_process_time = 0;
    for (int i = 0; i < traj_process_times.size(); i++)
    {
        sum_process_time += traj_process_times[i];
    }
    std::cout << "Average process time: " << sum_process_time * 1000 / traj_process_times.size() << " ms" << std::endl;
    std::cout << "Relocation time is: " << relocate_cnt << std::endl;
    std::cout << "Relocation sucess time is: " << relocate_success_cnt << std::endl;
}

/*********************** map function *******************************/
int MapOptimization::loadFolderMap()
{
    TicToc t_load(true);
    map = new MultiMap();
    map->loadMultiMap();
    t_load.toc("Load Floder Map total use time");
    return 0;
}

int MapOptimization::saveBinaryMap()
{
    std::ofstream out(binaryMapFile, std::ios_base::binary);
    if (!out)
    {
        std::cerr << "Cannot Write to Mapfile: " << binaryMapFile << std::endl;
        exit(-1);
    }
    std::cout << "Saving Mapfile: " << binaryMapFile << " ...." << std::endl;
    boost::archive::binary_oarchive oa(out, boost::archive::no_header);
    oa << map;
    std::cout << "Mapfile saved succesfully!" << std::endl;
    out.close();
    return 0;
}

int MapOptimization::loadBinaryMap()
{
    TicToc t_load(true);
    TicToc t_binary(true);
    std::ifstream in(binaryMapFile, std::ios_base::binary);
    if (!in)
    {
        cerr << "Cannot Open Mapfile: " << binaryMapFile << ". No existing map file found!" << std::endl;
        return false;
    }
    cout << "Loading Mapfile: " << binaryMapFile << "....." << std::endl;
    boost::archive::binary_iarchive ia(in, boost::archive::no_header);
    ia >> map; // 会执行map的构造函数
    in.close();
    t_binary.toc("Load binary file use time");

    // 加载scManager
    TicToc t_sc(true);
    // map->loadSCbyScans();
    //  还有优化空间，Scancontext类的数据类型比较简单
    for (auto &cloud : map->cloudKeyFrames)
    {
        map->scManager.makeAndSaveScancontextAndKeys(*cloud);
    }
    t_sc.toc("Load Scan Context use time");

    // 加载字典
    TicToc t_vocabulary(true);
    map->loadVocabulary(VOC_PATH);
    t_vocabulary.toc("Load vocabulary use time");

    // 加载地图词袋
    TicToc t_db(true);
    for (auto &kf : map->keyframelist)
        map->db.add(kf->brief_descriptors);
    t_db.toc("Load Brief Database use time");

    t_load.toc("Load Binary Map total use time");

    return 0;
}

void MapOptimization::speedUpExtractSurroundingKeyFrames()
{
    TicToc t_speedup(true);

    if(map->cornerCloudKeyFrames.size() != map->surfCloudKeyFrames.size()) {
        std::cout << "cornerCloudKeyFrames size is not equal to surfCloudKeyFrames size" << std::endl;
        return;
    }
    if(map->cornerCloudKeyFrames.size() != map->cloudKeyPoses3D->points.size()) {
        std::cout << "cornerCloudKeyFrames size is not equal to cloudKeyPoses3D size" << std::endl;
        return;
    }

    int num = map->cloudKeyPoses3D->points.size();
    for(int i = 0; i < num; i++) {
        int thisKeyInd = map->cloudKeyPoses3D->points[i].intensity;

        pcl::PointCloud<PointType> laserCloudCornerTemp = *transformPointCloud(map->cornerCloudKeyFrames[thisKeyInd], &map->cloudKeyPoses6D->points[thisKeyInd]);
        pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud(map->surfCloudKeyFrames[thisKeyInd], &map->cloudKeyPoses6D->points[thisKeyInd]);
        laserCloudMapContainer[thisKeyInd] = std::make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
    }

    t_speedup.toc("Speed up use time");
}

