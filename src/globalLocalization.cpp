#include "globalLocalization.h"

mapOptimization::mapOptimization()
{
    binaryMapFile = binarymapName;  // yabao

    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.relinearizeSkip = 1;
    isam = new ISAM2(parameters);

    pubKeyPoses = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/trajectory", 1);
    pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/map_global", 1);
    pubOdomAftMappedROS = nh.advertise<nav_msgs::Odometry> ("lio_sam/mapping/odometry", 1);
    pubPath = nh.advertise<nav_msgs::Path>("lio_sam/mapping/path", 1);
    pubMatchImg = nh.advertise<sensor_msgs::Image>("match_image", 10);
    pubInitialInfo = nh.advertise<std_msgs::String>("info/initial_method", 5);

    pubHistoryKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/icp_loop_closure_history_cloud", 1);
    pubIcpKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/icp_loop_closure_corrected_cloud", 1);
    pubRecentKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/map_local", 1);
    pubRecentKeyFrame = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/cloud_registered", 1);
    pubCloudRegisteredRaw = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/cloud_registered_raw", 1);

    subLaserCloudInfo = nh.subscribe<lvi_sam_localization::cloud_info>("lio_sam/feature/cloud_info", 10, &mapOptimization::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
    subGPS = nh.subscribe<nav_msgs::Odometry> (gpsTopic, 200, &mapOptimization::gpsHandler, this, ros::TransportHints().tcpNoDelay());
    
    //added by gc
    // 从rviz获取粗位姿
    subIniPoseFromRviz = nh.subscribe("/initialpose", 8, &mapOptimization::initialpose_callback, this);
    pubLaserCloudInWorld = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/lasercloud_in_world", 1);
    pubMapWorld = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/cloud_map_map",1);
    //fortest_publasercloudINWorld = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/laserclouinmapframe",1);
    pubOdomToMapPose = nh.advertise<geometry_msgs::PoseStamped>("lio_sam/mapping/pose_odomTo_map", 1);
    //added by gc

    downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
    downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
    downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
    downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity); // for surrounding key poses of scan-to-map optimization
    
    allocateMemory();
}

void mapOptimization::allocateMemory()
{
    cloudGlobalMap.reset(new pcl::PointCloud<PointType>());             
    cloudGlobalMapDS.reset(new pcl::PointCloud<PointType>());           
    cloudScanForInitialize.reset(new pcl::PointCloud<PointType>());
    resetLIO();
    //added by gc
    for (int i = 0; i < 6; ++i){
        transformInTheWorld[i] = 0;
    }

    for (int i = 0; i < 6; ++i){
        tranformOdomToWorld[i] = 0;
    }
    initializedFlag = NonInitialized;
    cloudGlobalLoad();      
    //added by gc
}

void mapOptimization::resetLIO()
{
    cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
    cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

    //kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
    //kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

    laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());   // corner feature set from odoOptimization
    laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());     // surf feature set from odoOptimization
    laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled corner featuer set from odoOptimization
    laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>());   // downsampled surf featuer set from odoOptimization

    laserCloudLast.reset(new pcl::PointCloud<PointType>());         // yabao
    laserCloudLastDS.reset(new pcl::PointCloud<PointType>());       // yabao

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

    laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
    laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

    kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

    latestKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
    nearHistoryKeyFrameCloud.reset(new pcl::PointCloud<PointType>());

    for (int i = 0; i < 6; ++i){
        transformTobeMapped[i] = 0;
    }

    matP.setZero();

    queryCloud.reset(new pcl::PointCloud<PointType>());                 // yabao
    kdtreeLidarKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());       // yabao
}

void mapOptimization::laserCloudInfoHandler(const lvi_sam_localization::cloud_infoConstPtr& msgIn)
{

    // extract time stamp
    timeLaserInfoStamp = msgIn->header.stamp;
    timeLaserCloudInfoLast = msgIn->header.stamp.toSec();  

    // extract info and feature cloud
    cloudInfo = *msgIn;
    pcl::fromROSMsg(msgIn->cloud_corner,  *laserCloudCornerLast);
    pcl::fromROSMsg(msgIn->cloud_surface, *laserCloudSurfLast);
    
    // yabao
    mtxRelocate.lock();
    pcl::fromROSMsg(msgIn->cloud_deskewed, *laserCloudLast);        
    imageAvailable = cloudInfo.imageAvailable;
    if (imageAvailable)
    {
        cv_bridge::CvImageConstPtr ptr;
        ptr = cv_bridge::toCvCopy(msgIn->image, sensor_msgs::image_encodings::MONO8);
        currentPicture = ptr->image;
    }
    mtxRelocate.unlock();

    /************************************added by gc*****************************/
    //if the sysytem is not initialized offer the first scan for the system to initialize
    //the LIO system stsrt working only when the localization initializing is finished
    if(initializedFlag == NonInitialized || initializedFlag == Initializing)
    {
        if(cloudScanForInitialize->points.size() == 0)
        {
            downsampleCurrentScan();
            mtx_general.lock();
            *cloudScanForInitialize += *laserCloudCornerLastDS;
            *cloudScanForInitialize += *laserCloudSurfLastDS;
            mtx_general.unlock();
            laserCloudCornerLastDS->clear();
            laserCloudSurfLastDS->clear();
            laserCloudCornerLastDSNum = 0;
            laserCloudSurfLastDSNum = 0;

            transformTobeMapped[0] = cloudInfo.imuRollInit;
            transformTobeMapped[1] = cloudInfo.imuPitchInit;
            transformTobeMapped[2] = cloudInfo.imuYawInit;
            if (!useImuHeadingInitialization)
                transformTobeMapped[2] = 0;
            
        }
        return;
    }
    frameNum++;
    /************************************added by gc*****************************/

    std::lock_guard<std::mutex> lock(mtx);

    if (timeLaserCloudInfoLast - timeLastProcessing >= mappingProcessInterval) {
        timeLastProcessing = timeLaserCloudInfoLast;
        // 更新当前匹配结果的初始位姿
        updateInitialGuess();           
        // 提取当前帧相关的关键帧并且构建点云局部地图
        extractSurroundingKeyFrames();  
        // 对当前帧进行下采样
        downsampleCurrentScan();
        // 对点云配准进行优化问题构建求解
        scan2MapOptimization();
        // 根据配准结果确定是否是关键帧
        saveKeyFramesAndFactor();
        // 调整全局轨迹
        //correctPoses();
        // 将lidar里程记信息发送出去
        publishOdometry();
        // 发送可视化点云信息
        publishFrames();
    }
}

void mapOptimization::gpsHandler(const nav_msgs::Odometry::ConstPtr& gpsMsg)
{
    gpsQueue.push_back(*gpsMsg);
}

void mapOptimization::pointAssociateToMap(PointType const * const pi, PointType * const po)
{
    po->x = transPointAssociateToMap(0,0) * pi->x + transPointAssociateToMap(0,1) * pi->y + transPointAssociateToMap(0,2) * pi->z + transPointAssociateToMap(0,3);
    po->y = transPointAssociateToMap(1,0) * pi->x + transPointAssociateToMap(1,1) * pi->y + transPointAssociateToMap(1,2) * pi->z + transPointAssociateToMap(1,3);
    po->z = transPointAssociateToMap(2,0) * pi->x + transPointAssociateToMap(2,1) * pi->y + transPointAssociateToMap(2,2) * pi->z + transPointAssociateToMap(2,3);
    po->intensity = pi->intensity;
}

pcl::PointCloud<PointType>::Ptr mapOptimization::transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    PointType *pointFrom;

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);

    for (int i = 0; i < cloudSize; ++i){

        pointFrom = &cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom->x + transCur(0,1) * pointFrom->y + transCur(0,2) * pointFrom->z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom->x + transCur(1,1) * pointFrom->y + transCur(1,2) * pointFrom->z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom->x + transCur(2,1) * pointFrom->y + transCur(2,2) * pointFrom->z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom->intensity;
    }
    return cloudOut;
}

gtsam::Pose3 mapOptimization::pclPointTogtsamPose3(PointTypePose thisPoint)
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                        gtsam::Point3(double(thisPoint.x),    double(thisPoint.y),     double(thisPoint.z)));
}

gtsam::Pose3 mapOptimization::trans2gtsamPose(float transformIn[])
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]),
                        gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
}

Eigen::Affine3f mapOptimization::pclPointToAffine3f(PointTypePose thisPoint)
{
    return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
}

Eigen::Affine3f mapOptimization::trans2Affine3f(float transformIn[])
{
    return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
}

PointTypePose mapOptimization::trans2PointTypePose(float transformIn[])
{
    PointTypePose thisPose6D;
    thisPose6D.x = transformIn[3];
    thisPose6D.y = transformIn[4];
    thisPose6D.z = transformIn[5];
    thisPose6D.roll  = transformIn[0];
    thisPose6D.pitch = transformIn[1];
    thisPose6D.yaw   = transformIn[2];
    return thisPose6D;
}

void mapOptimization::updateInitialGuess()
{
    static Eigen::Affine3f lastImuTransformation;
    // initialization
    if (cloudKeyPoses3D->points.empty())
    {
        transformTobeMapped[0] = cloudInfo.imuRollInit;
        transformTobeMapped[1] = cloudInfo.imuPitchInit;
        transformTobeMapped[2] = cloudInfo.imuYawInit;

        if (!useImuHeadingInitialization)
            transformTobeMapped[2] = 0;

        lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
        return;
    }

    // use imu pre-integration estimation for pose guess
    if (cloudInfo.odomAvailable == true && cloudInfo.imuPreintegrationResetId == imuPreintegrationResetId)
    {
        transformTobeMapped[0] = cloudInfo.initialGuessRoll;
        transformTobeMapped[1] = cloudInfo.initialGuessPitch;
        transformTobeMapped[2] = cloudInfo.initialGuessYaw;

        transformTobeMapped[3] = cloudInfo.initialGuessX;
        transformTobeMapped[4] = cloudInfo.initialGuessY;
        transformTobeMapped[5] = cloudInfo.initialGuessZ;

        lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
        return;
    }

    // use imu incremental estimation for pose guess (only rotation)
    if (cloudInfo.imuAvailable == true)
    {
        Eigen::Affine3f transBack = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);
        Eigen::Affine3f transIncre = lastImuTransformation.inverse() * transBack;//gc: the transform of IMU between two scans

        Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
        Eigen::Affine3f transFinal = transTobe * transIncre;
        pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                            transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

        lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
        return;
    }
}

void mapOptimization::extractForLoopClosure()
{
    /**************gc added**************/
    // change-1
    // in this place the maximum of numPoses is winSize
    pcl::PointCloud<PointType>::Ptr cloudToExtract(new pcl::PointCloud<PointType>());
    int numPoses = win_cloudKeyPoses3D.size();
    for (int i =  numPoses-1; i >=0; --i)
    {
        cloudToExtract->push_back(win_cloudKeyPoses3D[i]);

    }
    extractCloud(cloudToExtract);
    /**************gc added**************/

    // pcl::PointCloud<PointType>::Ptr cloudToExtract(new pcl::PointCloud<PointType>());
    // int numPoses = cloudKeyPoses3D->size();
    // for (int i = numPoses-1; i >= 0; --i)
    // {
    //     if ((int)cloudToExtract->size() <= surroundingKeyframeSize)
    //         cloudToExtract->push_back(cloudKeyPoses3D->points[i]);
    //     else
    //         break;
    // }
    // extractCloud(cloudToExtract);
}

void mapOptimization::extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract)
{
    /**************gc added**************/
    //change-2
    //std::cout << "cloudToExtract size: " << cloudToExtract->size() << std::endl;
    std::vector<pcl::PointCloud<PointType>> laserCloudCornerSurroundingVec;
    std::vector<pcl::PointCloud<PointType>> laserCloudSurfSurroundingVec;

    laserCloudCornerSurroundingVec.resize(cloudToExtract->size());
    laserCloudSurfSurroundingVec.resize(cloudToExtract->size());

    // extract surrounding map
#pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < (int)cloudToExtract->size(); ++i)
    {
        PointTypePose thisPose6D;
        thisPose6D = win_cloudKeyPoses6D[i];
        laserCloudCornerSurroundingVec[i]  = *transformPointCloud(win_cornerCloudKeyFrames[i],  &thisPose6D);
        laserCloudSurfSurroundingVec[i]    = *transformPointCloud(win_surfCloudKeyFrames[i],    &thisPose6D);
    }

    // fuse the map
    laserCloudCornerFromMap->clear();
    laserCloudSurfFromMap->clear();
    for (int i = 0; i < (int)cloudToExtract->size(); ++i)
    {
        *laserCloudCornerFromMap += laserCloudCornerSurroundingVec[i];
        *laserCloudSurfFromMap   += laserCloudSurfSurroundingVec[i];
    }

    // Downsample the surrounding corner key frames (or map)
    downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
    downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
    laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->size();
    // Downsample the surrounding surf key frames (or map)
    downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
    downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
    laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();

}

void mapOptimization::extractSurroundingKeyFrames()
{
    if (cloudKeyPoses3D->points.empty() == true)
        return;

    // if (loopClosureEnableFlag == true)
    // {
    //     extractForLoopClosure();    
    // } else {
    //     extractNearby();
    // }

    extractForLoopClosure();    //gc: the name is misleading
}

void mapOptimization::downsampleCurrentScan()
{
    // Downsample cloud from current scan
    laserCloudCornerLastDS->clear();
    downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
    downSizeFilterCorner.filter(*laserCloudCornerLastDS);
    laserCloudCornerLastDSNum = laserCloudCornerLastDS->size();

    laserCloudSurfLastDS->clear();
    downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
    downSizeFilterSurf.filter(*laserCloudSurfLastDS);
    laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();
}

void mapOptimization::updatePointAssociateToMap()
{
    transPointAssociateToMap = trans2Affine3f(transformTobeMapped);
}

void mapOptimization::cornerOptimization()
{
    updatePointAssociateToMap();

#pragma omp parallel for num_threads(numberOfCores)
    //gc: for every corner point
    for (int i = 0; i < laserCloudCornerLastDSNum; i++)
    {
        PointType pointOri, pointSel, coeff;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        pointOri = laserCloudCornerLastDS->points[i];
        //gc: calculate its location in the map using the prediction pose
        pointAssociateToMap(&pointOri, &pointSel);
        kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

        cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

        if (pointSearchSqDis[4] < 1.0) {
            float cx = 0, cy = 0, cz = 0;
            for (int j = 0; j < 5; j++) {
                cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
                cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
                cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
            }
            //gc: the average coordinate of the most nearest points
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

            cv::eigen(matA1, matD1, matV1);

            if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {

                float x0 = pointSel.x;
                float y0 = pointSel.y;
                float z0 = pointSel.z;
                float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                float z2 = cz - 0.1 * matV1.at<float>(0, 2);

                float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                    + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                    + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)) * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

                float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

                float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                            + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

                float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                float ld2 = a012 / l12;

                float s = 1 - 0.9 * fabs(ld2);

                coeff.x = s * la;
                coeff.y = s * lb;
                coeff.z = s * lc;
                coeff.intensity = s * ld2;

                if (s > 0.1) {
                    laserCloudOriCornerVec[i] = pointOri;
                    coeffSelCornerVec[i] = coeff;
                    laserCloudOriCornerFlag[i] = true;
                }
            }
        }
    }
}

void mapOptimization::surfOptimization()
{
    updatePointAssociateToMap();

#pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < laserCloudSurfLastDSNum; i++)
    {
        PointType pointOri, pointSel, coeff;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        pointOri = laserCloudSurfLastDS->points[i];
        pointAssociateToMap(&pointOri, &pointSel);
        kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

        Eigen::Matrix<float, 5, 3> matA0;
        Eigen::Matrix<float, 5, 1> matB0;
        Eigen::Vector3f matX0;

        matA0.setZero();
        matB0.fill(-1);
        matX0.setZero();

        if (pointSearchSqDis[4] < 1.0) {
            for (int j = 0; j < 5; j++) {
                matA0(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                matA0(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                matA0(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
            }

            matX0 = matA0.colPivHouseholderQr().solve(matB0);

            float pa = matX0(0, 0);
            float pb = matX0(1, 0);
            float pc = matX0(2, 0);
            float pd = 1;

            float ps = sqrt(pa * pa + pb * pb + pc * pc);
            pa /= ps; pb /= ps; pc /= ps; pd /= ps;

            bool planeValid = true;
            for (int j = 0; j < 5; j++) {
                if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                            pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                            pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2) {
                    planeValid = false;
                    break;
                }
            }

            if (planeValid) {
                float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

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

void mapOptimization::combineOptimizationCoeffs()
{
    // combine corner coeffs
    for (int i = 0; i < laserCloudCornerLastDSNum; ++i){
        if (laserCloudOriCornerFlag[i] == true){
            laserCloudOri->push_back(laserCloudOriCornerVec[i]);
            coeffSel->push_back(coeffSelCornerVec[i]);
        }
    }
    // combine surf coeffs
    for (int i = 0; i < laserCloudSurfLastDSNum; ++i){
        if (laserCloudOriSurfFlag[i] == true){
            laserCloudOri->push_back(laserCloudOriSurfVec[i]);
            coeffSel->push_back(coeffSelSurfVec[i]);
        }
    }
    // reset flag for next iteration
    std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
    std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
}

bool mapOptimization::LMOptimization(int iterCount)
{
    // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
    // lidar <- camera      ---     camera <- lidar
    // x = z                ---     x = y
    // y = x                ---     y = z
    // z = y                ---     z = x
    // roll = yaw           ---     roll = pitch
    // pitch = roll         ---     pitch = yaw
    // yaw = pitch          ---     yaw = roll

    // lidar -> camera
    float srx = sin(transformTobeMapped[1]);
    float crx = cos(transformTobeMapped[1]);
    float sry = sin(transformTobeMapped[2]);
    float cry = cos(transformTobeMapped[2]);
    float srz = sin(transformTobeMapped[0]);
    float crz = cos(transformTobeMapped[0]);

    int laserCloudSelNum = laserCloudOri->size();
    if (laserCloudSelNum < 50) {
        return false;
    }

    cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));

    PointType pointOri, coeff;

    for (int i = 0; i < laserCloudSelNum; i++) {
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
        float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
                    + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
                    + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;

        float ary = ((cry*srx*srz - crz*sry)*pointOri.x
                        + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
                    + ((-cry*crz - srx*sry*srz)*pointOri.x
                        + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;

        float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
                    + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                    + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;
        // lidar -> camera
        matA.at<float>(i, 0) = arz;
        matA.at<float>(i, 1) = arx;
        matA.at<float>(i, 2) = ary;
        matA.at<float>(i, 3) = coeff.z;
        matA.at<float>(i, 4) = coeff.x;
        matA.at<float>(i, 5) = coeff.y;
        matB.at<float>(i, 0) = -coeff.intensity;
    }

    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

    if (iterCount == 0) {

        cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

        cv::eigen(matAtA, matE, matV);
        matV.copyTo(matV2);

        isDegenerate = false;
        float eignThre[6] = {100, 100, 100, 100, 100, 100};
        for (int i = 5; i >= 0; i--) {
            if (matE.at<float>(0, i) < eignThre[i]) {
                for (int j = 0; j < 6; j++) {
                    matV2.at<float>(i, j) = 0;
                }
                isDegenerate = true;
            } else {
                break;
            }
        }
        matP = matV.inv() * matV2;
    }

    if (isDegenerate) {
        cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
        matX.copyTo(matX2);
        matX = matP * matX2;
    }

    transformTobeMapped[0] += matX.at<float>(0, 0);
    transformTobeMapped[1] += matX.at<float>(1, 0);
    transformTobeMapped[2] += matX.at<float>(2, 0);
    transformTobeMapped[3] += matX.at<float>(3, 0);
    transformTobeMapped[4] += matX.at<float>(4, 0);
    transformTobeMapped[5] += matX.at<float>(5, 0);

    float deltaR = sqrt(
            pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
            pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
            pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
    float deltaT = sqrt(
            pow(matX.at<float>(3, 0) * 100, 2) +
            pow(matX.at<float>(4, 0) * 100, 2) +
            pow(matX.at<float>(5, 0) * 100, 2));

    if (deltaR < 0.05 && deltaT < 0.05) {
        return true; // converged
    }
    return false; // keep optimizing
}

void mapOptimization::scan2MapOptimization()
{
    if (cloudKeyPoses3D->points.empty())
        return;

    if (laserCloudCornerLastDSNum > edgeFeatureMinValidNum && laserCloudSurfLastDSNum > surfFeatureMinValidNum)
    {
        kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
        kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

        for (int iterCount = 0; iterCount < 30; iterCount++)
        {
            laserCloudOri->clear();
            coeffSel->clear();
            
            cornerOptimization();
            surfOptimization();

            combineOptimizationCoeffs();

            if (LMOptimization(iterCount) == true)
                break;
        }
        
        transformUpdate();
    } else {
        ROS_WARN("Not enough features! Only %d edge and %d planar features available.", laserCloudCornerLastDSNum, laserCloudSurfLastDSNum);
    }
}

void mapOptimization::transformUpdate()
{
    if (cloudInfo.imuAvailable == true)
    {
        if (std::abs(cloudInfo.imuPitchInit) < 1.4)
        {
            double imuWeight = 0.01;
            tf::Quaternion imuQuaternion;
            tf::Quaternion transformQuaternion;
            double rollMid, pitchMid, yawMid;

            // slerp roll
            transformQuaternion.setRPY(transformTobeMapped[0], 0, 0);
            imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
            //gc: interpolate between Imu roll measurement and angle from lidar calculation
            tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
            transformTobeMapped[0] = rollMid;

            // slerp pitch
            transformQuaternion.setRPY(0, transformTobeMapped[1], 0);
            imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
            //gc: interpolate between Imu roll measurement and angle from lidar calculation
            tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
            transformTobeMapped[1] = pitchMid;
        }
    }

    transformTobeMapped[0] = constraintTransformation(transformTobeMapped[0], rotation_tollerance);
    transformTobeMapped[1] = constraintTransformation(transformTobeMapped[1], rotation_tollerance);
    transformTobeMapped[5] = constraintTransformation(transformTobeMapped[5], z_tollerance);
}

float mapOptimization::constraintTransformation(float value, float limit)
{
    if (value < -limit)
        value = -limit;
    if (value > limit)
        value = limit;

    return value;
}

bool mapOptimization::saveFrame()
{
    if (cloudKeyPoses3D->points.empty())
        return true;

    Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
    Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                                        transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
    Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);
    //gc: judge whther should generate key pose
    if (abs(roll)  < surroundingkeyframeAddingAngleThreshold &&
        abs(pitch) < surroundingkeyframeAddingAngleThreshold &&
        abs(yaw)   < surroundingkeyframeAddingAngleThreshold &&
        sqrt(x*x + y*y + z*z) < surroundingkeyframeAddingDistThreshold)
        return false;

    return true;
}

void mapOptimization::addOdomFactor()
{
    //gc: the first key pose
    if (cloudKeyPoses3D->points.empty())
    {
        noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
        gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
        initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
    }else{
        noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
        gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
        gtsam::Pose3 poseTo   = trans2gtsamPose(transformTobeMapped);
        //gc: add constraint between current pose and previous pose
        gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size()-1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));
        initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
    }
}

void mapOptimization::saveKeyFramesAndFactor()
{
    if (saveFrame() == false)
        return;

    addOdomFactor();
    //addGPSFactor();
    //addLoopFactor();

    // update iSAM
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();

    gtSAMgraph.resize(0);
    initialEstimate.clear();

    //save key poses
    PointType thisPose3D;
    PointTypePose thisPose6D;
    Pose3 latestEstimate;

    isamCurrentEstimate = isam->calculateEstimate();
    latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size()-1);
    // cout << "****************************************************" << endl;
    // isamCurrentEstimate.print("Current estimate: ");

    //gc:cloudKeyPoses3D can be used to calculate the nearest key frames
    thisPose3D.x = latestEstimate.translation().x();
    thisPose3D.y = latestEstimate.translation().y();
    thisPose3D.z = latestEstimate.translation().z();
    thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as index
    cloudKeyPoses3D->push_back(thisPose3D);


    thisPose6D.x = thisPose3D.x;
    thisPose6D.y = thisPose3D.y;
    thisPose6D.z = thisPose3D.z;
    thisPose6D.intensity = thisPose3D.intensity ; // this can be used as index
    thisPose6D.roll  = latestEstimate.rotation().roll();
    thisPose6D.pitch = latestEstimate.rotation().pitch();
    thisPose6D.yaw   = latestEstimate.rotation().yaw();
    thisPose6D.time = timeLaserCloudInfoLast;
    cloudKeyPoses6D->push_back(thisPose6D);

    // change-3
    /* added gc */
    mtxWin.lock();
    win_cloudKeyPoses3D.push_back(thisPose3D);
    win_cloudKeyPoses6D.push_back(thisPose6D);
    if(win_cloudKeyPoses3D.size() > winSize)
    {
        win_cloudKeyPoses3D.erase(win_cloudKeyPoses3D.begin());
        win_cloudKeyPoses6D.erase(win_cloudKeyPoses6D.begin());
    }
    /* added gc */

    // cout << "****************************************************" << endl;
    // cout << "Pose covariance:" << endl;
    // cout << isam->marginalCovariance(isamCurrentEstimate.size()-1) << endl << endl;
    poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size()-1);

    // save updated transform
    transformTobeMapped[0] = latestEstimate.rotation().roll();
    transformTobeMapped[1] = latestEstimate.rotation().pitch();
    transformTobeMapped[2] = latestEstimate.rotation().yaw();
    transformTobeMapped[3] = latestEstimate.translation().x();
    transformTobeMapped[4] = latestEstimate.translation().y();
    transformTobeMapped[5] = latestEstimate.translation().z();

    // save all the received edge and surf points
    pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());

    pcl::copyPointCloud(*laserCloudCornerLastDS,  *thisCornerKeyFrame);
    pcl::copyPointCloud(*laserCloudSurfLastDS,    *thisSurfKeyFrame);

    // save key frame cloud
    // cornerCloudKeyFrames.push_back(thisCornerKeyFrame);      // yabao
    // surfCloudKeyFrames.push_back(thisSurfKeyFrame);

    //change-4
    /* added gc */
    win_cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
    win_surfCloudKeyFrames.push_back(thisSurfKeyFrame);
    if(win_cornerCloudKeyFrames.size() > winSize)
    {
        win_cornerCloudKeyFrames.erase(win_cornerCloudKeyFrames.begin());
        win_surfCloudKeyFrames.erase(win_surfCloudKeyFrames.begin());
    }
    mtxWin.unlock();
    /* added gc */

    // save path for visualization
    updatePath(thisPose6D);
}

void mapOptimization::updatePath(const PointTypePose& pose_in)
{
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time().fromSec(pose_in.time);
    pose_stamped.header.frame_id = "odom";
    pose_stamped.pose.position.x = pose_in.x;
    pose_stamped.pose.position.y = pose_in.y;
    pose_stamped.pose.position.z = pose_in.z;
    tf::Quaternion q = tf::createQuaternionFromRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();

    globalPath.poses.push_back(pose_stamped);
}

void mapOptimization::publishOdometry()
{
    // Publish odometry for ROS
    nav_msgs::Odometry laserOdometryROS;
    laserOdometryROS.header.stamp = timeLaserInfoStamp;
    laserOdometryROS.header.frame_id = "odom";
    laserOdometryROS.child_frame_id = "odom_mapping";
    laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
    laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
    laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
    laserOdometryROS.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
    laserOdometryROS.pose.covariance[0] = double(imuPreintegrationResetId);
    pubOdomAftMappedROS.publish(laserOdometryROS);
}

void mapOptimization::publishFrames()
{
    if (cloudKeyPoses3D->points.empty())
        return;
    // publish key poses
    publishCloud(&pubKeyPoses, cloudKeyPoses3D, timeLaserInfoStamp, "odom");
    // Publish surrounding key frames
    publishCloud(&pubRecentKeyFrames, laserCloudSurfFromMapDS, timeLaserInfoStamp, "odom");
    // publish registered key frame
    //gc: feature points
    if (pubRecentKeyFrame.getNumSubscribers() != 0)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
        PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
        *cloudOut += *transformPointCloud(laserCloudCornerLastDS,  &thisPose6D);
        *cloudOut += *transformPointCloud(laserCloudSurfLastDS,    &thisPose6D);
        publishCloud(&pubRecentKeyFrame, cloudOut, timeLaserInfoStamp, "odom");
    }
    //added *****************by gc
    if(pubLaserCloudInWorld.getNumSubscribers() != 0)
    {
        pcl::PointCloud<PointType>::Ptr cloudInBase(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr cloudOutInWorld(new pcl::PointCloud<PointType>());
        PointTypePose thisPose6DInOdom = trans2PointTypePose(transformTobeMapped);
        Eigen::Affine3f T_thisPose6DInOdom = pclPointToAffine3f(thisPose6DInOdom);
        mtxtranformOdomToWorld.lock();
        PointTypePose pose_Odom_Map = trans2PointTypePose(tranformOdomToWorld);
        mtxtranformOdomToWorld.unlock();
        Eigen::Affine3f T_pose_Odom_Map = pclPointToAffine3f(pose_Odom_Map);

        Eigen::Affine3f T_poseInMap = T_pose_Odom_Map * T_thisPose6DInOdom;
        *cloudInBase += *laserCloudCornerLastDS;
        *cloudInBase += *laserCloudSurfLastDS;
        pcl::transformPointCloud(*cloudInBase, *cloudOutInWorld, T_poseInMap.matrix());
        publishCloud(&pubLaserCloudInWorld, cloudOutInWorld, timeLaserInfoStamp, "map");
    }
    //added *********************by gc

    // publish registered high-res raw cloud
    if (pubCloudRegisteredRaw.getNumSubscribers() != 0)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
        pcl::fromROSMsg(cloudInfo.cloud_deskewed, *cloudOut);
        PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
        *cloudOut = *transformPointCloud(cloudOut,  &thisPose6D);
        publishCloud(&pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp, "odom");
    }
    // publish path
    if (pubPath.getNumSubscribers() != 0)
    {
        globalPath.header.stamp = timeLaserInfoStamp;
        globalPath.header.frame_id = "odom";
        pubPath.publish(globalPath);
    }
}

/********added by gc*****/
// Todo: (1) ICP or matching point to edge and surface?  (2) global_pcd or whole keyframes
void mapOptimization::cloudGlobalLoad()
{
    pcl::io::loadPCDFile(loading_path + "/lidar/cloudGlobal.pcd", *cloudGlobalMap);

    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(cloudGlobalMap);
    downSizeFilterICP.filter(*cloud_temp);
    *cloudGlobalMapDS = *cloud_temp;

    std::cout << "the size of global cloud: " << cloudGlobalMap->points.size() << std::endl;
    std::cout << "the size of global map after filter: " << cloudGlobalMapDS->points.size() << std::endl;
}

void mapOptimization::globalLocalizeThread()
{

    //ros::Rate rate(0.2);
    while (ros::ok())
    {
        //avoid ICP using the same initial guess for many times
        if(initializedFlag == NonInitialized)
        {
            //ICPLocalizeInitialize();      // 使用NDT和ICP初始化
            relocateInitialize();           // 使用视觉激光融合重定位初始化
        } 
        else if(initializedFlag == Initializing)
        {
            std::cout << "Offer A New Guess Please." << std::endl;
            std::cout << "If you use relocation initialize, move the robot to a new place, and offer a new guess to activate relocation." << std::endl;
            //do nothing, wait for a new initial guess
            ros::Duration(1.0).sleep();
        }
        else
        {
            ros::Duration(10.0).sleep();

            double t_start = ros::Time::now().toSec();
            ICPscanMatchGlobal();
            double t_end = ros::Time::now().toSec();
            //std::cout << "ICP time consuming: " << t_end-t_start;
            
        }

        //rate.sleep();
    }
}

void mapOptimization::ICPLocalizeInitialize()
{
    pcl::PointCloud<PointType>::Ptr laserCloudIn(new pcl::PointCloud<PointType>());

    mtx_general.lock();
    *laserCloudIn += *cloudScanForInitialize;
    mtx_general.unlock();

    //publishCloud(&fortest_publasercloudINWorld, laserCloudIn, timeLaserInfoStamp, "map");

    if(laserCloudIn->points.size() == 0)
        return;
    //cloudScanForInitialize->clear();
    std::cout << "the size of incoming lasercloud: " << laserCloudIn->points.size() << std::endl;

    pcl::NormalDistributionsTransform<PointType, PointType> ndt;
    ndt.setTransformationEpsilon(0.01);
    ndt.setResolution(1.0);


    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(100);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    ndt.setInputSource(laserCloudIn);
    ndt.setInputTarget(cloudGlobalMapDS);
    pcl::PointCloud<PointType>::Ptr unused_result_0(new pcl::PointCloud<PointType>());

    // 获取ndt初值
    PointTypePose thisPose6DInWorld = trans2PointTypePose(transformInTheWorld);
    Eigen::Affine3f T_thisPose6DInWorld = pclPointToAffine3f(thisPose6DInWorld);
    ndt.align(*unused_result_0, T_thisPose6DInWorld.matrix());


    //use the outcome of ndt as the initial guess for ICP
    icp.setInputSource(laserCloudIn);
    icp.setInputTarget(cloudGlobalMapDS);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result, ndt.getFinalTransformation());

    std::cout << "the pose before initializing is: x" << transformInTheWorld[3] << " y" << transformInTheWorld[4]
                << " z" << transformInTheWorld[5] <<std::endl;
    std::cout << "the pose in odom before initializing is: x" << tranformOdomToWorld[3] << " y" << tranformOdomToWorld[4]
                << " z" << tranformOdomToWorld[5] <<std::endl;
    std::cout << "the icp score in initializing process is: " << icp.getFitnessScore() << std::endl;
    std::cout << "the pose after initializing process is: "<< icp.getFinalTransformation() << std::endl;

    // 获取里程计位姿
    PointTypePose thisPose6DInOdom = trans2PointTypePose(transformTobeMapped);
    std::cout<< "transformTobeMapped X_Y_Z: " << transformTobeMapped[3] << " " << transformTobeMapped[4] << " " << transformTobeMapped[5] << std::endl;
    Eigen::Affine3f T_thisPose6DInOdom = pclPointToAffine3f(thisPose6DInOdom);

    // 初始化地图坐标系下的位姿
    Eigen::Affine3f T_thisPose6DInMap;
    T_thisPose6DInMap = icp.getFinalTransformation();
    float x_g, y_g, z_g, R_g, P_g, Y_g;
    pcl::getTranslationAndEulerAngles (T_thisPose6DInMap, x_g, y_g, z_g, R_g, P_g, Y_g);
    transformInTheWorld[0] = R_g;
    transformInTheWorld[1] = P_g;
    transformInTheWorld[2] = Y_g;
    transformInTheWorld[3] = x_g;
    transformInTheWorld[4] = y_g;
    transformInTheWorld[5] = z_g;

    // 地图坐标系与odom坐标系的位姿变换
    Eigen::Affine3f transOdomToMap = T_thisPose6DInMap * T_thisPose6DInOdom.inverse();
    float deltax, deltay, deltaz, deltaR, deltaP, deltaY;
    pcl::getTranslationAndEulerAngles (transOdomToMap, deltax, deltay, deltaz, deltaR, deltaP, deltaY);
    mtxtranformOdomToWorld.lock();
    //renew tranformOdomToWorld
    tranformOdomToWorld[0] = deltaR;
    tranformOdomToWorld[1] = deltaP;
    tranformOdomToWorld[2] = deltaY;
    tranformOdomToWorld[3] = deltax;
    tranformOdomToWorld[4] = deltay;
    tranformOdomToWorld[5] = deltaz;
    mtxtranformOdomToWorld.unlock();
    std::cout << "the pose of odom relative to Map: x" << tranformOdomToWorld[3] << " y" << tranformOdomToWorld[4]
                << " z" << tranformOdomToWorld[5] <<std::endl;
    
    // 发布先验地图
    publishCloud(&pubLaserCloudInWorld, unused_result, timeLaserInfoStamp, "map");
    publishCloud(&pubMapWorld, cloudGlobalMapDS, timeLaserInfoStamp, "map");

    if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
    {
        initializedFlag = Initializing;
        std::cout << "Initializing Fail" << std::endl;
        return;
    } else{
        initializedFlag = Initialized;
        std::cout << "Initializing Succeed" << std::endl;
        geometry_msgs::PoseStamped pose_odomTo_map;
        tf::Quaternion q_odomTo_map = tf::createQuaternionFromRPY(deltaR, deltaP, deltaY);

        pose_odomTo_map.header.stamp = timeLaserInfoStamp;
        pose_odomTo_map.header.frame_id = "map";
        pose_odomTo_map.pose.position.x = deltax; 
        pose_odomTo_map.pose.position.y = deltay; 
        pose_odomTo_map.pose.position.z = deltaz;
        pose_odomTo_map.pose.orientation.x = q_odomTo_map.x();
        pose_odomTo_map.pose.orientation.y = q_odomTo_map.y();
        pose_odomTo_map.pose.orientation.z = q_odomTo_map.z();
        pose_odomTo_map.pose.orientation.w = q_odomTo_map.w();
        pubOdomToMapPose.publish(pose_odomTo_map);

    }

    //cloudScanForInitialize.reset(new pcl::PointCloud<PointType>());

}

void mapOptimization::relocateInitialize()
{
    // 还没开始播包
    // 和重定位有关的变量需要加锁
    mtxRelocate.lock();
    int sz = laserCloudLast->points.size();
    mtxRelocate.unlock();
    if(sz == 0)
        return;

    bool relocateSucceedFlag = true;

    std::cout << "start relocate initialization" << std::endl;

    std_msgs::String initial_msg;
    std::string initial_info;

    if (imageAvailable)
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
                relocateSucceedFlag = false;
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
            relocateSucceedFlag = false;
        }
    }

    // 3.精重定位
    if (relocateSucceedFlag && refineRelocateResult() != 0)
    {
        std::cout << "Refine rough result failure!" << std::endl;
        relocateSucceedFlag = false;
    }


    // std::cout << "the pose before initializing is: x" << transformInTheWorld[3] << " y" << transformInTheWorld[4]
    //             << " z" << transformInTheWorld[5] <<std::endl;
    // std::cout << "the pose in odom before initializing is: x" << tranformOdomToWorld[3] << " y" << tranformOdomToWorld[4]
    //             << " z" << tranformOdomToWorld[5] <<std::endl;
    // std::cout << "the icp score in initializing process is: " << icp.getFitnessScore() << std::endl;
    // std::cout << "the pose after initializing process is: "<< icp.getFinalTransformation() << std::endl;

    PointTypePose thisPose6DInOdom = trans2PointTypePose(transformTobeMapped);
    std::cout<< "transformTobeMapped X_Y_Z: " << transformTobeMapped[3] << " " << transformTobeMapped[4] << " " << transformTobeMapped[5] << std::endl;
    Eigen::Affine3f T_thisPose6DInOdom = pclPointToAffine3f(thisPose6DInOdom);

    // 初始化地图坐标系下的位姿
    Eigen::Affine3f T_thisPose6DInMap;
    T_thisPose6DInMap = lastTransformation;
    float x_g, y_g, z_g, R_g, P_g, Y_g;
    pcl::getTranslationAndEulerAngles (T_thisPose6DInMap, x_g, y_g, z_g, R_g, P_g, Y_g);
    transformInTheWorld[0] = R_g;
    transformInTheWorld[1] = P_g;
    transformInTheWorld[2] = Y_g;
    transformInTheWorld[3] = x_g;
    transformInTheWorld[4] = y_g;
    transformInTheWorld[5] = z_g;

    // 地图坐标系与odom坐标系的位姿变换
    Eigen::Affine3f transOdomToMap = T_thisPose6DInMap * T_thisPose6DInOdom.inverse();
    float deltax, deltay, deltaz, deltaR, deltaP, deltaY;
    pcl::getTranslationAndEulerAngles (transOdomToMap, deltax, deltay, deltaz, deltaR, deltaP, deltaY);
    mtxtranformOdomToWorld.lock();
    //renew tranformOdomToWorld
    tranformOdomToWorld[0] = deltaR;
    tranformOdomToWorld[1] = deltaP;
    tranformOdomToWorld[2] = deltaY;
    tranformOdomToWorld[3] = deltax;
    tranformOdomToWorld[4] = deltay;
    tranformOdomToWorld[5] = deltaz;
    mtxtranformOdomToWorld.unlock();
    std::cout << "the pose of odom relative to Map: x" << tranformOdomToWorld[3] << " y" << tranformOdomToWorld[4]
                << " z" << tranformOdomToWorld[5] <<std::endl;
    
    // 发布先验地图
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*queryCloud, *unused_result, lastTransformation.matrix());
    publishCloud(&pubLaserCloudInWorld, unused_result, timeLaserInfoStamp, "map");
    publishCloud(&pubMapWorld, cloudGlobalMapDS, timeLaserInfoStamp, "map");

    if (relocateSucceedFlag == false)
    {
        initializedFlag = Initializing;
        std::cout << "Initializing Fail" << std::endl;
        initial_info = "Relocation Fail";
        initial_msg.data = initial_info.c_str();
        pubInitialInfo.publish(initial_msg);
        return;
    } else{
        initializedFlag = Initialized;
        std::cout << "Initializing Succeed" << std::endl;
        if(relocation_method == 1)
            initial_info = "Visual Relocation";
        else if(relocation_method == 2)
            initial_info = "LiDAR Relocation";
        else
            initial_info = "Unknow";
        initial_msg.data = initial_info.c_str();
        pubInitialInfo.publish(initial_msg);

        geometry_msgs::PoseStamped pose_odomTo_map;
        tf::Quaternion q_odomTo_map = tf::createQuaternionFromRPY(deltaR, deltaP, deltaY);

        pose_odomTo_map.header.stamp = timeLaserInfoStamp;
        pose_odomTo_map.header.frame_id = "map";
        pose_odomTo_map.pose.position.x = deltax; 
        pose_odomTo_map.pose.position.y = deltay; 
        pose_odomTo_map.pose.position.z = deltaz;
        pose_odomTo_map.pose.orientation.x = q_odomTo_map.x();
        pose_odomTo_map.pose.orientation.y = q_odomTo_map.y();
        pose_odomTo_map.pose.orientation.z = q_odomTo_map.z();
        pose_odomTo_map.pose.orientation.w = q_odomTo_map.w();
        pubOdomToMapPose.publish(pose_odomTo_map);

    }

    return;
}

void mapOptimization::ICPscanMatchGlobal()
{
    if (cloudKeyPoses3D->points.empty() == true)
        return;

    //change-5
    /******************added by gc************************/

    mtxWin.lock();
    int latestFrameIDGlobalLocalize;
    latestFrameIDGlobalLocalize = win_cloudKeyPoses3D.size() - 1;

    pcl::PointCloud<PointType>::Ptr latestCloudIn(new pcl::PointCloud<PointType>());
    *latestCloudIn += *transformPointCloud(win_cornerCloudKeyFrames[latestFrameIDGlobalLocalize], &win_cloudKeyPoses6D[latestFrameIDGlobalLocalize]);
    *latestCloudIn += *transformPointCloud(win_surfCloudKeyFrames[latestFrameIDGlobalLocalize],   &win_cloudKeyPoses6D[latestFrameIDGlobalLocalize]);
    std::cout << "the size of input cloud: " << latestCloudIn->points.size() <<std::endl;
    mtxWin.unlock();

    /******************added by gc************************/

    // int latestFrameIDGlobalLocalize;
    // latestFrameIDGlobalLocalize = cloudKeyPoses3D->size() - 1;

    // //latest Frame cloudpoints In the odom Frame

    // pcl::PointCloud<PointType>::Ptr latestCloudIn(new pcl::PointCloud<PointType>());
    // *latestCloudIn += *transformPointCloud(cornerCloudKeyFrames[latestFrameIDGlobalLocalize], &cloudKeyPoses6D->points[latestFrameIDGlobalLocalize]);
    // *latestCloudIn += *transformPointCloud(surfCloudKeyFrames[latestFrameIDGlobalLocalize],   &cloudKeyPoses6D->points[latestFrameIDGlobalLocalize]);
    // std::cout << "the size of input cloud: " << latestCloudIn->points.size() <<std::endl;

    pcl::NormalDistributionsTransform<PointType, PointType> ndt;
    ndt.setTransformationEpsilon(0.01);
    ndt.setResolution(1.0);


    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(100);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // Align cloud
    //3. calculate the tranform of odom relative to world
    //Eigen::Affine3f transodomToWorld_init = pcl::getTransformation(0,0,0,0,0,0);
    mtxtranformOdomToWorld.lock();
    Eigen::Affine3f transodomToWorld_init = pcl::getTransformation(tranformOdomToWorld[3], tranformOdomToWorld[4],tranformOdomToWorld[5],tranformOdomToWorld[0],tranformOdomToWorld[1],tranformOdomToWorld[2]);
    mtxtranformOdomToWorld.unlock();

    Eigen::Matrix4f matricInitGuess = transodomToWorld_init.matrix();
    //std::cout << "matricInitGuess: " << matricInitGuess << std::endl;
    //Firstly perform ndt in coarse resolution
    ndt.setInputSource(latestCloudIn);
    ndt.setInputTarget(cloudGlobalMapDS);
    pcl::PointCloud<PointType>::Ptr unused_result_0(new pcl::PointCloud<PointType>());
    ndt.align(*unused_result_0, matricInitGuess);
    //use the outcome of ndt as the initial guess for ICP
    icp.setInputSource(latestCloudIn);
    icp.setInputTarget(cloudGlobalMapDS);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result, ndt.getFinalTransformation());

    std::cout << "amonverg flag:" << icp.hasConverged() << ". Fitness score: " << icp.getFitnessScore() << std::endl << std::endl;


    //if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
        // return;

    Eigen::Affine3f transodomToWorld_New;
    transodomToWorld_New = icp.getFinalTransformation();
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles (transodomToWorld_New, x, y, z, roll, pitch, yaw);
    //std::cout << " in test 0.03: deltaX = " << x << " deltay = " << y << " deltaZ = " << z << std::endl;

    mtxtranformOdomToWorld.lock();
    //renew tranformOdomToWorld
    tranformOdomToWorld[0] = roll;
    tranformOdomToWorld[1] = pitch;
    tranformOdomToWorld[2] = yaw;
    tranformOdomToWorld[3] = x;
    tranformOdomToWorld[4] = y;
    tranformOdomToWorld[5] = z;
    mtxtranformOdomToWorld.unlock();
    //publish the laserpointcloud in world frame

    //publish global map
    publishCloud(&pubMapWorld, cloudGlobalMapDS, timeLaserInfoStamp, "map");//publish world map

    if (icp.hasConverged() == true && icp.getFitnessScore() < historyKeyframeFitnessScore)
    {
        geometry_msgs::PoseStamped pose_odomTo_map;
        tf::Quaternion q_odomTo_map = tf::createQuaternionFromRPY(roll, pitch, yaw);

        pose_odomTo_map.header.stamp = timeLaserInfoStamp;
        pose_odomTo_map.header.frame_id = "map";
        pose_odomTo_map.pose.position.x = x; pose_odomTo_map.pose.position.y = y; pose_odomTo_map.pose.position.z = z;
        pose_odomTo_map.pose.orientation.x = q_odomTo_map.x();
        pose_odomTo_map.pose.orientation.y = q_odomTo_map.y();
        pose_odomTo_map.pose.orientation.z = q_odomTo_map.z();
        pose_odomTo_map.pose.orientation.w = q_odomTo_map.w();
        pubOdomToMapPose.publish(pose_odomTo_map);
    }
    //publish the trsformation between map and odom

}

void mapOptimization::initialpose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg)
{
    //first calculate global pose
    //x-y-z
    if(initializedFlag == Initialized)
        return;

    float x = pose_msg->pose.pose.position.x;
    float y = pose_msg->pose.pose.position.y;
    float z = pose_msg->pose.pose.position.z;

    //roll-pitch-yaw
    tf::Quaternion q_global;
    double roll_global; 
    double pitch_global; 
    double yaw_global;

    q_global.setX(pose_msg->pose.pose.orientation.x);
    q_global.setY(pose_msg->pose.pose.orientation.y);
    q_global.setZ(pose_msg->pose.pose.orientation.z);
    q_global.setW(pose_msg->pose.pose.orientation.w);

    tf::Matrix3x3(q_global).getRPY(roll_global, pitch_global, yaw_global);
    //global transformation
    transformInTheWorld[0] = roll_global;
    transformInTheWorld[1] = pitch_global;
    transformInTheWorld[2] = yaw_global;
    transformInTheWorld[3] = x;
    transformInTheWorld[4] = y;
    transformInTheWorld[5] = z;
    PointTypePose thisPose6DInWorld = trans2PointTypePose(transformInTheWorld);
    Eigen::Affine3f T_thisPose6DInWorld = pclPointToAffine3f(thisPose6DInWorld);
    //Odom transformation
    PointTypePose thisPose6DInOdom = trans2PointTypePose(transformTobeMapped);
    Eigen::Affine3f T_thisPose6DInOdom = pclPointToAffine3f(thisPose6DInOdom);
    //transformation: Odom to Map
    Eigen::Affine3f T_OdomToMap = T_thisPose6DInWorld * T_thisPose6DInOdom.inverse();
    float delta_x, delta_y, delta_z, delta_roll, delta_pitch, delta_yaw;
    pcl::getTranslationAndEulerAngles (T_OdomToMap, delta_x, delta_y, delta_z, delta_roll, delta_pitch, delta_yaw);

    mtxtranformOdomToWorld.lock();
    //keep for co-operate the initializing and lio, not useful for the present
    tranformOdomToWorld[0] = delta_roll;
    tranformOdomToWorld[1] = delta_pitch;
    tranformOdomToWorld[2] = delta_yaw;
    tranformOdomToWorld[3] = delta_x;
    tranformOdomToWorld[4] = delta_y;
    tranformOdomToWorld[5] = delta_z;

    mtxtranformOdomToWorld.unlock();
    initializedFlag = NonInitialized;
}

/*********************** multimap function *******************************/
int mapOptimization::loadFolderMap()
{
    TicToc t_load(true);
    map = new MultiMap();
    map->loadMultiMap();
    t_load.toc("Load Floder Map total use time");
    return 0;
}

int mapOptimization::saveBinaryMap()
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

int mapOptimization::loadBinaryMap()
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
    cout << "Load Binary Map Finsh! Please play rosbag!" << std::endl;

    return 0;
}

/*********************** visual relocation *******************************/
int mapOptimization::visualRelocate()
{
    // 1.词袋重定位
    int loop_index = detectLoop(queryPicture, queryPicture->index);
    if (loop_index != -1)
    {
        KeyFrame *old_kf = getKeyFrame(loop_index);

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
            tmp_relocation_YPR <<   tmp_relocation_pose.yaw / M_PI * 180,
                                    tmp_relocation_pose.pitch / M_PI * 180,
                                    tmp_relocation_pose.roll / M_PI * 180;
            tmp_relocation_R = Utility::ypr2R(tmp_relocation_YPR);

            // 5.视觉粗重定位成功，输出结果
            ROS_INFO("Visual rough relocation success!");
            relocation_method = 1;
            std::cout << "T is: " << tmp_relocation_T.transpose() << std::endl;
            std::cout << "YPR is: " << tmp_relocation_YPR.transpose() << std::endl;
            // std::cout << "image time stamp: " << fixed << setprecision(5) << image_time << std::endl; 
            // std::cout << "lidar time stamp: " << fixed << setprecision(5) << cloud_time << std::endl; 

            int gap = 10;
	        cv::Mat gap_image(ROW, gap, CV_8UC1, cv::Scalar(255, 255, 255));
            cv::Mat query_img = queryPicture->image;
            cv::Mat old_img = old_kf->image;
            cv::Mat show_img;
            cv::hconcat(query_img, gap_image, gap_image);
            cv::hconcat(gap_image, old_img, show_img);
            cv::Mat notation(50, COL + gap + COL, CV_8UC1, cv::Scalar(255));
	        putText(notation, "current picture" , cv::Point2f(20, 30), CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0), 3);
            putText(notation, "map picture", cv::Point2f(20 + COL + gap, 30), CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0), 3);
	        cv::vconcat(notation, show_img, show_img);

            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", show_img).toImageMsg();
            pubMatchImg.publish(msg);
            return 0;
        }
    }
    ROS_INFO("Visual rough relocation failure!");
    return -1;
}

// relocate_test = true:测试重定位模块，图片从路径加载
// relocate_test = false:正常运行
int mapOptimization::loadQueryPicture(bool relocate_test)
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
    else {
        mtxRelocate.lock();
        image = currentPicture.clone();
        mtxRelocate.unlock();
    }

    if (image.empty())
    {
        std::cout << "Load the query picture from: " << QUERY_IMAGE_PATH << " failed!" << std::endl;
        return -1;
    }

    queryPicture = new KeyFrame(time_stamp, frame_index, T, R,
                                image, point_3d, point_2d_uv, point_2d_normal, point_id, sequence);
    std::cout << "Load the query picture success!" << std::endl;
    return 0;
}

int mapOptimization::detectLoop(KeyFrame *keyframe, int frame_index)
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

KeyFrame *mapOptimization::getKeyFrame(int index)
{
    // unique_lock<mutex> lock(m_keyframelist);
    list<KeyFrame*>::iterator it = map->keyframelist.begin();
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

int mapOptimization::lidarRelocate()
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
        relocation_method = 2;

        // 3.得到粗位姿，以找到的lidar帧作为粗位姿，并在yaw上做一点处理
        tmp_relocation_pose = map->cloudKeyPoses6D->points[loopKeyPre];
        tmp_relocation_pose.yaw = tmp_relocation_pose.yaw - yaw_diff_rad;
        relocation_lidar_index = loopKeyPre;
        tmp_relocation_T(0) = tmp_relocation_pose.x;
        tmp_relocation_T(1) = tmp_relocation_pose.y;
        tmp_relocation_T(2) = tmp_relocation_pose.z;
        Eigen::Vector3d tmp_relocation_YPR;
        tmp_relocation_YPR <<   tmp_relocation_pose.yaw / M_PI * 180,
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

int mapOptimization::loadQueryCloud(bool relocate_test)
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
        mtxRelocate.lock();
        *queryCloud = *laserCloudLast;
        mtxRelocate.unlock();
    }

    map->scManager.makeAndSaveScancontextAndKeys(*queryCloud);

    return 0;
}

void mapOptimization::loopFindNearKeyframesByIndex(pcl::PointCloud<PointType>::Ptr &nearKeyframes, const int &key, const int &searchNum)
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

void mapOptimization::loopFindNearKeyframesByPose(pcl::PointCloud<PointType>::Ptr &nearKeyframes, const PointType &pose, const int &searchNum)
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

int mapOptimization::refineRelocateResult()
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
        std::cout << "YPR is: " << tmp_YPR.transpose() << std::endl;

// 可视化
#ifdef SHOW_RELOCATE
        showLidarResult(prevKeyframeCloud, cureKeyframeCloud, "before ndt refine");
        showLidarResult(prevKeyframeCloud, result, "after ndt refine");
#endif
    }

    return 0;
}

