#include "imageProjection.h"

ImageProjection::ImageProjection():deskewFlag(0)
{
    // 订阅imu数据，后端里程记数据，原始点云数据
    subImage      = nh.subscribe<sensor_msgs::Image>(image_topic, 20, &ImageProjection::imageHandler, this, ros::TransportHints().tcpNoDelay());
    subImu        = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &ImageProjection::imuHandler, this, ros::TransportHints().tcpNoDelay());
    subOdom       = nh.subscribe<nav_msgs::Odometry>(odomTopic, 2000, &ImageProjection::odometryHandler, this, ros::TransportHints().tcpNoDelay());
    subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 5, &ImageProjection::cloudHandler, this, ros::TransportHints().tcpNoDelay());

    pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/deskew/cloud_deskewed", 1);      // 发布去畸变后的点云
    pubLaserCloudInfo = nh.advertise<lvi_sam_localization::cloud_info>("lio_sam/deskew/cloud_info", 1);
    pubEnvDensityInfo = nh.advertise<std_msgs::String>("info/env_density", 5);
    pubEnvDensityCloud = nh.advertise<sensor_msgs::PointCloud2>("info/env_density_cloud", 1);

    allocateMemory();
    resetParameters();

    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
}

void ImageProjection::allocateMemory()
{
    laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
    // tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());
    fullCloud.reset(new pcl::PointCloud<PointType>());
    extractedCloud.reset(new pcl::PointCloud<PointType>());
    judgeCloud.reset(new pcl::PointCloud<PointType>()); 

    fullCloud->points.resize(N_SCAN*Horizon_SCAN);

    cloudInfo.startRingIndex.assign(N_SCAN, 0);
    cloudInfo.endRingIndex.assign(N_SCAN, 0);

    cloudInfo.pointColInd.assign(N_SCAN*Horizon_SCAN, 0);
    cloudInfo.pointRange.assign(N_SCAN*Horizon_SCAN, 0);

    resetParameters();
}

void ImageProjection::resetParameters()
{
    laserCloudIn->clear();
    extractedCloud->clear();
    judgeCloud->clear();
    // reset range matrix for range image projection
    rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

    imuPointerCur = 0;
    firstPointFlag = true;
    odomDeskewFlag = false;

    for (int i = 0; i < queueLength; ++i)
    {
        imuTime[i] = 0;
        imuRotX[i] = 0;
        imuRotY[i] = 0;
        imuRotZ[i] = 0;
    }
}

void ImageProjection::imageHandler(const sensor_msgs::ImageConstPtr &imageMsg)
{
    std::lock_guard<std::mutex> lock3(imageLock);
    imageQueue.push_back(*imageMsg);

}

void ImageProjection::imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
{
    sensor_msgs::Imu thisImu = imuConverter(*imuMsg);   // 对imu做一个坐标转换
    // 加一个线程锁，把imu数据保存进队列
    std::lock_guard<std::mutex> lock1(imuLock);
    imuQueue.push_back(thisImu);

    // debug IMU data
    // cout << std::setprecision(6);
    // cout << "IMU acc: " << endl;
    // cout << "x: " << thisImu.linear_acceleration.x << 
    //       ", y: " << thisImu.linear_acceleration.y << 
    //       ", z: " << thisImu.linear_acceleration.z << endl;
    // cout << "IMU gyro: " << endl;
    // cout << "x: " << thisImu.angular_velocity.x << 
    //       ", y: " << thisImu.angular_velocity.y << 
    //       ", z: " << thisImu.angular_velocity.z << endl;
    // double imuRoll, imuPitch, imuYaw;
    // tf::Quaternion orientation;
    // tf::quaternionMsgToTF(thisImu.orientation, orientation);
    // tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
    // cout << "IMU roll pitch yaw: " << endl;
    // cout << "roll: " << imuRoll << ", pitch: " << imuPitch << ", yaw: " << imuYaw << endl << endl;
}

void ImageProjection::odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg)
{
    std::lock_guard<std::mutex> lock2(odoLock);
    odomQueue.push_back(*odometryMsg);
}

void ImageProjection::cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
    if (!cachePointCloud(laserCloudMsg))
        return;

    if (!deskewInfo())
        return;

    getSyncImage();

    projectPointCloud();

    cloudExtraction();

    publishClouds();

    judgeEnvironmentStatus();

    resetParameters();
}

bool ImageProjection::cachePointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
    // cache point cloud
    // 点云数据保存进队列
    cloudQueue.push_back(*laserCloudMsg);
    // 确保队列里大于两帧点云数据
    if (cloudQueue.size() <= 2)
        return false;

    // 缓存了足够多的点云之后
    // convert cloud
    currentCloudMsg = std::move(cloudQueue.front());
    cloudQueue.pop_front();
    pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);    // 转成pcl的点云格式
    // if (sensor == SensorType::VELODYNE)
    // {
    //     pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);    // 转成pcl的点云格式
    // }
    // else if (sensor == SensorType::OUSTER)
    // {
    //     // Convert to Velodyne format
    //     pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
    //     laserCloudIn->points.resize(tmpOusterCloudIn->size());
    //     laserCloudIn->is_dense = tmpOusterCloudIn->is_dense;
    //     for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
    //     {
    //         auto &src = tmpOusterCloudIn->points[i];
    //         auto &dst = laserCloudIn->points[i];
    //         dst.x = src.x;
    //         dst.y = src.y;
    //         dst.z = src.z;
    //         dst.intensity = src.intensity;
    //         dst.ring = src.ring;
    //         dst.time = src.t * 1e-9f;
    //     }
    // }
    // else
    // {
    //     ROS_ERROR_STREAM("Unknown sensor type: " << int(sensor));
    //     ros::shutdown();
    // }

    // get timestamp
    cloudHeader = currentCloudMsg.header;
    timeScanCur = cloudHeader.stamp.toSec();
    timeScanEnd = timeScanCur + laserCloudIn->points.back().time;

    // check dense flag
    // is_dense是点云是否有序排列的标志
    if (laserCloudIn->is_dense == false)
    {
        ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
        ros::shutdown();
    }

    // check ring channel
    // 查看驱动里是否把每个点属于哪一根扫描scan这个信息
    static int ringFlag = 0;
    if (ringFlag == 0)
    {
        ringFlag = -1;
        for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
        {
            if (currentCloudMsg.fields[i].name == "ring")
            {
                ringFlag = 1;
                break;
            }
        }
        // 如果没有这个信息就需要像loam或者lego loam那样手动计算scan id，现在velodyne的驱动里都会携带这些信息的
        if (ringFlag == -1)
        {
            ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
            ros::shutdown();
        }
    }

    // check point time
    // 同样，检查是否有时间戳信息，若没有时间戳信息，无法进行运动补偿
    if (deskewFlag == 0)
    {
        deskewFlag = -1;
        for (auto &field : currentCloudMsg.fields)
        {
            if (field.name == "time" || field.name == "t")
            {
                deskewFlag = 1;
                break;
            }
        }
        if (deskewFlag == -1)
            ROS_WARN("Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
    }

    return true;
}

// 获取运动补偿所需的信息
bool ImageProjection::deskewInfo()
{
    std::lock_guard<std::mutex> lock1(imuLock);
    std::lock_guard<std::mutex> lock2(odoLock);

    // make sure IMU data available for the scan
    // 确保imu的数据覆盖这一帧的点云
    if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur || imuQueue.back().header.stamp.toSec() < timeScanEnd)
    {
        ROS_DEBUG("Waiting for IMU data ...");
        return false;
    }
    // 准备imu补偿的信息
    imuDeskewInfo();

    odomDeskewInfo();

    return true;
}

void ImageProjection::imuDeskewInfo()
{
    cloudInfo.imuAvailable = false;

    while (!imuQueue.empty())
    {
        if (imuQueue.front().header.stamp.toSec() < timeScanCur - 0.01) // 扔掉把过早的imu
            imuQueue.pop_front();
        else
            break;
    }

    if (imuQueue.empty())
        return;

    imuPointerCur = 0;

    for (int i = 0; i < (int)imuQueue.size(); ++i)
    {
        sensor_msgs::Imu thisImuMsg = imuQueue[i];
        double currentImuTime = thisImuMsg.header.stamp.toSec();

        // get roll, pitch, and yaw estimation for this scan
        if (currentImuTime <= timeScanCur)
            // 把imu的姿态转成欧拉角
            imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);

        if (currentImuTime > timeScanEnd + 0.01)    // 这一帧遍历完了就break
            break;

        if (imuPointerCur == 0){    // 起始帧
            imuRotX[0] = 0;
            imuRotY[0] = 0;
            imuRotZ[0] = 0;
            imuTime[0] = currentImuTime;
            ++imuPointerCur;
            continue;
        }

        // get angular velocity
        double angular_x, angular_y, angular_z;
        // 取出当前帧的角速度
        imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

        // integrate rotation
        double timeDiff = currentImuTime - imuTime[imuPointerCur-1];
        // 计算每一个时刻的姿态角（相对于起始点），方便后续查找对应每个点云时间的值
        imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + angular_x * timeDiff;
        imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + angular_y * timeDiff;
        imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + angular_z * timeDiff;
        imuTime[imuPointerCur] = currentImuTime;
        ++imuPointerCur;
    }

    --imuPointerCur;

    if (imuPointerCur <= 0)
        return;
    // 可以使用imu数据进行运动补偿
    cloudInfo.imuAvailable = true;
}

void ImageProjection::odomDeskewInfo()
{
    cloudInfo.odomAvailable = false;

    while (!odomQueue.empty())
    {
        // 扔掉过早的数据
        if (odomQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
            odomQueue.pop_front();
        else
            break;
    }

    if (odomQueue.empty())
        return;
    // 点云时间   ×××××××
    // odom时间     ×××××
    // 显然不能覆盖整个点云的时间
    if (odomQueue.front().header.stamp.toSec() > timeScanCur)
        return;

    // get start odometry at the beinning of the scan
    nav_msgs::Odometry startOdomMsg;
    // 找到对应的最早的点云时间的odom数据
    for (int i = 0; i < (int)odomQueue.size(); ++i)
    {
        startOdomMsg = odomQueue[i];

        if (ROS_TIME(&startOdomMsg) < timeScanCur)
            continue;
        else
            break;
    }
    // 将ros消息格式中的姿态转成tf的格式
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);
    // 然后将四元数转成欧拉角
    double roll, pitch, yaw;
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    // 记录点云起始时刻的对应的odom姿态
    // Initial guess used in mapOptimization
    cloudInfo.initialGuessX = startOdomMsg.pose.pose.position.x;
    cloudInfo.initialGuessY = startOdomMsg.pose.pose.position.y;
    cloudInfo.initialGuessZ = startOdomMsg.pose.pose.position.z;
    cloudInfo.initialGuessRoll  = roll;
    cloudInfo.initialGuessPitch = pitch;
    cloudInfo.initialGuessYaw   = yaw;
    cloudInfo.imuPreintegrationResetId = round(startOdomMsg.pose.covariance[0]);    // add gc

    cloudInfo.odomAvailable = true; // odom提供了这一帧点云的初始位姿

    // get end odometry at the end of the scan
    odomDeskewFlag = false;
    // 这里发现没有覆盖到最后的点云，那就不能用odom数据来做运动补偿
    if (odomQueue.back().header.stamp.toSec() < timeScanEnd)
        return;

    nav_msgs::Odometry endOdomMsg;
    // 找到点云最晚时间对应的odom数据
    for (int i = 0; i < (int)odomQueue.size(); ++i)
    {
        endOdomMsg = odomQueue[i];

        if (ROS_TIME(&endOdomMsg) < timeScanEnd)
            continue;
        else
            break;
    }
    // 这个代表odom退化了，就置信度不高了
    if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
        return;
    // 起始位姿和结束位姿都转成Affine3f这个数据结构
    Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

    tf::quaternionMsgToTF(endOdomMsg.pose.pose.orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);
    // 计算起始位姿和结束位姿之间的delta pose
    Eigen::Affine3f transBt = transBegin.inverse() * transEnd;
    // 将这个增量转成xyz和欧拉角的形式
    float rollIncre, pitchIncre, yawIncre;
    pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

    odomDeskewFlag = true;  // 表示可以用odom来做运动补偿
}

void ImageProjection::findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
{
    *rotXCur = 0; *rotYCur = 0; *rotZCur = 0;

    int imuPointerFront = 0;
    // imuPointerCur是imu计算的旋转buffer的总共大小，这里用的就是一种朴素的确保不越界的方法
    while (imuPointerFront < imuPointerCur)
    {
        if (pointTime < imuTime[imuPointerFront])
            break;
        ++imuPointerFront;
    }

    // imuPointerBack     imuPointerFront
    //    ×                      ×
    //               ×
    //           imuPointerCur
    
    // 如果时间戳不在两个imu的旋转之间，就直接赋值了
    if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
    {
        *rotXCur = imuRotX[imuPointerFront];
        *rotYCur = imuRotY[imuPointerFront];
        *rotZCur = imuRotZ[imuPointerFront];
    } else {
        // 否则 做一个线性插值，得到相对旋转
        int imuPointerBack = imuPointerFront - 1;
        double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
        double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
        *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
        *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
        *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
    }
}

void ImageProjection::findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)
{
    *posXCur = 0; *posYCur = 0; *posZCur = 0;

    // If the sensor moves relatively slow, like walking speed, positional deskew seems to have little benefits. Thus code below is commented.

    // if (cloudInfo.odomAvailable == false || odomDeskewFlag == false)
    //     return;

    // float ratio = relTime / (timeScanEnd - timeScanCur);

    // *posXCur = ratio * odomIncreX;
    // *posYCur = ratio * odomIncreY;
    // *posZCur = ratio * odomIncreZ;
}

PointType ImageProjection::deskewPoint(PointType *point, double relTime)
{
    if (deskewFlag == -1 || cloudInfo.imuAvailable == false)
        return *point;
    // relTime是相对时间，加上起始时间就是绝对时间
    double pointTime = timeScanCur + relTime;

    float rotXCur, rotYCur, rotZCur;
    // 计算当前点相对起始点的相对旋转（使用的是imu的信息）
    findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);
    float posXCur, posYCur, posZCur;
    // 这里没有计算平移补偿（会用的是imu里程计的信息）
    findPosition(relTime, &posXCur, &posYCur, &posZCur);

    if (firstPointFlag == true)
    {
        // 计算第一个点的相对位姿
        transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
        firstPointFlag = false;
    }

    // 计算当前点和第一个点的相对位姿
    // transform points to start
    Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
    Eigen::Affine3f transBt = transStartInverse * transFinal;

    PointType newPoint;
    // 就是R × p + t，把点补偿到第一个点对应时刻的位姿
    newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
    newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
    newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
    newPoint.intensity = point->intensity;

    return newPoint;
}

bool ImageProjection::getSyncImage()
{
    std::lock_guard<std::mutex> lock3(imageLock);

    while(!imageQueue.empty()){
        if(imageQueue.front().header.stamp.toSec() < timeScanCur - 0.08) // 扔掉把过早的image
            imageQueue.pop_front();
        else
            break;
    }

    if(imageQueue.empty() || imageQueue.front().header.stamp.toSec() > timeScanCur + 0.08){
        // ROS_WARN("Sync image failure!");
        cloudInfo.imageAvailable = false;
        return false;
    }
    
    cloudInfo.image = imageQueue.front(); 
    cloudInfo.imageAvailable = true;
    // std::cout << "Sync image success! Abs(pointTime - imageTime) = " << 
    // std::abs(timeScanCur - imageQueue.front().header.stamp.toSec()) << std::endl;
    return true;

}

// 将点云投影到一个矩阵上，并且保存每个点的信息
void ImageProjection::projectPointCloud()
{
    int cloudSize = laserCloudIn->points.size();
    // range image projection
    for (int i = 0; i < cloudSize; ++i)
    {
        PointType thisPoint;
        // 取出对应的某个点
        thisPoint.x = laserCloudIn->points[i].x;
        thisPoint.y = laserCloudIn->points[i].y;
        thisPoint.z = laserCloudIn->points[i].z;
        thisPoint.intensity = laserCloudIn->points[i].intensity;
        // 计算这个点距离lidar中心的距离
        float range = pointDistance(thisPoint);
        // 距离太小或者太远都认为是异常点
        if (range < lidarMinRange || range > lidarMaxRange)
            continue;
        // 取出对应的在第几根scan上
        int rowIdn = laserCloudIn->points[i].ring;
        // scan id必须合理
        if (rowIdn < 0 || rowIdn >= N_SCAN)
            continue;
        // 如果需要降采样，就根据scan id适当跳过
        if (rowIdn % downsampleRate != 0)
            continue;
        // 计算水平角
        float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

        static float ang_res_x = 360.0/float(Horizon_SCAN);
        // 计算水平线束id，转换到x负方向e为起始，顺时针为正方向，范围[0,H]
        int columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
        if (columnIdn >= Horizon_SCAN)
            columnIdn -= Horizon_SCAN;
        // 对水平id进行检查
        if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
            continue;
        // 如果这个位置已经有填充了就跳过
        if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
            continue;
        // 对点做运动补偿
        thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);
        // 将这个点的距离数据保存进这个range矩阵中
        rangeMat.at<float>(rowIdn, columnIdn) = range;
        // 算出这个点的索引
        int index = columnIdn + rowIdn * Horizon_SCAN;
        // 保存这个点的坐标
        fullCloud->points[index] = thisPoint;

        // yabao, 判断环境密度
        int midIdn = N_SCAN / 2;
        if(rowIdn == midIdn + 2 || rowIdn == midIdn + 4) {
            judgeCloud->push_back(thisPoint);
        }
    }
}

// 提取出有效的点的信息
void ImageProjection::cloudExtraction()
{
    int count = 0;
    // extract segmented cloud for lidar odometry
    // 遍历每一根scan
    for (int i = 0; i < N_SCAN; ++i)
    {
        // 这个scan可以计算曲率的起始点（计算曲率需要左右各五个点）
        cloudInfo.startRingIndex[i] = count - 1 + 5;

        for (int j = 0; j < Horizon_SCAN; ++j)
        {
            if (rangeMat.at<float>(i,j) != FLT_MAX)
            {
                // 这是一个有用的点
                // mark the points' column index for marking occlusion later
                // 这个点对应着哪一根垂直线
                cloudInfo.pointColInd[count] = j;
                // save range info
                // 他的距离信息
                cloudInfo.pointRange[count] = rangeMat.at<float>(i,j);
                // save extracted cloud
                // 他的3d坐标信息
                extractedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                // size of extracted cloud
                // count只在有效点才会累加
                ++count;
            }
        }
        // 这个scan可以计算曲率的终点
        cloudInfo.endRingIndex[i] = count -1 - 5;
    }
}

void ImageProjection::publishClouds()
{
    cloudInfo.header = cloudHeader;
    // 发布提取出来的有效的点
    cloudInfo.cloud_deskewed  = publishCloud(&pubExtractedCloud, extractedCloud, cloudHeader.stamp, lidarFrame);
    pubLaserCloudInfo.publish(cloudInfo);
}

void ImageProjection::judgeEnvironmentStatus()
{
    int sz = judgeCloud->size();
    std::vector<float> distances(sz);
    for(int i = 0; i < sz; i++) {
        float x = judgeCloud->points[i].x;
        float y = judgeCloud->points[i].y; 
        float z = judgeCloud->points[i].z;
        distances[i] = x*x + y*y + z*z;
    }
    sort(distances.begin(), distances.end());

    if(distances[DENSE_RATIO * sz] < DENSE_DIS * DENSE_DIS)
        envDensity = "dense";
    else if(distances[(1.0 - OPEN_RATIO) * sz] > OPEN_DIS * OPEN_DIS)
        envDensity = "open";
    else
        envDensity = "sparse";

    std_msgs::String msg;
    msg.data = envDensity.c_str();
    pubEnvDensityInfo.publish(msg);
    publishCloud(&pubEnvDensityCloud, judgeCloud, cloudHeader.stamp, lidarFrame);
    // ROS_INFO("Environment density: %s", msg.data.c_str());

    return;
}