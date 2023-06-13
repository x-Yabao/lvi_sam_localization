#include "featureExtraction.h"

FeatureExtraction::FeatureExtraction()
{
    subLaserCloudInfo = nh.subscribe<lvi_sam_localization::cloud_info>("lio_sam/deskew/cloud_info", 1, &FeatureExtraction::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());

    pubLaserCloudInfo = nh.advertise<lvi_sam_localization::cloud_info> ("lio_sam/feature/cloud_info", 1);
    pubCornerPoints = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_corner", 1);
    pubSurfacePoints = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_surface", 1);
    
    initializationValue();
}

void FeatureExtraction::initializationValue()
{
    cloudSmoothness.resize(N_SCAN*Horizon_SCAN);

    downSizeFilter.setLeafSize(odometrySurfLeafSize, odometrySurfLeafSize, odometrySurfLeafSize);

    extractedCloud.reset(new pcl::PointCloud<PointType>());
    cornerCloud.reset(new pcl::PointCloud<PointType>());
    surfaceCloud.reset(new pcl::PointCloud<PointType>());

    cloudCurvature = new float[N_SCAN*Horizon_SCAN];
    cloudNeighborPicked = new int[N_SCAN*Horizon_SCAN];
    cloudLabel = new int[N_SCAN*Horizon_SCAN];
}

// 订阅上一个结点的消息
void FeatureExtraction::laserCloudInfoHandler(const lvi_sam_localization::cloud_infoConstPtr& msgIn)
{
    cloudInfo = *msgIn;             // new cloud info
    cloudHeader = msgIn->header;    // new cloud header
    // 把提取出来的有效的点转成pcl的格式
    pcl::fromROSMsg(msgIn->cloud_deskewed, *extractedCloud); // new cloud for extraction

    calculateSmoothness();

    markOccludedPoints();

    extractFeatures();

    publishFeatureCloud();
}
// 计算曲率
void FeatureExtraction::calculateSmoothness()
{
    int cloudSize = extractedCloud->points.size();
    for (int i = 5; i < cloudSize - 5; i++)
    {
        // 计算当前点和周围十个点的距离差
        float diffRange = cloudInfo.pointRange[i-5] + cloudInfo.pointRange[i-4]
                        + cloudInfo.pointRange[i-3] + cloudInfo.pointRange[i-2]
                        + cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i] * 10
                        + cloudInfo.pointRange[i+1] + cloudInfo.pointRange[i+2]
                        + cloudInfo.pointRange[i+3] + cloudInfo.pointRange[i+4]
                        + cloudInfo.pointRange[i+5];            

        cloudCurvature[i] = diffRange * diffRange;    //diffX * diffX + diffY * diffY + diffZ * diffZ;
        // 下面两个值赋成初始值
        cloudNeighborPicked[i] = 0;
        cloudLabel[i] = 0;
        // cloudSmoothness for sorting
        // 用来进行曲率排序
        cloudSmoothness[i].value = cloudCurvature[i];
        cloudSmoothness[i].ind = i;
    }
}
// 标记一下遮挡的点
void FeatureExtraction::markOccludedPoints()
{
    int cloudSize = extractedCloud->points.size();
    // mark occluded points and parallel beam points
    for (int i = 5; i < cloudSize - 6; ++i)
    {
        // occluded points
        // 取出相邻两个点距离信息
        float depth1 = cloudInfo.pointRange[i];
        float depth2 = cloudInfo.pointRange[i+1];
        // 计算两个有效点之间的列id差
        int columnDiff = std::abs(int(cloudInfo.pointColInd[i+1] - cloudInfo.pointColInd[i]));
        // 只有比较靠近才有意义
        if (columnDiff < 10){
            // 10 pixel diff in range image
            // 这样depth1容易被遮挡，因此其之前的5个点走设置为无效点
            if (depth1 - depth2 > 0.3){
                cloudNeighborPicked[i - 5] = 1;
                cloudNeighborPicked[i - 4] = 1;
                cloudNeighborPicked[i - 3] = 1;
                cloudNeighborPicked[i - 2] = 1;
                cloudNeighborPicked[i - 1] = 1;
                cloudNeighborPicked[i] = 1;
            }else if (depth2 - depth1 > 0.3){   // 这里同理
                cloudNeighborPicked[i + 1] = 1;
                cloudNeighborPicked[i + 2] = 1;
                cloudNeighborPicked[i + 3] = 1;
                cloudNeighborPicked[i + 4] = 1;
                cloudNeighborPicked[i + 5] = 1;
                cloudNeighborPicked[i + 6] = 1;
            }
        }
        // parallel beam
        float diff1 = std::abs(float(cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i]));
        float diff2 = std::abs(float(cloudInfo.pointRange[i+1] - cloudInfo.pointRange[i]));
        // 如果两点距离比较大 就很可能是平行的点，也很可能失去观测
        if (diff1 > 0.02 * cloudInfo.pointRange[i] && diff2 > 0.02 * cloudInfo.pointRange[i])
            cloudNeighborPicked[i] = 1;
    }
}
// 提取特征
void FeatureExtraction::extractFeatures()
{
    cornerCloud->clear();
    surfaceCloud->clear();

    pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointType>());

    for (int i = 0; i < N_SCAN; i++)
    {
        surfaceCloudScan->clear();
        // 把每一根scan等分成6份，每份分别提取特征点
        for (int j = 0; j < 6; j++)
        {
            // 根据之前得到的每个scan的起始和结束id来均分
            int sp = (cloudInfo.startRingIndex[i] * (6 - j) + cloudInfo.endRingIndex[i] * j) / 6;
            int ep = (cloudInfo.startRingIndex[i] * (5 - j) + cloudInfo.endRingIndex[i] * (j + 1)) / 6 - 1;
            // 这种情况就不正常
            if (sp >= ep)
                continue;
            // 按照曲率排序
            std::sort(cloudSmoothness.begin()+sp, cloudSmoothness.begin()+ep, by_value());

            // 收集角点
            int largestPickedNum = 0;
            for (int k = ep; k >= sp; k--)
            {
                // 找到这个点对应的原先的idx
                int ind = cloudSmoothness[k].ind;
                // 如果没有被认为是遮挡点 且曲率大于边缘点门限
                if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > edgeThreshold)
                {
                    largestPickedNum++;
                    // 每段最多找20个角点
                    if (largestPickedNum <= 20){
                        // 标签置1表示是角点
                        cloudLabel[ind] = 1;
                        // 这个点收集进存储角点的点云中
                        cornerCloud->push_back(extractedCloud->points[ind]);
                    } else {
                        break;
                    }
                    // 将这个点周围的几个点设置成遮挡点，避免选取太集中
                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++)
                    {
                        int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                        // 列idx距离太远就算了，空间上也不会太集中
                        if (columnDiff > 10)
                            break;
                        cloudNeighborPicked[ind + l] = 1;
                    }
                    // 同理
                    for (int l = -1; l >= -5; l--)
                    {
                        int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                        if (columnDiff > 10)
                            break;
                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }
            // 开始收集面点
            for (int k = sp; k <= ep; k++)
            {
                int ind = cloudSmoothness[k].ind;
                // 同样要求不是遮挡点且曲率小于给定阈值
                if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < surfThreshold)
                {
                    // -1表示是面点
                    cloudLabel[ind] = -1;
                    // 同理 把周围的点都设置为遮挡点
                    cloudNeighborPicked[ind] = 1;

                    for (int l = 1; l <= 5; l++) {

                        int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                        if (columnDiff > 10)
                            break;

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--) {

                        int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                        if (columnDiff > 10)
                            break;

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++)
            {
                // 注意这里是小于等于0,也就是说不是角点的都认为是面点了
                if (cloudLabel[k] <= 0){
                    surfaceCloudScan->push_back(extractedCloud->points[k]);
                }
            }
        }

        surfaceCloudScanDS->clear();
        // 因为面点太多了，所以做一个下采样
        downSizeFilter.setInputCloud(surfaceCloudScan);
        downSizeFilter.filter(*surfaceCloudScanDS);

        *surfaceCloud += *surfaceCloudScanDS;
    }
}
// 将一些不会用到的存储空间释放掉
void FeatureExtraction::freeCloudInfoMemory()
{
    cloudInfo.startRingIndex.clear();
    cloudInfo.endRingIndex.clear();
    cloudInfo.pointColInd.clear();
    cloudInfo.pointRange.clear();
}

void FeatureExtraction::publishFeatureCloud()
{
    // free cloud info memory
    freeCloudInfoMemory();
    // save newly extracted features
    // 把角点和面点发送给后端
    cloudInfo.cloud_corner  = publishCloud(&pubCornerPoints,  cornerCloud,  cloudHeader.stamp, lidarFrame);
    cloudInfo.cloud_surface = publishCloud(&pubSurfacePoints, surfaceCloud, cloudHeader.stamp, lidarFrame);
    // publish to mapOptimization
    pubLaserCloudInfo.publish(cloudInfo);
}