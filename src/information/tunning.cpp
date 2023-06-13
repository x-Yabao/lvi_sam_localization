#include "tunning.h"

Tunning::Tunning()
{
    subEnvDensityInfo = nh.subscribe<std_msgs::String>("info/env_density", 10, &Tunning::envDensityCallback, this, ros::TransportHints().tcpNoDelay());
}

// 建筑密集时，局部地图小一点；建筑稀疏时，局部地图大一点
void Tunning::envDensityCallback(const std_msgs::String::ConstPtr& msg)
{
    static int open_cnt = 0;
    static int sparse_cnt = 0;
    static int dense_cnt = 0;

    // ROS_INFO("I heard: [%s]", msg->data.c_str());
    std::string env_density = msg->data.c_str();
    float tmp_surroundingKeyframeSearchRadius;

    if (env_density == "open") {
        tmp_surroundingKeyframeSearchRadius = 50;
        open_cnt++;
    }
    else if (env_density == "sparse") {
        tmp_surroundingKeyframeSearchRadius = 50;
        sparse_cnt++;
    }
    else if (env_density == "dense") {
        tmp_surroundingKeyframeSearchRadius = 50;
        dense_cnt++;
    }
    else {
        tmp_surroundingKeyframeSearchRadius = 50;
    }     

    nh.setParam("surroundingKeyframeSearchRadius", tmp_surroundingKeyframeSearchRadius);

    // ROS_INFO("surroundingKeyframeSearchRadius now is: %f", tmp_surroundingKeyframeSearchRadius);
    ROS_INFO("open_cnt: %d, sparse_cnt: %d, dense_cnt: %d", open_cnt, sparse_cnt, dense_cnt);
}




