#include "tunning.h"

Tunning::Tunning()
{
    subEnvDensityInfo = nh.subscribe<std_msgs::String>("info/env_density", 1, &Tunning::envDensityCallback, this, ros::TransportHints().tcpNoDelay());
    envDensity = "unknown";
}

void Tunning::envDensityCallback(const std_msgs::String::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mtx);
    envDensity = msg->data.c_str();
}

// 建筑密集时，局部地图小一点；建筑稀疏时，局部地图大一点
void Tunning::tunningRule_1()
{
    std::lock_guard<std::mutex> lock(mtx);

    float tmp_surroundingKeyframeSearchRadius;

    if (envDensity == "open") {
        tmp_surroundingKeyframeSearchRadius = 50;
    }
    else if (envDensity == "sparse") {
        tmp_surroundingKeyframeSearchRadius = 40;
    }
    else if (envDensity == "dense") {
        tmp_surroundingKeyframeSearchRadius = 30;
    }
    else {
        return;
    }
   

    nh.setParam("surroundingKeyframeSearchRadius", tmp_surroundingKeyframeSearchRadius);

    // ROS_INFO("In parameter service, surroundingKeyframeSearchRadius now is: %f", tmp_surroundingKeyframeSearchRadius);
}

void Tunning::tunningRule_2()
{

}

void Tunning::tunningThread()
{
    ros::Rate rate(1);

    while (ros::ok())
    {
        tunningRule_1();
        tunningRule_2();
        rate.sleep();
    }
}




