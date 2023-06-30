#pragma once

#include "parameters.h"


class Tunning
{
public:
    Tunning();

    // 订阅相关话题，获取用于调参的信息
    void envDensityCallback(const std_msgs::String::ConstPtr& msg);

    // 调参规则
    void tunningRule_1(); 
    void tunningRule_2();

    void tunningThread();

private:
    ros::NodeHandle nh;
    ros::Subscriber subEnvDensityInfo;

    // 用于调参的信息
    std::string envDensity;

    std::mutex mtx;

};