#pragma once

#include "parameters.h"


class Tunning
{
public:
    Tunning();
    void envDensityCallback(const std_msgs::String::ConstPtr& msg);

    ros::NodeHandle nh;

    ros::Subscriber subEnvDensityInfo;


};