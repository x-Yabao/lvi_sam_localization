#include "tunning.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tunning");

    readParameters();

    Tunning tunning;

    ROS_INFO("\033[1;32m----> Tunning Started.\033[0m");

    ros::spin();

    return 0;
}