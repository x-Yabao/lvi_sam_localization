#include "tunning.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tunning");

    readParameters();

    Tunning tunning;

    ROS_INFO("\033[1;32m----> Tunning Started.\033[0m");

    std::thread makeTunningThread(&Tunning::tunningThread, &tunning);

    ros::spin();

    makeTunningThread.join();

    return 0;
}