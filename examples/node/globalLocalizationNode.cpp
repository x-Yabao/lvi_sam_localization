#include "parameters.h"
#include "globalLocalization.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "globalLocalization");

    readParameters();

    mapOptimization MO;

    ROS_INFO("\033[1;32m----> Map Optimization Started.\033[0m");

    // std::thread loopthread(&mapOptimization::loopClosureThread, &MO);
    // std::thread visualizeMapThread(&mapOptimization::visualizeGlobalMapThread, &MO);
    std::thread localizeInWorldThread(&mapOptimization::globalLocalizeThread, &MO);

    ros::spin();

    // loopthread.join();
    // visualizeMapThread.join();
    localizeInWorldThread.join();

    return 0;
}