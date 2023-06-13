#include "parameters.h"
#include "mapOptimization.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mapOptimization");
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);  

    readParameters();

    MapOptimization MO;

    ROS_INFO("\033[1;32m----> Map Optimization Started.\033[0m");
    std::thread visualizeMapThread(&MapOptimization::visualizeGlobalMapThread, &MO);

    MO.loadBinaryMap();
    MO.speedUpExtractSurroundingKeyFrames();
    ROS_INFO("Load map from binary file finish!");

    ros::spin();

    visualizeMapThread.join();

    return 0;
}