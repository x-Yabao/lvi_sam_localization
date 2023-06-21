#include "parameters.h"
#include "globalLocalization.h"

// 把地图以文件夹组织的形式转变为bin文件的格式

int main(int argc, char** argv)
{
    ros::init(argc, argv, "globalLocalization");

    readParameters();

    mapOptimization MO;

    ROS_INFO("\033[1;32m----> Map Optimization Started.\033[0m");

    MO.loadFolderMap();
    MO.saveBinaryMap();

    std::cout << "Map saved. Please press Ctrl + C to finish." << std::endl;
    ros::spin();

    return 0;
}