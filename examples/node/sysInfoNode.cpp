#include "SysInfo.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sysInfoNode");
    ros::NodeHandle n;
    ros::Rate loop_rate(0.5);

    SysInfo sysInfo;

    while (ros::ok())
    {
        sysInfo.updateSysInfo();
        sysInfo.outputSysInfo();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}