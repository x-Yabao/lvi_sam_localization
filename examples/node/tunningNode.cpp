#include "tunning.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tunning");

    readParameters();

    Tunning tunning;

    ROS_INFO("\033[1;32m----> Tunning Started.\033[0m");

    ros::Rate rate(0.5);

    while (ros::ok())
    {
        ros::spinOnce();

        tunning.tunningRule_1();
        tunning.tunningRule_2();

        rate.sleep();
    }

    return 0;
}