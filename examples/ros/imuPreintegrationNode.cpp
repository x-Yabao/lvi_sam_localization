#include "parameters.h"
#include "imuPreintegration.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imuPreintegration");

    readParameters();
    
    IMUPreintegration ImuP;

    TransformFusion TF;

    ROS_INFO("\033[1;32m----> IMU Preintegration Started.\033[0m");
    
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    
    return 0;
}