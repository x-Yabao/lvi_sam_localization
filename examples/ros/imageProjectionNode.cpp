#include "parameters.h"
#include "imageProjection.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "imageProjection");

    readParameters();
    
    ImageProjection IP;
    
    ROS_INFO("\033[1;32m----> Image Projection Started.\033[0m");

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    
    return 0;
}