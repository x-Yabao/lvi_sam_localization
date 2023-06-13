#include "parameters.h"
#include "featureExtraction.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "featureExtraction");

    readParameters();

    FeatureExtraction FE;

    ROS_INFO("\033[1;32m----> Feature Extraction Started.\033[0m");
   
    ros::spin();

    return 0;
}