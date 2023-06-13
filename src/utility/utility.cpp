#include "utility.h"

Eigen::Matrix3d Utility::g2R(const Eigen::Vector3d &g)
{
    Eigen::Matrix3d R0;
    Eigen::Vector3d ng1 = g.normalized();
    Eigen::Vector3d ng2{0, 0, 1.0};
    R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
    double yaw = Utility::R2ypr(R0).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    // R0 = Utility::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
    return R0;
}

std::string padZeros(int val, int num_digits) {
    std::ostringstream out;
    out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
    return out.str();
}


#if defined(NCLT)
int _loadBINFile(std::string infile, pcl::PointCloud<pcl::PointXYZI> &cloud){
    // Load point cloud
    std::fstream input(infile.c_str(), std::ios::in | std::ios::binary);
    if(!input.good()){
        std::cerr << "Could not read file: " << infile << std::endl;
        return -1;
    }
    input.seekg(0, std::ios::beg);

    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud (new pcl::PointCloud<pcl::PointXYZI>);

    int i;
    for (i = 0; input.good() && !input.eof(); i++) {
        pcl::PointXYZI point;

        unsigned short u_x, u_y, u_z;
        unsigned char u_i, u_l;
        input.read((char *) &u_x, sizeof(unsigned short));
        input.read((char *) &u_y, sizeof(unsigned short));
        input.read((char *) &u_z, sizeof(unsigned short));
        input.read((char *) &u_i, sizeof(unsigned char));
        input.read((char *) &u_l, sizeof(unsigned char));

        float scaling = 0.005;
        float offset = -100.0;
        float x, y, z;
        x = u_x * scaling + offset;
        y = u_y * scaling + offset;
        z = u_z * scaling + offset;

        point.x = x;
        point.y = y;
        point.z = z;
        point.intensity = i;
        pointCloud->push_back(point);
    }
    input.close();

    cloud = *pointCloud;

    return 0;
}
#else
int _loadBINFile(std::string infile, pcl::PointCloud<pcl::PointXYZI> &cloud){
    // Load point cloud
    std::fstream input(infile.c_str(), std::ios::in | std::ios::binary);
    if(!input.good()){
        std::cerr << "Could not read file: " << infile << std::endl;
        return -1;
    }
    input.seekg(0, std::ios::beg);

    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud (new pcl::PointCloud<pcl::PointXYZI>);

    int i;
    for (i = 0; input.good() && !input.eof(); i++) {
        pcl::PointXYZI point;
        input.read((char *) &point.x, 3 * sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));
        point.intensity *= 255;
        pointCloud->push_back(point);
    }
    input.close();

    cloud = *pointCloud;

    return 0;
}
#endif


void CameraFrame2LidarFrame(const Eigen::Vector3d& camera_T, const Eigen::Matrix3d& camera_R,
                            Eigen::Vector3d& lidar_T, Eigen::Matrix3d& lidar_R)
{
    Eigen::Vector3d camera_to_lidar_T, lidar_to_camera_T, tmp_YPR;
    Eigen::Matrix3d camera_to_lidar_R, lidar_to_camera_R;
    // get lidar_to_camera
    tmp_YPR <<  C_RZ_L, C_RY_L, C_RX_L;
    lidar_to_camera_R = Utility::ypr2R_radian(tmp_YPR);
    lidar_to_camera_T << C_TX_L, C_TY_L, C_TZ_L;
    // get camera_to_lidar
    camera_to_lidar_R = lidar_to_camera_R.transpose();
    camera_to_lidar_T = -camera_to_lidar_R * lidar_to_camera_T;
    
    lidar_R = camera_to_lidar_R * camera_R;
    lidar_T = camera_to_lidar_R * camera_T + camera_to_lidar_T;
    return;
}

void getFiles(std::string path, std::vector<std::string>& file) 
{
    DIR *dr;
    struct dirent *en;
    dr = opendir(path.c_str()); 
    if(dr) {
        while ((en = readdir(dr)) != NULL) {
            //std::cout << en->d_name << std::endl; 
            std::string tmp(en->d_name);
            if(tmp == "." || tmp == "..")
                continue;
            file.push_back(tmp);
        }
        closedir(dr); 
    }
}

/***********************lio-sam***************************/
sensor_msgs::Imu imuConverter(const sensor_msgs::Imu& imu_in)
{
    sensor_msgs::Imu imu_out = imu_in;
    // 这里把imu的数据旋转到前左上坐标系下，可以参考https://github.com/TixiaoShan/LIO-SAM/issues/6
    // rotate acceleration
    Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
    acc = extRot * acc;
    imu_out.linear_acceleration.x = acc.x();
    imu_out.linear_acceleration.y = acc.y();
    imu_out.linear_acceleration.z = acc.z();
    // rotate gyroscope
    Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
    gyr = extRot * gyr;
    imu_out.angular_velocity.x = gyr.x();
    imu_out.angular_velocity.y = gyr.y();
    imu_out.angular_velocity.z = gyr.z();
    // rotate roll pitch yaw
    // 这是一个九轴imu，因此还会有姿态信息
    Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
    Eigen::Quaterniond q_final = q_from * extQRPY;
    imu_out.orientation.x = q_final.x();
    imu_out.orientation.y = q_final.y();
    imu_out.orientation.z = q_final.z();
    imu_out.orientation.w = q_final.w();
    // 简单校验一下结果
    if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
    {
        ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
        ros::shutdown();
    }

    return imu_out;
}

sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->getNumSubscribers() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}

float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}



