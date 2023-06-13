#include "MultiMap.h"
#include "Location.h"
#include "parameters.h"

#include <chrono>

int main(int argc, char **argv) {

    // log 设置
    google::InitGoogleLogging("log_info");                                    //使用glog之前必须先初始化库，仅需执行一次，括号内为程序名
    FLAGS_alsologtostderr = true;                                             //是否将日志输出到文件和stderr
    FLAGS_colorlogtostderr = true;                                            //是否启用不同颜色显示
    google::SetLogDestination(google::GLOG_INFO, "../logs/INFO_");            //INFO级别的日志都存放到logs目录下且前缀为INFO_
    google::SetLogDestination(google::GLOG_WARNING, "../logs/WARNING_");      //WARNING级别的日志都存放到logs目录下且前缀为WARNING_
    google::SetLogDestination(google::GLOG_ERROR, "../logs/ERROR_");          //ERROR级别的日志都存放到logs目录下且前缀为ERROR_
    google::SetLogDestination(google::GLOG_FATAL, "../logs/FATAL_");          //FATAL级别的日志都存放到logs目录下且前缀为FATAL_
    
    readParameters();

    Location location;
    location.loadBinaryMap();

    // 开启监测模块
    // std::shared_ptr<EnvInfo> envinfo = std::shared_ptr<EnvInfo>(new EnvInfo);
    // std::shared_ptr<SysInfo> sysinfo = std::shared_ptr<SysInfo>(new SysInfo);
    // location.setTunningEnvInfo(envinfo);
    // location.setTunningSysInfo(sysinfo);

    // 开启tunning
    // location.startParasTunning();

    // 控制变量
    int start_frame = START_FRAME;
    int end_frame = END_FRAME;
    bool process_all = true;            // 是否处理全部帧
    int process_count = 0;              // 已测试的帧数

    // 保存轨迹
    std::string filename("kitti_trajectory.txt");
    ofstream traj_file;
    traj_file.open(filename.c_str());
    traj_file << fixed;

    // 准备路径，读取同步的图像和lidar数据
    // raw data
    // std::string kitti_path = DATASET_PATH;
    // std::string camera_path = kitti_path + "/image_00/data/";
    // std::string lidar_path = kitti_path + "/velodyne_points/data/";
    // std::string timestamps_path = kitti_path + "/velodyne_points/my_timestamps.txt";

    // odometry
    std::string kitti_path = "/media/yabao/TOSHIBA1/Datasets/KITTI/odometry";
    std::string camera_path = kitti_path + "/data_odometry_gray/dataset/sequences/07/image_0/";
    std::string lidar_path = kitti_path + "/data_odometry_velodyne/dataset/sequences/07/velodyne/";
    std::string timestamps_path = kitti_path + "/data_odometry_calib/dataset/sequences/07/times.txt";

    // 读取时间戳文件
    std::vector<std::string> timestamps;
    std::ifstream timestamps_file;
    timestamps_file.open(timestamps_path.c_str());

    if(!timestamps_file.is_open()) {
        std::cout << "Can't find timestamps file in: " << timestamps_path << std::endl;
        return -1;
    }

    while(!timestamps_file.eof()) {
        string line;
        getline(timestamps_file, line);
        
        if(!line.empty())
            timestamps.push_back(line);
    }
    timestamps_file.close();

    if(process_all) {
        start_frame = 0;
        end_frame = timestamps.size();
    }

    double elapsed_ms = 0;
    for(int i = start_frame; i < end_frame; i++){
        process_count++;

        // 1.准备路径
        // std::string idx = padZeros(i, 10);   // raw data
        std::string idx = padZeros(i, 6);       // odometry
        std::string image_path = camera_path + idx + ".png";
        std::string cloud_path = lidar_path + idx + ".bin";

        // 2.读数数据集
        cv::Mat im;
        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());

        im = cv::imread(image_path, 0);
        if(im.empty()) {
            std::cout << "Dataset: failed to load image at: " << image_path << std::endl;
            return -1;
        }

        if (_loadBINFile(cloud_path, *cloud) == -1) {
            std::cout << "Dataset: failed to load cloud at: " << cloud_path << std::endl;
            return -1;
        };

        // 3.开始处理每一帧
        std::cout << "****** Process the " << process_count << " frame now. ******" << std::endl;
        location.process(im, cloud);
        std::cout << "****** Process the " << process_count << " frame finish. ******" << std::endl;

        //4.tunning
        // envinfo->addPointCloud(cloud);
        // location.makeParasTunning();
        // LOG(INFO) << "frequency is:" << location.getLocaFrequency() << std::endl;

        // 5.保存定位结果
        if(location.getLocationStatus() == LocationStatus::TRACKING_GOOD ||
            location.getLocationStatus() == LocationStatus::TRACKING_BAD) {
            PointType pos;
            Eigen::Matrix3d R;
            location.getLocationResult(pos, R);
            Eigen::Quaterniond q(R);

            // double timestamp = std::stod(timestamps[i]) - std::stod(timestamps[0]);      // raw data
            std::string timestamp = timestamps[i];                                          // odometry
            traj_file << timestamp << " " <<
                setprecision(7) << pos.x << " " << pos.y << " " << pos.z << " " << 
                q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
        }

        elapsed_ms += location.getLastProcessTime() * 1000;
    }

    traj_file.close();

    // 6.输出最终结果
    std::cout << "Finish run, per frame use time: " << elapsed_ms / process_count << "ms" << std::endl;
    std::cout << "Relocation time is: " << location.getRelocationTime() << std::endl;
    std::cout << "Relocation sucess time is: " << location.getRelocationSucessTime() << std::endl;

    // location.stopParasTunning();
    // sysinfo->stop();

    google::ShutdownGoogleLogging(); 
    
    return 0;
}
