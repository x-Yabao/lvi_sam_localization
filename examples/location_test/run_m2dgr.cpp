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
    bool process_all = false;           // 是否处理全部帧
    int process_count = 0;              // 已测试的帧数

    // 保存轨迹
    std::string filename("m2dgr_trajectory.txt");
    ofstream traj_file;
    traj_file.open(filename.c_str());
    traj_file << fixed;
    
    // 准备路径
    std::string m2dgr_path = DATASET_PATH;
    string sync_file_path = m2dgr_path + "/sync.txt";
    std::ifstream sync_file;
    sync_file.open(sync_file_path.c_str());

    if(!sync_file.is_open()) {
        std::cout << "Can't find sync file in: " << sync_file_path << std::endl;
        return -1;
    }

    // 读取同步文件
    std::vector<std::pair<std::string, std::string>> lidar_image_syncs;

    while(!sync_file.eof()) {
        string line;
        getline(sync_file, line);
        
        if(!line.empty()) {
            stringstream ss;
            ss << line;
            string lidar_file, image_file;
            ss >> lidar_file >> image_file;

            lidar_image_syncs.push_back(std::make_pair(lidar_file, image_file));
        }
    }
    sync_file.close();

    if(process_all) {
        start_frame = 0;
        end_frame = lidar_image_syncs.size();
    }

    double elapsed_ms = 0;
    for(int i = start_frame; i < end_frame; i++) {
        process_count++;

        // 1.准备路径
        std::string lidar_path = m2dgr_path + "/pcd/" + lidar_image_syncs[i].first;
        std::string image_path = m2dgr_path + "/image/" + lidar_image_syncs[i].second;

        // 2.读数数据集
        cv::Mat im;
        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointXYZIR>::Ptr cloud_ring(new pcl::PointCloud<PointXYZIR>());

        im = cv::imread(image_path, 0);
        if(im.empty()) {
            std::cout << "Dataset: failed to load image at: " << image_path << std::endl;
            return -1;
        }

        if (pcl::io::loadPCDFile(lidar_path, *cloud) == -1) {
            std::cout << "Dataset: failed to load cloud at: " << lidar_path << std::endl;
            return -1;
        };

        if (pcl::io::loadPCDFile(lidar_path, *cloud_ring) == -1) {
            std::cout << "Dataset: failed to load cloud at: " << lidar_path << std::endl;
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

            traj_file << lidar_image_syncs[i].first.substr(0, 20) << " " <<
                setprecision(7) << pos.x << " " << pos.y << " " << pos.z << " " << 
                q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
        }

        // 去到下个地方重定位
        if(location.getLocationStatus() == LocationStatus::LOST)
            i += 5;

        elapsed_ms += location.getLastProcessTime() * 1000;
    }

    traj_file.close();

    // 6.输出最终结果
    std::cout << "Finish run, per frame use time: " << elapsed_ms / process_count << "ms" << std::endl;
    std::cout << "Relocation time is: " << location.getRelocationTime() << std::endl;
    std::cout << "Relocation sucess time is: " << location.getRelocationSucessTime() << std::endl;
    std::cout << "surroundingKeyframeSearchRadius is: " << surroundingKeyframeSearchRadius << std::endl;

    // location.stopParasTunning();
    // sysinfo->stop();

    google::ShutdownGoogleLogging(); 

    return 0;
}
