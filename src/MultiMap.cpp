#include "MultiMap.h"

int MultiMap::testMap()
{   
    // Test1:检查激光路径和视觉路径的差别
    std::cout << "Compare the lidar trajectory and camera trajectory!" << std::endl;
    pcl::PointCloud<PointType>::Ptr trajectory_lidar(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr trajectory_camera(new pcl::PointCloud<PointType>());
    for(int i = 0; i < cloudKeyPoses3D->size(); i++){
        PointType thisPose3D;
        thisPose3D.x = -cloudKeyPoses3D->points[i].x;
        thisPose3D.y = -cloudKeyPoses3D->points[i].y;
        thisPose3D.z = cloudKeyPoses3D->points[i].z;
        thisPose3D.intensity = trajectory_lidar->size(); 
        trajectory_lidar->push_back(thisPose3D); 
    }
    pcl::io::savePCDFileASCII("trajectory_lidar.pcd", *trajectory_lidar);

	// 保存的地图里，vins的坐标系和lio的坐标系绕z轴旋转了180度
	Eigen::AngleAxisd rotation_vector(M_PI, Vector3d(0, 0, 1));
	Eigen::Matrix3d vins_to_lio = rotation_vector.toRotationMatrix();


    for(auto& kf : keyframelist){
        Eigen::Vector3d temp_T = vins_to_lio * kf->T_w_i;
        PointType thisPose3D;
        // thisPose3D.x = temp_T.x();
        // thisPose3D.y = temp_T.y();
        // thisPose3D.z = temp_T.z();
        thisPose3D.x = kf->T_w_i.x();
        thisPose3D.y = kf->T_w_i.y();
        thisPose3D.z = kf->T_w_i.z();
        thisPose3D.intensity = trajectory_camera->size(); 
        trajectory_camera->push_back(thisPose3D);
    }
    pcl::io::savePCDFileASCII("trajectory_camera.pcd", *trajectory_camera);

    //Test2:检查激光关键帧的时间戳
    for(int i = 0; i < cloudKeyPoses6D->size(); i++) {
        std::cout << "lidar frame time stamp " << ": " << 
        fixed << setprecision(5) << cloudKeyPoses6D->points[i].time << std::endl;
    }

    return 0;
}

int MultiMap::buildNCLTMap()
{
    std::cout << "Building the nclt map." << std::endl;
    TicToc t_build(true);

    loadVocabulary(VOC_PATH);                   // 加载字典
    // 1.准备
    //(1)准备路径
    std::string nclt_path = "/media/yabao/TOSHIBA1/Datasets/NCLT/raw_data";
    std::string date = "2012-04-29";
    std::string lidar_folder = nclt_path + "/" + date + "/velodyne_data/" + date + "/velodyne_sync/";
    std::string image_folder = nclt_path + "/" + date + "/images/" + date + "/lb3/Cam5/";

    //(2)获取所有点云文件
    std::vector<std::string> lidar_list;        // 所有点云
    getFiles(lidar_folder, lidar_list);
    sort(lidar_list.begin(), lidar_list.end());

    int totalNum = 600;              // 地图大小 600
    int startFrame = 20;            // 地图开始帧 20
    int skipNum = 4;               // 间隔帧数 4

    for(int i = 0; i < totalNum; i++) {
        // 2.构建lidar keyframe
        std::string lidar_name = lidar_list[startFrame + i * skipNum];
        std::string lidar_path = lidar_folder + lidar_name;
        //std::cout << lidar_path << std::endl;
        pcl::PointCloud<PointType>::Ptr theCloudKeyFrame(new pcl::PointCloud<PointType>());
        // 验证以下kitti的bin和nclt的bin格式一样吗
        if(_loadBINFile(lidar_path, *theCloudKeyFrame) == -1){
            std::cout << "Load pcd file failed." << std::endl;
            return -1;
        }
        //pcl::io::savePCDFile(lidar_name.substr(0, 16) + ".pcd",*theCloudKeyFrame);       // 验证_loadBINFile()是否正确
        cloudKeyFrames.push_back(theCloudKeyFrame);
        scManager.makeAndSaveScancontextAndKeys(*theCloudKeyFrame);

        // 3.构建image keyframe
        //(1)准备路径
        std::string image_name = lidar_name.substr(0, 16) + ".tiff";
        std::string image_path = image_folder + image_name;
        //std::cout << image_path << std::endl;
        //(2)读取图片
        cv::Mat image;
        image = cv::imread(image_path, 0);
        if(image.empty()){
            std::cout << "Load the picture from: " << image_path << " failed!" << std::endl;
            return -1;
        }
        cv::transpose(image, image); 
        cv::flip(image, image, 1);          // 90度旋转

        // cv::imshow("Picture", image);
        // cv::waitKey();
        //(3)构建关键帧
        double time_stamp = 0;
        int frame_index = keyframelist.size();
        Vector3d T = Vector3d::Zero();
        Matrix3d R = Matrix3d::Zero();
        vector<cv::Point3f> point_3d;
        vector<cv::Point2f> point_2d_uv;
        vector<cv::Point2f> point_2d_normal;
        vector<double> point_id;
        int sequence = 0;

        KeyFrame* keyframe = new KeyFrame(time_stamp, frame_index, T, R,
                                image, point_3d, point_2d_uv, point_2d_normal, point_id, sequence);
        
        //(4)加载关键帧到地图
        loadKeyFrame(keyframe);
    }

    t_build.toc("Building the nclt map sucess");

    return 0;
}

MultiMap::MultiMap()
{
    // lidar map
    allocateMemory();

    // visual map
    global_index = 0;
}

void MultiMap::allocateMemory()
{
    cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
    cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
}

int MultiMap::loadMultiMap()
{
    if(loadLidarMap() != 0){
        std::cout << "Load lidar map failure!" << std::endl;
        return -1;
    }
    if(loadVisualMap() != 0){
        std::cout << "Load visual map failure!" << std::endl;
        return -1;
    }
    return 0;
}


/********************************* lidar map ***************************************/

int MultiMap::loadLidarMap()
{
    loadCloudKeyFrames();

    loadCornerAndSurf();

    loadSCbyScans();

    loadTrajectory();

    loadTransformations();

    return 0;
}

int MultiMap::loadCloudKeyFrames()
{
    std::cout << "Loading the cloud keyframe." << std::endl;
    TicToc t_load(true);
    for(int i = 0; i < lidar_keyFrame_num; i++) {
        std::string idx = padZeros(i);
        std::string file_path = loading_path + "/lidar/Scans/" + idx + ".pcd";
        pcl::PointCloud<PointType>::Ptr theCloudKeyFrame(new pcl::PointCloud<PointType>());
        if(pcl::io::loadPCDFile(file_path, *theCloudKeyFrame) == -1){
            std::cout << "Load pcd file failed." << std::endl;
            return -1;
        }
        cloudKeyFrames.push_back(theCloudKeyFrame);
    }
    t_load.toc("Load the cloud keyframe sucess");
    return 0;
}

int MultiMap::loadCornerAndSurf()
{
    std::cout << "Loading the corner and surf." << std::endl;
    TicToc t_load(true);
    for(int i = 0; i < lidar_keyFrame_num; i++) {
        std::string idx = padZeros(i);
        std::string corner_path = loading_path + "/lidar/Corners/" + idx + ".pcd";
        std::string surf_path = loading_path + "/lidar/Surfs/" + idx + ".pcd";
        pcl::PointCloud<PointType>::Ptr theCornerFrame(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr theSurfFrame(new pcl::PointCloud<PointType>());
        if(pcl::io::loadPCDFile(corner_path, *theCornerFrame) == -1){
            std::cout << "Load pcd file failed." << std::endl;
            return -1;
        }
        if(pcl::io::loadPCDFile(surf_path, *theSurfFrame) == -1){
            std::cout << "Load pcd file failed." << std::endl;
            return -1;
        }
        cornerCloudKeyFrames.push_back(theCornerFrame);
        surfCloudKeyFrames.push_back(theSurfFrame);
    }
    t_load.toc("Load the corner and surf sucess.");
    return 0;
}

int MultiMap::loadSCbyScans()
{
    std::cout << "Loading the keyframe Scancontext." << std::endl;
    TicToc t_load(true);
    for(int i = 0; i < lidar_keyFrame_num; i++) {
        std::string idx = padZeros(i);
        std::string file_path = loading_path + "/lidar/Scans/" + idx + ".pcd";
        pcl::PointCloud<PointType>::Ptr theCloudKeyFrame(new pcl::PointCloud<PointType>());
        if(pcl::io::loadPCDFile(file_path, *theCloudKeyFrame) == -1){
            std::cout << "Load pcd file failed." << std::endl;
            return -1;
        }
        scManager.makeAndSaveScancontextAndKeys(*theCloudKeyFrame);
    }
    t_load.toc("Load the keyframe Scancontext sucess");
    return 0;
}

int MultiMap::loadTrajectory()
{
    std::cout << "Loading the Trajectory." << std::endl;
    std::string file_path = loading_path + "/lidar/trajectory.pcd";
    if(pcl::io::loadPCDFile(file_path, *cloudKeyPoses3D) == -1){
        std::cout << "Load pcd file failed." << std::endl;
        return -1;
    }
    std::cout << "Load the Trajectory sucess." << std::endl;
    return 0;
}

int MultiMap::loadTransformations()
{
    std::cout << "Loading the Transformations." << std::endl;
    std::string file_path = loading_path + "/lidar/transformations.pcd";
    if(pcl::io::loadPCDFile(file_path, *cloudKeyPoses6D) == -1){
        std::cout << "Load pcd file failed." << std::endl;
        return -1;
    }
    std::cout << "Load the Transformations sucess." << std::endl;
    return 0;
}


/********************************* visual map ***************************************/

int MultiMap::loadVisualMap()
{
    loadVocabulary(VOC_PATH);

    loadPoseGraph();

    return 0;
}

void MultiMap::loadVocabulary(std::string voc_path)
{
    voc = new BriefVocabulary(voc_path);
    db.setVocabulary(*voc, false, 0);
}

int MultiMap::loadPoseGraph()
{
    TicToc tmp_t(true);
    FILE * pFile;
    string file_path = POSE_GRAPH_SAVE_PATH + "pose_graph.txt";
    printf("lode pose graph from: %s \n", file_path.c_str());
    printf("pose graph loading...\n");
    pFile = fopen (file_path.c_str(),"r");
    if (pFile == NULL)
    {
        printf("lode previous pose graph error: wrong previous pose graph path or no previous pose graph \n");
        return -1;
    }
    int index;
    double time_stamp;
    double VIO_Tx, VIO_Ty, VIO_Tz;
    double PG_Tx, PG_Ty, PG_Tz;
    double VIO_Qw, VIO_Qx, VIO_Qy, VIO_Qz;
    double PG_Qw, PG_Qx, PG_Qy, PG_Qz;
    double loop_info_0, loop_info_1, loop_info_2, loop_info_3;
    double loop_info_4, loop_info_5, loop_info_6, loop_info_7;
    int loop_index;
    int keypoints_num;
    int window_keypoints_num;   // yabao
    Eigen::Matrix<double, 8, 1 > loop_info;
    int cnt = 0;
    while (fscanf(pFile,"%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %d %d", &index, &time_stamp, 
                                    &VIO_Tx, &VIO_Ty, &VIO_Tz, 
                                    &PG_Tx, &PG_Ty, &PG_Tz, 
                                    &VIO_Qw, &VIO_Qx, &VIO_Qy, &VIO_Qz, 
                                    &PG_Qw, &PG_Qx, &PG_Qy, &PG_Qz, 
                                    &loop_index,
                                    &loop_info_0, &loop_info_1, &loop_info_2, &loop_info_3, 
                                    &loop_info_4, &loop_info_5, &loop_info_6, &loop_info_7,
                                    &keypoints_num, &window_keypoints_num) != EOF) 
    {
        cv::Mat image;
        std::string image_path, descriptor_path;
        if (DEBUG_IMAGE)
        {
            image_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_image.png";
            image = cv::imread(image_path.c_str(), 0);
        }

        Vector3d VIO_T(VIO_Tx, VIO_Ty, VIO_Tz);
        Vector3d PG_T(PG_Tx, PG_Ty, PG_Tz);
        Quaterniond VIO_Q;
        VIO_Q.w() = VIO_Qw;
        VIO_Q.x() = VIO_Qx;
        VIO_Q.y() = VIO_Qy;
        VIO_Q.z() = VIO_Qz;
        Quaterniond PG_Q;
        PG_Q.w() = PG_Qw;
        PG_Q.x() = PG_Qx;
        PG_Q.y() = PG_Qy;
        PG_Q.z() = PG_Qz;
        Matrix3d VIO_R, PG_R;
        VIO_R = VIO_Q.toRotationMatrix();
        PG_R = PG_Q.toRotationMatrix();
        Eigen::Matrix<double, 8, 1 > loop_info;
        loop_info << loop_info_0, loop_info_1, loop_info_2, loop_info_3, loop_info_4, loop_info_5, loop_info_6, loop_info_7;

        // load keypoints, brief_descriptors   
        string brief_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_briefdes.dat";
        std::ifstream brief_file(brief_path, std::ios::binary);
        string keypoints_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_keypoints.txt";
        FILE *keypoints_file;
        keypoints_file = fopen(keypoints_path.c_str(), "r");
        vector<cv::KeyPoint> keypoints;
        vector<cv::KeyPoint> keypoints_norm;
        vector<DVision::BRIEF::bitset> brief_descriptors;
        for (int i = 0; i < keypoints_num; i++)
        {
            DVision::BRIEF::bitset tmp_des;
            brief_file >> tmp_des;
            brief_descriptors.push_back(tmp_des);
            cv::KeyPoint tmp_keypoint;
            cv::KeyPoint tmp_keypoint_norm;
            double p_x, p_y, p_x_norm, p_y_norm;
            if(!fscanf(keypoints_file,"%lf %lf %lf %lf", &p_x, &p_y, &p_x_norm, &p_y_norm))
                printf(" fail to load pose graph \n");
            tmp_keypoint.pt.x = p_x;
            tmp_keypoint.pt.y = p_y;
            tmp_keypoint_norm.pt.x = p_x_norm;
            tmp_keypoint_norm.pt.y = p_y_norm;
            keypoints.push_back(tmp_keypoint);
            keypoints_norm.push_back(tmp_keypoint_norm);
        }
        brief_file.close();
        fclose(keypoints_file);

        // load window_keypoints, window_brief_descriptors -- yabao   
        string window_brief_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_window_briefdes.dat";
        std::ifstream window_brief_file(window_brief_path, std::ios::binary);
        string window_keypoints_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_window_keypoints.txt";
        FILE *window_keypoints_file;
        window_keypoints_file = fopen(window_keypoints_path.c_str(), "r");

        vector<cv::Point3f> point_3d;  					// 关键帧对应的3D点
	    vector<cv::Point2f> point_2d_uv;  				// 特征点的像素坐标
	    vector<cv::Point2f> point_2d_norm;  			// 特征点的归一化坐标
	    vector<double> point_id;						// 特征点的id
        vector<cv::KeyPoint> window_keypoints;   		// 原来光流追踪的特征点的像素坐标
        vector<DVision::BRIEF::bitset> window_brief_descriptors;

        for (int i = 0; i < window_keypoints_num; i++)
        {
            DVision::BRIEF::bitset tmp_des;
            window_brief_file >> tmp_des;
            window_brief_descriptors.push_back(tmp_des);
            cv::Point3f tmp_point_3d; 
            cv::Point2f tmp_point_2d_uv;  
            cv::Point2f tmp_point_2d_norm;  
            double tmp_point_id;	
            cv::KeyPoint tmp_keypoint;
            double p_3d_x, p_3d_y, p_3d_z, p_2d_uv_x, p_2d_uv_y, p_2d_norm_x, p_2d_norm_y;
            if(!fscanf(window_keypoints_file,"%lf %lf %lf %lf %lf %lf %lf %lf", 
                &p_3d_x, &p_3d_y, &p_3d_z, &p_2d_uv_x, &p_2d_uv_y, &p_2d_norm_x, &p_2d_norm_y, &tmp_point_id))
                printf(" fail to load pose graph \n");

            tmp_point_3d.x = p_3d_x;
            tmp_point_3d.y = p_3d_y;
            tmp_point_3d.z = p_3d_z;
            tmp_keypoint.pt.x = p_2d_uv_x;
            tmp_keypoint.pt.y = p_2d_uv_y;
            tmp_point_2d_uv.x = p_2d_uv_x;
            tmp_point_2d_uv.y = p_2d_uv_y;
            tmp_point_2d_norm.x = p_2d_norm_x;
            tmp_point_2d_norm.y = p_2d_norm_y;

            point_3d.push_back(tmp_point_3d);	
	        point_2d_uv.push_back(tmp_point_2d_uv); 				
	        point_2d_norm.push_back(tmp_point_2d_norm);
	        point_id.push_back(tmp_point_id);					
            window_keypoints.push_back(tmp_keypoint);   		
        }
        window_brief_file.close();
        fclose(window_keypoints_file);
        // load window_keypoints, window_brief_descriptors -- yabao   
        
        KeyFrame* keyframe = new KeyFrame(time_stamp, index, VIO_T, VIO_R, PG_T, PG_R, image, loop_index, loop_info, 
                                        keypoints, keypoints_norm, brief_descriptors,
                                        point_3d, point_2d_uv, point_2d_norm, point_id, window_keypoints, window_brief_descriptors);
        loadKeyFrame(keyframe);
        
        if (cnt % 20 == 0)
        {
            // publish();
        }
        cnt++;
    }
    fclose (pFile);
    tmp_t.toc("load pose graph time");
    return 0;
}

void MultiMap::loadKeyFrame(KeyFrame* cur_kf)
{
    cur_kf->index = global_index;           // 第一帧index为0
    global_index++;

    db.add(cur_kf->brief_descriptors);      // 词袋用的是fast角点，没有用光流追踪的点
    keyframelist.push_back(cur_kf);
}

template<class Archive>
void MultiMap::serialize(Archive &ar, const unsigned int version)
{
    ar & cloudKeyFrames & cornerCloudKeyFrames & surfCloudKeyFrames;
    ar & cloudKeyPoses3D & cloudKeyPoses6D;
    ar & keyframelist;
}
template void MultiMap::serialize(boost::archive::binary_iarchive&, const unsigned int);
template void MultiMap::serialize(boost::archive::binary_oarchive&, const unsigned int);

