# Cloud Info
Header header 

int32[] startRingIndex  
int32[] endRingIndex

int32[]  pointColInd    # point column index in range image
float32[] pointRange    # point range 

int64 imuAvailable
int64 odomAvailable

# Attitude for LOAM initialization（imuAvailable为true，则下面的数据可用）
# imu 磁力计测得的rpy
float32 imuRollInit
float32 imuPitchInit
float32 imuYawInit

# Initial guess from imu pre-integration（odomAvailabe为true，则下面的数据可用）
# IMU 里程计信息得到的帧起始位姿估计
float32 initialGuessX
float32 initialGuessY
float32 initialGuessZ
float32 initialGuessRoll
float32 initialGuessPitch
float32 initialGuessYaw

# Point cloud messages
sensor_msgs/PointCloud2 cloud_deskewed  # original cloud deskewed，去掉无效点的
sensor_msgs/PointCloud2 cloud_corner    # extracted corner feature
sensor_msgs/PointCloud2 cloud_surface   # extracted surface feature

# image
int64 imageAvailable
sensor_msgs/Image image