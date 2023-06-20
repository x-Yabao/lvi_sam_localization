cmake_minimum_required(VERSION 2.8.3)
project(lvi_sam_localization)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
# 确保openmp是打开的，可以快上一倍
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread -fopenmp")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  visualization_msgs
)

find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Glog REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer serialization)
find_package(GTSAM REQUIRED QUIET)

set(THIRD_PARTY_LIBS 
  ${cv_bridge_LIBRARIES}
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES}  
  ${GLOG_LIBRARIES} 
  Boost::timer 
  Boost::serialization
  gtsam
)

add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

# catkin_package(
#     DEPENDS PCL GTSAM
# )

include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/src/ThirdParty/lidar
  ${PROJECT_SOURCE_DIR}/src/ThirdParty/camera
  ${PROJECT_SOURCE_DIR}/src/ThirdParty/camera/DBow
  ${PROJECT_SOURCE_DIR}/src/ThirdParty/camera/DUtils
  ${PROJECT_SOURCE_DIR}/src/ThirdParty/camera/DVision
  ${PROJECT_SOURCE_DIR}/src/utility
  ${PROJECT_SOURCE_DIR}/src/information
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${GLOG_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

add_library(slam SHARED 
  ${PROJECT_SOURCE_DIR}/src/MultiMap.cpp
  ${PROJECT_SOURCE_DIR}/src/keyframe.cpp
  ${PROJECT_SOURCE_DIR}/src/parameters.cpp
  ${PROJECT_SOURCE_DIR}/src/imageProjection.cpp
  ${PROJECT_SOURCE_DIR}/src/featureExtraction.cpp
  ${PROJECT_SOURCE_DIR}/src/imuPreintegration.cpp
  ${PROJECT_SOURCE_DIR}/src/globalLocalization.cpp
  ${PROJECT_SOURCE_DIR}/src/utility/utility.cpp
  ${PROJECT_SOURCE_DIR}/src/information/SysInfo.cpp
  # ${PROJECT_SOURCE_DIR}/src/information/LocaInfo.cpp
  ${PROJECT_SOURCE_DIR}/src/information/tunning.cpp
  ${PROJECT_SOURCE_DIR}/src/ThirdParty/lidar/Scancontext.cpp
  ${PROJECT_SOURCE_DIR}/src/ThirdParty/camera/DBoW/BowVector.cpp
  ${PROJECT_SOURCE_DIR}/src/ThirdParty/camera/DBoW/FBrief.cpp
  ${PROJECT_SOURCE_DIR}/src/ThirdParty/camera/DBoW/FeatureVector.cpp
  ${PROJECT_SOURCE_DIR}/src/ThirdParty/camera/DBoW/QueryResults.cpp
  ${PROJECT_SOURCE_DIR}/src/ThirdParty/camera/DBoW/ScoringObject.cpp
  ${PROJECT_SOURCE_DIR}/src/ThirdParty/camera/DUtils/Random.cpp
  ${PROJECT_SOURCE_DIR}/src/ThirdParty/camera/DUtils/Timestamp.cpp
  ${PROJECT_SOURCE_DIR}/src/ThirdParty/camera/DVision/BRIEF.cpp
  ${PROJECT_SOURCE_DIR}/src/ThirdParty/camera/VocabularyBinary.cpp
)


add_executable(imageProjectionNode examples/node/imageProjectionNode.cpp)
add_dependencies(imageProjectionNode ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(imageProjectionNode slam ${catkin_LIBRARIES} ${THIRD_PARTY_LIBS})

add_executable(featureExtractionNode examples/node/featureExtractionNode.cpp)
add_dependencies(featureExtractionNode ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(featureExtractionNode slam ${catkin_LIBRARIES} ${THIRD_PARTY_LIBS})

add_executable(imuPreintegrationNode examples/node/imuPreintegrationNode.cpp)
add_dependencies(imuPreintegrationNode ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(imuPreintegrationNode slam ${catkin_LIBRARIES} ${THIRD_PARTY_LIBS})

add_executable(globalLocalizationNode examples/node/globalLocalizationNode.cpp)
add_dependencies(globalLocalizationNode ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(globalLocalizationNode PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(globalLocalizationNode slam ${catkin_LIBRARIES} ${THIRD_PARTY_LIBS} ${OpenMP_CXX_FLAGS})

add_executable(buildBinaryMap examples/build_map/buildBinaryMap.cpp)
add_dependencies(buildBinaryMap ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(buildBinaryMap PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(buildBinaryMap slam ${catkin_LIBRARIES} ${THIRD_PARTY_LIBS} ${OpenMP_CXX_FLAGS})

add_executable(sysInfoNode examples/node/sysInfoNode.cpp)
add_dependencies(sysInfoNode ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(sysInfoNode slam ${catkin_LIBRARIES} ${THIRD_PARTY_LIBS})

add_executable(tunningNode examples/node/tunningNode.cpp)
add_dependencies(tunningNode ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(tunningNode slam ${catkin_LIBRARIES} ${THIRD_PARTY_LIBS})