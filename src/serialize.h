/*
This file contains the necessary serialize functions for complex datatypes
used by ORBSLAM2 and which are not available in boost serialization library
*/
#pragma once

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/set.hpp>
#include <boost/dynamic_bitset/serialization.hpp>
#include <boost/serialization/shared_ptr.hpp>

#include <opencv2/core/core.hpp>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <gtsam/base/Vector.h>

// #include "Thirdparty/DBoW2/DBoW2/BowVector.h"
// #include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

struct PointXYZIRPYT;

BOOST_SERIALIZATION_SPLIT_FREE(::cv::Mat)
namespace boost{
    namespace serialization {

    /*
    template<class Archive>
    void serialize(Archive &ar, DBoW2::BowVector &BowVec, const unsigned int file_version)
    {
        //Derived classes should include serializations of their base classes.
        ar & boost::serialization::base_object<DBoW2::BowVector::super>(BowVec);
    }
    template<class Archive>
    void serialize(Archive &ar, DBoW2::FeatureVector &FeatVec, const unsigned int file_version)
    {
        ar & boost::serialization::base_object<DBoW2::FeatureVector::super>(FeatVec);
    }
    */

    // OpenCV 数据类型
    template<class Archive>
    void save(Archive &ar, const ::cv::Mat &m, const unsigned int file_version)
    {
        cv::Mat m_ = m;
        if (!m.isContinuous())
            m_ = m.clone();
        size_t elem_size = m_.elemSize();
        size_t elem_type = m_.type();
        ar & m_.cols;
        ar & m_.rows;
        ar & elem_size;
        ar & elem_type;

        const size_t data_size = m_.cols * m_.rows * elem_size;

        ar & boost::serialization::make_array(m_.ptr(), data_size);
    }

    template<class Archive>
    void load(Archive & ar, ::cv::Mat& m, const unsigned int version)
    {
        int cols, rows;
        size_t elem_size, elem_type;

        ar & cols;
        ar & rows;
        ar & elem_size;
        ar & elem_type;

        m.create(rows, cols, elem_type);
        size_t data_size = m.cols * m.rows * elem_size;

        ar & boost::serialization::make_array(m.ptr(), data_size);
    }

    template<class Archive>
    void serialize(Archive &ar, ::cv::KeyPoint &kf, const unsigned int file_version)
    {
        ar & kf.angle;
        ar & kf.class_id;
        ar & kf.octave;
        ar & kf.response;
        ar & kf.response;
        ar & kf.pt.x;
        ar & kf.pt.y;
    }

    template<class Archive>
    void serialize(Archive &ar, ::cv::Point3f &pt, const unsigned int file_version)
    {
        ar & pt.x;
        ar & pt.y;
        ar & pt.z;
    }

    template<class Archive>
    void serialize(Archive &ar, ::cv::Point2f &pt, const unsigned int file_version)
    {
        ar & pt.x;
        ar & pt.y;
    }

    // Eigen 数据类型
    template<class Archive>
    void serialize(Archive &ar, Eigen::Matrix3d &m, const unsigned int file_version)
    {
        ar & boost::serialization::make_array(m.data(), m.size());
    }

    // 与gtsam重复了，在<gtsam/base/Vector.h>中
    // template<class Archive>
    // void serialize(Archive &ar, Eigen::Vector3d &m, const unsigned int file_version)
    // {
    //     ar & boost::serialization::make_array(m.data(), m.size());
    // }

    template<class Archive>
    void serialize(Archive &ar, Eigen::Matrix<double, 8, 1> &m, const unsigned int file_version)
    {
        ar & boost::serialization::make_array(m.data(), m.size());
    }

    // PCL 数据类型
    template<class Archive>
    void serialize(Archive & ar, pcl::PCLHeader & g, const unsigned int version)
    {
        ar & g.seq;
        ar & g.stamp;
        ar & g.frame_id;
    }

    template<class Archive>
    void serialize(Archive & ar, pcl::PointXYZI & g, const unsigned int version)
    {
        ar & g.x;
        ar & g.y;
        ar & g.z;
        ar & g.intensity;
    }

    template<class Archive>
    void serialize(Archive & ar, PointXYZIRPYT & g, const unsigned int version)
    {
        ar & g.x;
        ar & g.y;
        ar & g.z;
        ar & g.intensity;
        ar & g.roll;
        ar & g.pitch;
        ar & g.yaw;
        ar & g.time;
    }

    template<class Archive>
    void serialize(Archive & ar, pcl::PointCloud<pcl::PointXYZI> & g, const unsigned int version)
    {
        ar & g.header;
        ar & g.points;
        ar & g.height;
        ar & g.width;
        ar & g.is_dense;
    }

    template<class Archive>
    void serialize(Archive & ar, pcl::PointCloud<PointXYZIRPYT> & g, const unsigned int version)
    {
        ar & g.header;
        ar & g.points;
        ar & g.height;
        ar & g.width;
        ar & g.is_dense;
    }

    }
}
