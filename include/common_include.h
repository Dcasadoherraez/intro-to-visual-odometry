#pragma once
#ifndef MYSLAM_COMMON_INCLUDE_H
#define MYSLAM_COMMON_INCLUDE_H

// std
#include <atomic>
#include <condition_variable>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <typeinfo>
#include <unordered_map>
#include <vector>
#include <stdlib.h>
#include <unordered_map>
#include <fstream>

// define the commonly included file to avoid a long include list
#include <Eigen/Core>
#include <Eigen/Geometry>

// typedefs for eigen
// double matricies
typedef Eigen::Matrix<double, 3, 3> Mat33;


// float matricies
typedef Eigen::Matrix<float, 3, 3> Mat33f;

// double vectors
typedef Eigen::Matrix<double, 3, 1> Vec3;
typedef Eigen::Matrix<double, 2, 1> Vec2;

// float vectors
typedef Eigen::Matrix<float, 3, 1> Vec3f;
typedef Eigen::Matrix<float, 2, 1> Vec2f;

// for Sophus
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

// typedef Sophus::SE3d SE3;
// typedef Sophus::SO3d SO3;

// for cv
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/features2d.hpp>
#include "opencv4/opencv2/highgui.hpp"
// glog
// #include <glog/logging.h>

#endif  // MYSLAM_COMMON_INCLUDE_H