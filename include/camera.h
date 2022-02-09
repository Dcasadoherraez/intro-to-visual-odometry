#pragma once

#ifndef CAMERA_H
#define CAMERA_H

#include "common_include.h"

using namespace std;
using namespace cv;


class Camera {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef shared_ptr<Camera> Ptr;

    // intrinsics
    double _fx = 0, _fy = 0, _cx = 0, _cy = 0, _baseline = 0;

    // extrinsics
    Sophus::SE3d _pose;

    // camera constructors
    Camera() {}

    Camera(double fx, double fy, double cx, double cy, double baseline, Sophus::SE3d &pose);

    Mat33 K() const {
        Mat33 K;
        K << _fx, 0, _cx, 0, _fy, _cy, 0, 0, 1;
        return K;
    }

    // coordinate transformations
    Vec3 world2cam(const Vec3 &p_w, const Sophus::SE3d &T_c_w);

    Vec3 cam2world(const Vec3 &p_c, const Sophus::SE3d &T_c_w);

    Vec2 cam2px(const Vec3 &p_c);

    Vec3 px2cam(const Vec2 &p_p, double depth = 1);

    Vec3 px2world(const Vec2 &p_p, const Sophus::SE3d &T_c_w, double depth = 1);

    Vec2 world2px(const Vec3 &p_w, const Sophus::SE3d &T_c_w);

    double GetDepth(double x_l, double x_r);
};

#endif