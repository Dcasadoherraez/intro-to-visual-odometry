#include "camera.h"
#include "common_include.h"

using namespace std;
using namespace cv;


Camera::Camera(double fx, double fy, double cx, double cy, double baseline, Sophus::SE3d &pose)
        : _fx(fx), _fy(fy), _cx(cx), _cy(cy), _baseline(baseline)  {}

Vec3 Camera::world2cam(const Vec3 &p_w, const Sophus::SE3d &T_c_w) {
    return _pose * T_c_w * p_w;
}

Vec3 Camera::cam2world(const Vec3 &p_c, const Sophus::SE3d &T_c_w) {
    return T_c_w.inverse() * _pose.inverse() * p_c;
}

Vec2 Camera::cam2px(const Vec3 &p_c) {
    return Vec2(
        _fx * p_c(0,0) / p_c(2, 0) + _cx,
        _fy * p_c(1,0) / p_c(2, 0) + _cy
    );
}

Vec3 Camera::px2cam(const Vec2 &p_p, double depth) {
    return Vec3(
        (p_p(0,0) - _cx) * depth / _fx,
        (p_p(1,0) - _cy) * depth / _fy,
        depth
    );
}

Vec3 Camera::px2world(const Vec2 &p_p, const Sophus::SE3d &T_c_w, double depth) {
    return cam2world(px2cam(p_p, depth), T_c_w);
}

Vec2 Camera::world2px(const Vec3 &p_w, const Sophus::SE3d &T_c_w){
    return cam2px(world2cam(p_w, T_c_w));
}

