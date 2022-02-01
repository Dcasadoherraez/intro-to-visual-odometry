#pragma once

#ifndef FRONTEND_H
#define FRONTEND_H

#include "common_include.h"
#include "camera.h"
#include "frame.h"
#include "map.h"

using namespace std;
using namespace cv;

class Frontend {
public:
    typedef shared_ptr<Frontend> Ptr;

    vector<Camera> _cam;
    Frame::Ptr _current_frame;
    Map::Ptr _map;

    Frontend() {}

    Frontend(Camera cam_left, Camera cam_right);

    void InitMap();

    void ProjectFeatures(vector<KeyPoint> features_left, vector<KeyPoint> features_right, vector<DMatch> matches);

    double GetDepth(double x_l, double x_r);

};

#endif