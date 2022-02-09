#pragma once

#ifndef FRAME_H
#define FRAME_H

#include "common_include.h"

using namespace std;
using namespace cv;


class Frame {
public:
    typedef shared_ptr<Frame> Ptr;

    int _id;
    Mat _img_left;
    Mat _img_right;
    double _timestamp;

    // ORB 2D features
    vector<KeyPoint> _features_left;
    Mat _descriptors_left;
    vector<KeyPoint> _features_right;
    Mat _descriptors_right;

    Frame() {}

    Frame(int id, Mat img_left, Mat img_right, double timestamp);

    // get feature keypoints from one image
    void GetFeatures();

    // match features from left and right images
    vector<DMatch> MatchFeatures() ;

    // extract and match features
    void ExtractAndMatch(Mat image_left, Mat image_right);

    // display matches between frames
    void DisplayMatches();
};

#endif