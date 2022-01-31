#pragma once

#ifndef FRAME_H
#define FRAME_H

#include <stdlib.h>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/features2d.hpp>

using namespace std;
using namespace cv;


class Frame {
public:
    int _id;
    Mat _img;
    double _timestamp;

    // ORB 2D features
    vector<KeyPoint> _features_left;
    vector<KeyPoint> _features_right;

    Frame(int id, Mat img, double timestamp);

    // get feature keypoints from one image
    vector<KeyPoint> GetFeatures(Mat image);

    // assign the features to the instance attributes
    void GetLeftRightFeatures(Mat img_left, Mat img_right) {

    // match features from left and right images
    void MatchFeatures(Mat descriptors_left, Mat descriptors_right);

    // extract and match features
    void ExtractAndMatch(Mat image_left, Mat image_right);
};

#endif