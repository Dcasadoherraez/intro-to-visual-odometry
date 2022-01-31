#include <stdlib.h>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/features2d.hpp>
#include "opencv4/opencv2/highgui.hpp"


#include "frame.h"

using namespace std;
using namespace cv;

Frame::Frame(int id, Mat img, double timestamp) : _id(id), _img(img), _timestamp(timestamp) {}

void Frame::GetLeftRightFeatures(Mat img_left, Mat img_right) {
    _features_left = Frame::GetFeatures(img_left);
    _features_right = Frame::GetFeatures(img_right);
}

vector<KeyPoint> Frame::GetFeatures(Mat img) {
    vector<KeyPoint> keypoints;
    Mat descriptors;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();

    detector->detect(img, keypoints);
    descriptor->compute(img, keypoints, descriptors);

    Mat outimg;
    drawKeypoints(img, keypoints, outimg, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    imshow("ORB features", outimg);
    waitKey(0);

    return keypoints;
}

void Frame::MatchFeatures(Mat descriptors_left, Mat descriptors_right) {
    vector<DMatch> matches;
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    matcher->match(descriptors_left, descriptors_right, matches);

    auto min_max = minmax_element(matches.begin(), matches.end(),
                                [](const DMatch &m1, const DMatch &m2) { return m1.distance < m2.distance; });
    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;
  
    vector<DMatch> good_matches;
    for (int i = 0; i < descriptors_left.rows; i++) {
        if (matches[i].distance <= max(2 * min_dist, 30.0)) 
            good_matches.push_back(matches[i]);
    }
}

void Frame::ExtractAndMatch(Mat image_left, Mat image_right) {

}