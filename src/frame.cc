#include "common_include.h"
#include "frame.h"

using namespace std;
using namespace cv;

Frame::Frame(int id, Mat img_left, Mat img_right, double timestamp) : _id(id), _img_left(img_left), _img_right(img_right), _timestamp(timestamp) {}

void Frame::GetFeatures() {

    vector<KeyPoint> keypoints_left, keypoints_right;
    Mat descriptors_left, descriptors_right;
    cv::Ptr<FeatureDetector> detector = ORB::create();
    cv::Ptr<DescriptorExtractor> descriptor = ORB::create();

    // compute the descriptors in both images
    detector->detect(_img_left, keypoints_left);
    detector->detect(_img_right, keypoints_right);

    descriptor->compute(_img_left, keypoints_left, descriptors_left);
    descriptor->compute(_img_right, keypoints_right, descriptors_right);

    _features_left = keypoints_left;
    _features_right = keypoints_right;

    _descriptors_left = descriptors_left;
    _descriptors_right = descriptors_right;

    return;
}

vector<DMatch> Frame::MatchFeatures() {
    vector<DMatch> matches;
    cv::Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    matcher->match(_descriptors_left, _descriptors_right, matches);

    auto min_max = minmax_element(matches.begin(), matches.end(),
                                [](const DMatch &m1, const DMatch &m2) { return m1.distance < m2.distance; });
    double min_dist = min_max.first->distance;
  
    vector<DMatch> good_matches;
    for (int i = 0; i < _descriptors_left.rows; i++) {
        if (matches[i].distance <= max(2 * min_dist, 30.0)) 
            good_matches.push_back(matches[i]);
    }
    cout << "All matches: " << to_string(matches.size()) << " Good matches: " << to_string(good_matches.size()) << endl;

    return good_matches;
}

void Frame::DisplayMatches() {
    vector<DMatch> matches = Frame::MatchFeatures();
    
    Mat img_match;
    drawMatches(_img_left, _features_left, _img_right, _features_right, matches, img_match);
    imshow("all matches", img_match);
    waitKey(0);

}
