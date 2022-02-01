#include "common_include.h"
#include "frontend.h"
#include "camera.h"
#include "frame.h"
#include "mappoint.h"

using namespace std;
using namespace cv;


Frontend::Frontend(Camera cam_left, Camera cam_right) {
    _cam.push_back(cam_left);
    _cam.push_back(cam_right);
}

double Frontend::GetDepth(double x_l, double x_r) {
    return (_cam[0]._baseline * (_cam[0]._fx + _cam[0]._fy) / 2 ) / (x_l - x_r);
}

void Frontend::ProjectFeatures(vector<KeyPoint> features_left, vector<KeyPoint> features_right, vector<DMatch> matches) {
    
    if (matches.empty()) {
        cout << "No matches found!" << endl;
        return;
    }

    for (int i = 0; i < matches.size(); i++) {
        KeyPoint feat_right = features_right[matches[i].queryIdx];
        KeyPoint feat_left = features_left[matches[i].trainIdx];

        double x_l = _current_frame->_img_left.at<double>(feat_left.pt.x, feat_left.pt.y);
        double x_r = _current_frame->_img_right.at<double>(feat_right.pt.x, feat_right.pt.y);

        double depth = GetDepth(x_l, x_r);

        MapPoint::Ptr new_mappoint = MapPoint::CreateNewMappoint();
        Vec2 px;
        px << feat_left.pt.x, feat_left.pt.y;
        new_mappoint->_pos3d = _cam[0].px2cam(px, depth);
    }

    

    return; 
}