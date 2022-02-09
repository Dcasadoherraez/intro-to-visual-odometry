#include "common_include.h"
#include "frontend.h"
#include "camera.h"
#include "frame.h"
#include "mappoint.h"



using namespace std;
using namespace cv;


Frontend::Frontend(Camera::Ptr cam_left, Camera::Ptr cam_right) {
    _cam.push_back(cam_left);
    _cam.push_back(cam_right);
}

double Frontend::GetDepth(double x_l, double x_r) {
    double d = (abs(_cam[1]->_baseline) * _cam[1]->_fy)/ (x_l - x_r);
    if (d && d > 0)
        return d;
    return 0;
}

void Frontend::ProjectFeatures(vector<KeyPoint> &features_left, vector<KeyPoint> &features_right, vector<DMatch> &matches) {
    
    if (matches.empty()) {
        cout << "No matches found!" << endl;
        return;
    }

    Mat disparityMap = GetDisparityMap();
    Mat depthMap = GetDepthMap(disparityMap);

    for (int i = 0; i < matches.size(); i++) {
        KeyPoint feat_left = features_left[matches[i].queryIdx];
        KeyPoint feat_right = features_right[matches[i].trainIdx];

        double x_l = feat_left.pt.x;
        double x_r = feat_right.pt.y;
        double depth = GetDepth(x_l, x_r);

        MapPoint::Ptr new_mappoint = MapPoint::CreateNewMappoint();
        Vec2 px;
        px << feat_left.pt.x, feat_left.pt.y;
        cout << "Depth: " << depth << endl;

        new_mappoint->_pos3d = _cam[0]->px2cam(px, depth);
    }

}

Mat Frontend::GetDisparityMap() {
    Mat cameraMatrix1, cameraMatrix2;
    eigen2cv(_cam[0]->K(), cameraMatrix1);
    eigen2cv(_cam[1]->K(), cameraMatrix2);

    // use SGBM algorithm to get Disparity map
    int minDisparity = 0;
    int sad_window = 6;
    int numDisparities = sad_window * 16;
    int blockSize = 11;
    int P1 = 8 * 3 * pow(sad_window, 2);
    int P2 = 32 * 3 * pow(sad_window, 2);
    int disp12MaxDiff = 0;
    int preFilterCap = 10;
    int uniquenessRatio = 0;
    int speckleWindowSize = 0;
    int speckleRange = 0;
    int mode = StereoSGBM::MODE_SGBM_3WAY;

    cv::Ptr<StereoSGBM> sgbm = StereoSGBM::create(minDisparity, numDisparities, blockSize,
                                         P1, P2, disp12MaxDiff, preFilterCap, uniquenessRatio,
                                         speckleWindowSize, speckleRange, mode);
    int rows, cols;
    rows = _current_frame->_img_left.rows;
    cols = _current_frame->_img_left.cols;
    Mat leftImgF, rightImgF, dispMap, depthMap;


    sgbm->compute(_current_frame->_img_left, _current_frame->_img_right, dispMap); 
    dispMap.convertTo(dispMap, CV_32F);
    dispMap = dispMap / 16;
    dispMap.setTo(0.1, dispMap == 0.0);
    dispMap.setTo(0.1, dispMap == -1);
    // DisplayMap(dispMap);

    return dispMap;

}

Mat Frontend::GetDepthMap(Mat disparityMap) {
    Mat depthMap;
    depthMap = (abs(_cam[1]->_baseline) * _cam[1]->_fx) / disparityMap;
    // DisplayMap(depthMap);

    return depthMap;
}

void Frontend::DisplayMap(Mat &input) {
    double minVal, maxVal, minIdx, maxIdx;
    minMaxIdx(input, &minVal, &maxVal);
    minMaxLoc(input, &minIdx, &maxIdx);
    cout << "Min: " << minVal << " Max: " << maxVal << endl;

    Mat displayMap;
    convertScaleAbs(input, displayMap, 255 / maxVal);

    imshow("Depth map", displayMap);
    waitKey(0);   
}