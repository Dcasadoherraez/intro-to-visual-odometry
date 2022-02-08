#pragma once

#ifndef MAPPUBLISHER_H
#define MAPPUBLISHER_H

#include "common_include.h"

#include "mappoint.h"
#include "map.h"

using namespace std;

class MapPublisher
{
public:
    typedef shared_ptr<MapPublisher> Ptr;

    MapPublisher(Map::Ptr pMap);

    Map::Ptr mpMap;

    void Refresh();
    void PublishMapPoints(const std::vector<MapPoint::Ptr> &vpMPs);
    void PublishCurrentCamera(const Sophus::SE3d &Tcw);
    void SetCurrentCameraPose(const Sophus::SE3d &Tcw);
    cv::Mat SE3dtoMat(Sophus::SE3d Tcw);

private:

    Sophus::SE3d GetCurrentCameraPose();
    bool isCamUpdated();
    void ResetCamFlag();

    ros::NodeHandle nh;
    ros::Publisher publisher;

    visualization_msgs::Marker mPoints;
    visualization_msgs::Marker mCovisibilityGraph;
    visualization_msgs::Marker mMST;
    visualization_msgs::Marker mCurrentCamera;

    float fCameraSize;
    float fPointSize;

    Sophus::SE3d mCameraPose;
    bool mbCameraUpdated;

    // boost::mutex mMutexCamera;
};

#endif