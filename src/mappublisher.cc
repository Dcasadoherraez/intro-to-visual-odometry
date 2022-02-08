#include "common_include.h"
#include "mappublisher.h"
#include "mappoint.h"
#include "map.h"

MapPublisher::MapPublisher(Map::Ptr pMap) : mpMap(pMap), mbCameraUpdated(false)
{
    const char* MAP_FRAME_ID = "/ORB_SLAM/World";
    const char* POINTS_NAMESPACE = "MapPoints";
    // const char* GRAPH_NAMESPACE = "Graph";
    const char* CAMERA_NAMESPACE = "Camera";

    //Configure MapPoints
    fPointSize=0.01;
    mPoints.header.frame_id = MAP_FRAME_ID;
    mPoints.ns = POINTS_NAMESPACE;
    mPoints.id=0;
    mPoints.type = visualization_msgs::Marker::POINTS;
    mPoints.scale.x=fPointSize;
    mPoints.scale.y=fPointSize;
    mPoints.pose.orientation.w=1.0;
    mPoints.action=visualization_msgs::Marker::ADD;
    mPoints.color.a = 1.0;

    //Configure Current Camera
    mCurrentCamera.header.frame_id = MAP_FRAME_ID;
    mCurrentCamera.ns = CAMERA_NAMESPACE;
    mCurrentCamera.id=4;
    mCurrentCamera.type = visualization_msgs::Marker::LINE_LIST;
    mCurrentCamera.scale.x=0.01;//0.2; 0.03
    mCurrentCamera.pose.orientation.w=1.0;
    mCurrentCamera.action=visualization_msgs::Marker::ADD;
    mCurrentCamera.color.g=1.0f;
    mCurrentCamera.color.a = 1.0;

    //Configure Publisher
    publisher = nh.advertise<visualization_msgs::Marker>("ORB_SLAM/Map", 10);

    publisher.publish(mPoints);
    publisher.publish(mCovisibilityGraph);
    publisher.publish(mCurrentCamera);
}

void MapPublisher::Refresh()
{
    if(isCamUpdated())
    {
       Sophus::SE3d Tcw = GetCurrentCameraPose();
       PublishCurrentCamera(Tcw);
       ResetCamFlag();
    }
    vector<MapPoint::Ptr> vMapPoints = mpMap->GetAllMapPoints();

    PublishMapPoints(vMapPoints);   

}

void MapPublisher::PublishMapPoints(const vector<MapPoint::Ptr> &vpMPs)
{
    mPoints.points.clear();

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        // if(vpMPs[i]->isBad())
        //     continue;
        geometry_msgs::Point p;
        Vec3 pos = vpMPs[i]->GetWorldPos();
        p.x=pos[0];
        p.y=pos[1];
        p.z=pos[2];

        mPoints.points.push_back(p);
    }

    mPoints.header.stamp = ros::Time::now();
    publisher.publish(mPoints);
}

cv::Mat MapPublisher::SE3dtoMat(Sophus::SE3d Tcw) {
     cv::Mat Tcw_conv;
     cv::eigen2cv(Tcw.matrix(), Tcw_conv);
     return Tcw_conv;
}

void MapPublisher::PublishCurrentCamera(const Sophus::SE3d &Tcw_sophus)
{
    mCurrentCamera.points.clear();

    float d = fCameraSize;

    //Camera is a pyramid. Define in camera coordinate system
    cv::Mat o = (cv::Mat_<float>(4,1) << 0, 0, 0, 1);
    cv::Mat p1 = (cv::Mat_<float>(4,1) << d, d*0.8, d*0.5, 1);
    cv::Mat p2 = (cv::Mat_<float>(4,1) << d, -d*0.8, d*0.5, 1);
    cv::Mat p3 = (cv::Mat_<float>(4,1) << -d, -d*0.8, d*0.5, 1);
    cv::Mat p4 = (cv::Mat_<float>(4,1) << -d, d*0.8, d*0.5, 1);

    cv::Mat Tcw = SE3dtoMat(Tcw_sophus);
    cv::Mat Twc = Tcw.inv();
    cv::Mat ow = Twc*o;
    cv::Mat p1w = Twc*p1;
    cv::Mat p2w = Twc*p2;
    cv::Mat p3w = Twc*p3;
    cv::Mat p4w = Twc*p4;

    geometry_msgs::Point msgs_o,msgs_p1, msgs_p2, msgs_p3, msgs_p4;
    msgs_o.x=ow.at<float>(0);
    msgs_o.y=ow.at<float>(1);
    msgs_o.z=ow.at<float>(2);
    msgs_p1.x=p1w.at<float>(0);
    msgs_p1.y=p1w.at<float>(1);
    msgs_p1.z=p1w.at<float>(2);
    msgs_p2.x=p2w.at<float>(0);
    msgs_p2.y=p2w.at<float>(1);
    msgs_p2.z=p2w.at<float>(2);
    msgs_p3.x=p3w.at<float>(0);
    msgs_p3.y=p3w.at<float>(1);
    msgs_p3.z=p3w.at<float>(2);
    msgs_p4.x=p4w.at<float>(0);
    msgs_p4.y=p4w.at<float>(1);
    msgs_p4.z=p4w.at<float>(2);

    mCurrentCamera.points.push_back(msgs_o);
    mCurrentCamera.points.push_back(msgs_p1);
    mCurrentCamera.points.push_back(msgs_o);
    mCurrentCamera.points.push_back(msgs_p2);
    mCurrentCamera.points.push_back(msgs_o);
    mCurrentCamera.points.push_back(msgs_p3);
    mCurrentCamera.points.push_back(msgs_o);
    mCurrentCamera.points.push_back(msgs_p4);
    mCurrentCamera.points.push_back(msgs_p1);
    mCurrentCamera.points.push_back(msgs_p2);
    mCurrentCamera.points.push_back(msgs_p2);
    mCurrentCamera.points.push_back(msgs_p3);
    mCurrentCamera.points.push_back(msgs_p3);
    mCurrentCamera.points.push_back(msgs_p4);
    mCurrentCamera.points.push_back(msgs_p4);
    mCurrentCamera.points.push_back(msgs_p1);

    mCurrentCamera.header.stamp = ros::Time::now();

    publisher.publish(mCurrentCamera);
}

void MapPublisher::SetCurrentCameraPose(const Sophus::SE3d &Tcw)
{
    // boost::mutex::scoped_lock lock(mMutexCamera);
    mCameraPose = Tcw;
    mbCameraUpdated = true;
}

Sophus::SE3d MapPublisher::GetCurrentCameraPose()
{
    // boost::mutex::scoped_lock lock(mMutexCamera);
    return mCameraPose;
}

bool MapPublisher::isCamUpdated()
{
    // boost::mutex::scoped_lock lock(mMutexCamera);
    return mbCameraUpdated;
}

void MapPublisher::ResetCamFlag()
{
    // boost::mutex::scoped_lock lock(mMutexCamera);
    mbCameraUpdated = false;
}