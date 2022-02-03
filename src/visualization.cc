#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "common_include.h"
#include "visualization.h"


bool Visualization::map2pointcloud() {
    PointCloud::Ptr cloud;

    for (auto mp : _map->_MapPoints) {
        pcl::PointXYZ pt;
        pt.x = mp.second->_pos3d[0];
        pt.y = mp.second->_pos3d[1];
        pt.z = mp.second->_pos3d[2];

        cloud->push_back(pt);
    }

    _pcMap = cloud;

    return true;
}

Visualization::Ptr Visualization::InitVisualization(int argc, char **argv) {
    ros::init (argc, argv, "pub_pcl");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<PointCloud> ("orb_features", 1);

    

    ros::Rate loop_rate(4);
    while (nh.ok())
    {
        
        if (!map2pointcloud()) {
            cout << "Error converting map to pointcloud" << endl;
        }

        _pcMap->header.frame_id = "car";

        pcl_conversions::toPCL(ros::Time::now(), _pcMap->header.stamp);
        pub.publish (_pcMap);
        ros::spinOnce ();
        loop_rate.sleep ();
    }
}