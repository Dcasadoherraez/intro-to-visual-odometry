#pragma once

#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include "common_include.h"
#include "map.h"

using namespace std;

class Visualization {
public:
    typedef shared_ptr<Visualization> Ptr;
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

    string _tf_name = "robot";
    Map::Ptr _map;
    PointCloud::Ptr _pcMap;

    Visualization(string tf) : _tf_name(tf) {}

    Visualization::Ptr InitVisualization(int argc, char **argv);

    bool map2pointcloud();
};  

#endif