#pragma once

#ifndef MAP_H
#define MAP_H

#include "common_include.h"
#include "mappoint.h"
#include "frame.h"

using namespace std;
using namespace cv;

class Map {
public:
    typedef shared_ptr<Map> Ptr;

    int _id;
    unordered_map<unsigned long int, MapPoint::Ptr> _MapPoints;

    Map() {}

    Map(int id);

    static Map::Ptr InitMap();


};

#endif