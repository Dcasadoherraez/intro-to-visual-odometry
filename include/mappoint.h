#pragma once

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "common_include.h"
using namespace std;

class MapPoint {
public:
    typedef shared_ptr<MapPoint> Ptr;

    unsigned long int _id;
    Vec3 _pos3d;

    MapPoint() {}

    MapPoint(unsigned long int id, Vec3 pos);

    static MapPoint::Ptr CreateNewMappoint();

    Vec3 GetWorldPos();

};  

#endif