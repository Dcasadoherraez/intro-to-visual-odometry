#include "common_include.h"
#include "mappoint.h"

MapPoint::MapPoint(unsigned long int id, Vec3 pos) : _id(id), _pos3d(pos) {}


MapPoint::Ptr MapPoint::CreateNewMappoint() {
    static long factory_id = 0;
    MapPoint::Ptr new_mappoint(new MapPoint);
    new_mappoint->_id = factory_id++;
    return new_mappoint;
}

Vec3 MapPoint::GetWorldPos()
{
    // boost::mutex::scoped_lock lock(mMutexPos);
    return _pos3d;
}