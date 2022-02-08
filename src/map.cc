#include "common_include.h"
#include "frame.h"
#include "map.h"
#include "camera.h"
#include "mappoint.h"

using namespace std;
using namespace cv;

Map::Map(int id) : _id(id) {}

Map::Ptr Map::InitMap() {
    static long factory_id = 0;
    Map::Ptr new_map(new Map);
    new_map->_id = factory_id++;
    cout << "Map initialized with id " << factory_id << endl;
    return new_map;
}

vector<MapPoint::Ptr> Map::GetAllMapPoints()
{
    //boost::mutex::scoped_lock lock(mMutexMap);
    vector<MapPoint::Ptr> allMPs = {};
    for (pair<unsigned long, MapPoint::Ptr> i : _MapPoints) {
        allMPs.push_back(i.second);
    }
    return allMPs;
}
