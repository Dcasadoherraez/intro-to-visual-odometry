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
