#pragma once

#include <rpos/message/message.h>
#include <vector>

namespace rpos { namespace message { namespace lidar {

    struct LidarScanPoint
    {
        float dist;     // in meter
        float angle;    // in degree, 0 expected to be the front of LIDAR, and increase by rotate in counter-clockwise (left-hand system)
        bool valid;     // if the lidar scan point is valid or not (for eg. no obstacle detected)
    };

    typedef std::vector<LidarScanPoint> LidarScan;

}}}