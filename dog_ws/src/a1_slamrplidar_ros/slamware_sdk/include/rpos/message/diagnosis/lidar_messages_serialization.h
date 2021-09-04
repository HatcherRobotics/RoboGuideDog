/*
* lidar_message_serialization.h
* LIDAR message diagnosis serialization
*
* Created by Tony Huang (tony@slamtec.com) at 2017-3-15
* Copyright 2017 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include "../lidar_messages.h"
#include <rpos/system/diagnosis/diagnosis_serialization.h>

namespace rpos { namespace system { namespace diagnosis { namespace serialization {

    RP_SLAMWARE_TEST_LOGGING_DECLARE_SERIALIZER(message::lidar::LidarScanPoint);

    // The laser scan is a large data structure, so we specialize this template to compress the laser scan
    RP_SLAMWARE_TEST_LOGGING_DECLARE_SERIALIZER(std::vector<message::lidar::LidarScanPoint>);

} } } }
