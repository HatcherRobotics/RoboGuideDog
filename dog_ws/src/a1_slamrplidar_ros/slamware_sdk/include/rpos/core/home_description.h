#pragma once

#include <rpos/rpos_config.h>
#include <rpos/core/rpos_core_config.h>
#include <rpos/core/pose.h>


namespace rpos { namespace core {

    struct RPOS_CORE_API HomeDesc
    {
        Pose pose;
        double length;
    };

}}