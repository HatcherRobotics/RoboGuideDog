#pragma once

#include <rpos/core/rpos_core_config.h>
#include <rpos/core/geometry.h>

namespace rpos { namespace core {

    enum DoorType
    {
        DoorTypeDoor
        , DoorTypeCandidateDoor
    };

    struct RPOS_CORE_API DoorDesc
    {
        rpos::core::Vector2f start;
        rpos::core::Vector2f end;
        DoorType type;
    };

}}
