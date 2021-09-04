#pragma once

#include <string>

#include <rpos/core/geometry.h>

namespace rpos {
    namespace features {

        enum RegionType
        {
            RegionTypeSweep,
            RegionTypeRestricted,
        };

        struct Region
        {
            size_t id;
            std::string label;
            rpos::core::Vector2f points[4];
            float rad;
            RegionType type;
            size_t number;
        };

    }
}