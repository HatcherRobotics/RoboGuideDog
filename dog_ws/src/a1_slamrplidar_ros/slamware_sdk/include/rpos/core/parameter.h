#pragma once

#include <rpos/core/rpos_core_config.h>

#include <cstdint>

namespace rpos { namespace core {

    struct RPOS_CORE_API SlamcoreShutdownParam
    {
        std::uint32_t restartTimeIntervalMinute;
        std::uint32_t shutdownTimeIntervalMinute;

        SlamcoreShutdownParam();
    };

}}
