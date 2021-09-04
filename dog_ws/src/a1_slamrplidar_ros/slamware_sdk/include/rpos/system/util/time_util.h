/*
* time_util.h
* Time utilities
*
* Created by Tony Huang (cnwzhjs@gmail.com) at 2014-10-14
* Copyright 2014 (c) www.robopeak.com
*/

#pragma once

#include <rpos/core/rpos_core_config.h>

namespace rpos { namespace system { namespace util {

    namespace high_resolution_clock {
        RPOS_CORE_API long long get_time_in_us();
        RPOS_CORE_API long long get_time_in_ms();
        RPOS_CORE_API long long get_time_in_s();
    } 

    namespace thread_clock {
        RPOS_CORE_API long long get_time_in_us();
        RPOS_CORE_API long long get_time_in_ms();
        RPOS_CORE_API long long get_time_in_s();
    }

} } }
