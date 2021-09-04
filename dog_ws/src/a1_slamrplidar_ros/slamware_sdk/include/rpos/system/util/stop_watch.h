/*
* stop_watch.h
* Stop watch use boost::chrono::high_resolution_clock to measure a duration
*
* Created by Tony Huang (tony@slamtec.com) at 2016-11-22
* Copyright 2016 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include <cstdint>
#include <boost/chrono.hpp>
#include <rpos/core/rpos_core_config.h>

namespace rpos { namespace system { namespace util {

    class RPOS_CORE_API StopWatch
    {
    public:
        StopWatch();
        ~StopWatch();

    public:
        boost::chrono::high_resolution_clock::duration durationSinceStart() const;
        std::uint64_t usSinceStart() const;
        std::uint64_t msSinceStart() const;
        std::uint64_t secondsSinceStart() const;

    private:
        boost::chrono::high_resolution_clock::time_point startTime_;
    };

} } }
