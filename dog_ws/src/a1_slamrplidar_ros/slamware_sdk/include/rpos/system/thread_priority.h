/*
* thread_priority.h
* Thread priority manipulation utility functions
* Copyed from infra/hal/ *thread.hpp
*
* Created by Tony Huang (tony@slamtec.com) at 2017-8-7
* Copyright 2017 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include <boost/thread.hpp>

namespace rpos { namespace system {

    enum ThreadPriority {
        ThreadPriorityRealtime = 0,
        ThreadPriorityHigh = 1,
        ThreadPriorityNormal = 2,
        ThreadPriorityLow = 3,
        ThreadPriorityIdle = 4,
    };

    bool set_current_thread_priority(ThreadPriority priority);

} }
