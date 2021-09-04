/*
* loop_buffer.h
* Loop Buffer
*
* Created by Tony Huang (tony@slamtec.com) at 2017-5-14
* Copyright 2017 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include <boost/circular_buffer.hpp>
#include <cstdint>

namespace rpos { namespace system { namespace util {

    typedef boost::circular_buffer<std::uint8_t> LoopBuffer;

} } }
