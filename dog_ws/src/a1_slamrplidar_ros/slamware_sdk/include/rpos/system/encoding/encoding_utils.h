/*
* encoding_utils.h
* Some utility functions for encoding
*
* Created by Tony Huang (tony@slamtec.com) at 2018-10-05
* Copyright 2018 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include "i_encoder.h"
#include <vector>
#include <string>

namespace rpos { namespace system { namespace encoding {

    RPOS_CORE_API std::vector<std::uint8_t> encode(const std::vector<std::uint8_t>& src, IEncoder& encoder);
    RPOS_CORE_API std::string encode(const std::string& src, IEncoder& encoder);
    RPOS_CORE_API bool encode(const std::vector<std::uint8_t>& src, IEncoder& encoder, std::vector<std::uint8_t>& dest);
    RPOS_CORE_API bool encode(const std::string& src, IEncoder& encoder, std::string& dest);

} } }
