/**
 * os.h
 * Operation platform isolation layer
 *
 * Created By Tony @ 2015-8-11
 * Copyright 2015 (c) Shanghai Slamtec Co., Ltd.
 */

#pragma once

#include <rpos/core/rpos_core_config.h>
#include <string>
#include <vector>

namespace rpos { namespace system { namespace os {

    RPOS_CORE_API std::vector<std::string> enumSerialPorts();

} } }
