
#pragma once

#include <rpos/core/rpos_core_config.h>
#include <string>
#include <boost/uuid/uuid.hpp>

namespace rpos { namespace system { namespace util {

    RPOS_CORE_API boost::uuids::uuid generateUuid();
    RPOS_CORE_API std::string generateUuidString();

}}}
