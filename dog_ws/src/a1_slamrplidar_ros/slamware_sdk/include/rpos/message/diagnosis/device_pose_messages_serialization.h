#pragma once

#include "../base_messages.h"
#include <rpos/core/pose.h>
#include <rpos/system/diagnosis/diagnosis_serialization.h>

namespace rpos {
    namespace system {
        namespace diagnosis {
            namespace serialization {

                RP_SLAMWARE_TEST_LOGGING_DECLARE_SERIALIZER(rpos::core::Location);

            }
        }
    }
}