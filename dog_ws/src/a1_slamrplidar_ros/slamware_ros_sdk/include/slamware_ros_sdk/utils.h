
#pragma once

#include <cstdint>

#include <boost/optional.hpp>

namespace slamware_ros_sdk {

    template<class ValT, class RosMsgT>
    inline void optionalToRosMsg(const boost::optional<ValT> & optVal, RosMsgT& rosMsg)
    {
        if (optVal)
        {
            rosMsg.is_valid = true;
            rosMsg.value = (*optVal);
        }
        else
        {
            rosMsg = RosMsgT();
        }
    }

    template<class ValT, class RosMsgT>
    inline void rosMsgToOptional(const RosMsgT& rosMsg, boost::optional<ValT> & optVal)
    {
        if (rosMsg.is_valid)
        {
            optVal = rosMsg.value;
        }
        else
        {
            optVal.reset();
        }
    }

}
