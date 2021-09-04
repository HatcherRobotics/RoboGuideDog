#pragma once

namespace rpos {
    namespace core {

        enum InternalSystemEvent
        {
            InternalSystemEventRelocalizationFail = 1,
            InternalSystemEventBackHomeFail,
            InternalSystemEventMapUpdateEnable,
            InternalSystemEventMapUpdateDisable
        };

    }
}