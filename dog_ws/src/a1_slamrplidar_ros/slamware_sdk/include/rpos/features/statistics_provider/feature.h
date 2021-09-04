#pragma once

#include <rpos/rpos_config.h>

#include <rpos/core/feature.h>
#include <rpos/core/geometry.h>
#include <vector>

namespace rpos { namespace features {

    namespace detail {
        class StatisticsProviderImpl;
    }

    namespace statistics_provider {

        enum StatisticsOption
        {
            StatisticsOptionSweepTime, // feature/sweep_opt only
            StatisticsOptionOdometry,
            StatisticsOptionSystemRunningTime,
            StatisticsOptionSytemTimeSinceEpoch
        };

    }

    class RPOS_CORE_API StatisticsProvider : public rpos::core::Feature
    {
    public:
        typedef detail::StatisticsProviderImpl impl_t;
        RPOS_OBJECT_CTORS_WITH_BASE(StatisticsProvider, rpos::core::Feature);
        StatisticsProvider(boost::shared_ptr<detail::StatisticsProviderImpl> impl);
        ~StatisticsProvider();

    public:
        int getSweepTimeMs();
        double getOdometry();
        double getSystemRunningTime();
        int getLocalTimeSinceEpoch();
    };

}}
