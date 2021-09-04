/*
* feature.h
* Location Provider feature
*
* Created by Tony Huang (cnwzhjs@gmail.com) at 2014-05-25
* Copyright 2014~2017 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include <rpos/rpos_config.h>

#include <rpos/core/feature.h>
#include <rpos/core/pose.h>
#include <rpos/features/location_provider/points_map.h>
#include <vector>

#include "map.h"

namespace rpos {
    namespace features {

        namespace detail {
            class LocationProviderImpl;
        }

        namespace location_provider {
            enum AuxLocalizationSource {
                AuxLocalizationSourceUwb
            };

            struct AuxLocalizationStatus {
                /**
                * @brief the source of the aux localization, currently only UWB is supported
                */
                AuxLocalizationSource source;

                /**
                * @brief is map update enabled
                */
                bool enableMapUpdate;

                /**
                * @brief is this localization source supported by the robot
                */
                bool isSupported;

                /**
                * @brief seconds since last update of this localization source, in seconds. -1 for never updated
                */
                int secondsSinceLastReport;
            };
        }

        class RPOS_CORE_API LocationProvider : public rpos::core::Feature{
        public:
            typedef detail::LocationProviderImpl impl_t;

            RPOS_OBJECT_CTORS_WITH_BASE(LocationProvider, rpos::core::Feature);
            LocationProvider(boost::shared_ptr<detail::LocationProviderImpl> impl);
            ~LocationProvider();

        public:
            std::vector<location_provider::MapType> getAvailableMaps();
            location_provider::Map getMap(location_provider::MapType type, core::RectangleF area, location_provider::MapKind kind);
            bool setMap(const location_provider::Map& map, location_provider::MapType type, location_provider::MapKind kind, bool partially);
            bool setMapAndPose(const core::Pose& pose, const location_provider::Map& map, const location_provider::MapType& type, const location_provider::MapKind& kind, bool partially);
            core::RectangleF getKnownArea(location_provider::MapType type, location_provider::MapKind kind);
            bool clearMap();
            bool clearMap(location_provider::MapKind kind);

            core::Location getLocation();
            core::Pose getPose();
            bool setPose(const core::Pose& pose);
            bool getMapLocalization();
            bool setMapLocalization(bool localization);
            bool getMapUpdate(location_provider::MapKind kind = location_provider::EXPLORERMAP);
            bool setMapUpdate(bool update, location_provider::MapKind kind = location_provider::EXPLORERMAP);
            bool getMapLoopClosure();
            bool setMapLoopClosure(bool loopClosure);
            int getLocalizationQuality();
            location_provider::PointPDF getAuxLocation();
            bool getHomePose(core::Pose&);
            bool setHomePose(core::Pose);
            location_provider::AuxLocalizationStatus getAuxLocalizationStatus(location_provider::AuxLocalizationSource source);
        };
        
    }
}
