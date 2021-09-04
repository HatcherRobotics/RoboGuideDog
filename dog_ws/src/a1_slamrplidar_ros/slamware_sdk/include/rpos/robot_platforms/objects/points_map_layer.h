#pragma once

#include <rpos/core/pose.h>
#include <rpos/system/types.h>
#include <rpos/features/location_provider/points_map.h>

#include <rpos/robot_platforms/objects/metadata.h>
#include <rpos/robot_platforms/objects/map_layer.h>

#include <vector>


namespace rpos { namespace robot_platforms { namespace objects {


    class RPOS_SLAMWARE_API PointsMapLayer : public MapLayer
    {
    public:
        static const char* const Type;

        PointsMapLayer();
        ~PointsMapLayer();

    public:
        virtual void clear(void);

    public:
        const std::vector<rpos::features::location_provider::PointPDF>& points() const;
        std::vector<rpos::features::location_provider::PointPDF>& points();

    private:
        std::vector<rpos::features::location_provider::PointPDF> points_;
    };

}}}