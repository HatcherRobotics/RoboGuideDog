#pragma once
#include <rpos/core/pose.h>
#include <rpos/features/location_provider/map.h>

#define RPOS_FEATURES_POINTPDF_TAGS_MAX_COUNT            (0xFFU)
#define RPOS_FEATURES_POINTPDF_STRING_MAX_LENGTH         (0xFFFFU)

namespace rpos { namespace features { namespace location_provider {
    
    struct PointPDF
    {
        rpos::system::types::_u32 id;
        core::Location location;
        float circular_error_probability;//on x-y planar
        std::vector<std::string> tags;
    };

    class PointsMap;
    namespace detail {
        class PointsMapImpl;
        template<>
        struct RPOS_CORE_API map_caster < PointsMap > {
            static PointsMap cast(const Map& map);
        };
    }

    class RPOS_CORE_API PointsMap : public Map {
    public:
        typedef detail::PointsMapImpl impl_t;

        RPOS_OBJECT_CTORS_WITH_BASE(PointsMap, Map);
        PointsMap(boost::shared_ptr<detail::PointsMapImpl> impl);
        virtual ~PointsMap();
        static PointsMap createMap(rpos::system::types::_u64 timestamp = 0);

        virtual bool readFromStream(rpos::system::io::IStream &in);
        virtual bool writeToStream(rpos::system::io::IStream &out) const;

    public:
        void clear();
        size_t size() const;
        void resize(size_t N);
        const PointPDF& operator[](size_t i) const;
        PointPDF& operator[](size_t i);
    };

}}}