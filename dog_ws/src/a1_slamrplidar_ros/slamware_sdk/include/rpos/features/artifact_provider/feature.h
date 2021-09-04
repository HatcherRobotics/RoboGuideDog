/*
* feature.h
* Artifact Provider feature
*
* Created by Jacky Li (eveningwear@gmail.com) at 2014-12-26
* Copyright 2014 (c) www.robopeak.com
*/

#pragma once

#include <rpos/rpos_config.h>

#include <rpos/core/feature.h>
#include <rpos/core/geometry.h>
#include <vector>

namespace rpos { namespace features {

    namespace detail {
        class ArtifactProviderImpl;
    }

    namespace artifact_provider
    {
        enum ArtifactUsage
        {
            /**
            * Virtual walls works just like walls, they blocks robot move through
            */
            ArtifactUsageVirtualWall,

            /**
            * Virtual tracks are used to guide robot's movement. Robot will try its best to move along with virtual tracks
            */
            ArtifactUsageVirtualTrack
        };
    }

    class RPOS_CORE_API ArtifactProvider : public rpos::core::Feature{
    public:
        typedef detail::ArtifactProviderImpl impl_t;

        RPOS_OBJECT_CTORS_WITH_BASE(ArtifactProvider, rpos::core::Feature);
        ArtifactProvider(boost::shared_ptr<detail::ArtifactProviderImpl> impl);
        ~ArtifactProvider();

    public:
        std::vector<core::Line> getLines(artifact_provider::ArtifactUsage usage);
        bool addLine(artifact_provider::ArtifactUsage usage, const core::Line& line);
        bool addLines(artifact_provider::ArtifactUsage usage, const std::vector<core::Line>& lines);
        bool removeLineById(artifact_provider::ArtifactUsage usage, rpos::core::SegmentID id);
        bool clearLines(artifact_provider::ArtifactUsage usage);
        bool moveLine(artifact_provider::ArtifactUsage usage, const core::Line& line);
        bool moveLines(artifact_provider::ArtifactUsage usage, const std::vector<core::Line>& lines);

        std::vector<core::Line> getWalls();
        bool addWall(const core::Line& wall);
        bool addWalls(const std::vector<core::Line>& wall);
        bool clearWallById(const rpos::core::SegmentID& id);
        bool clearWalls();
    };
        
} }
