
#pragma once

#include <ros/ros.h>
#include <nav_msgs/GetMap.h>

#include <rpos/core/geometry.h>
#include <rpos/features/location_provider/map.h>

#include <cstdint>

namespace slamware_ros_sdk {

    class ServerMapHolder
    {
    public:
        static const float C_DEFAULT_RESOLUTION;
        static const std::uint32_t C_MAP_DATA_SIZE_ALIGNMENT;
        static const int C_DEFAULT_MORE_CELL_CNT_TO_EXTEND;

        typedef std::uint8_t                    cell_value_t;
        typedef std::vector<cell_value_t>       map_data_t;

    public:
        ServerMapHolder();
        ~ServerMapHolder();

        float resolution() const { return resolution_; }
        bool isMapDataEmpty() const { return sfIsCellIdxRectEmpty(validCellIdxRect_); }

        // when extending rect of map data, it will extend more count of cells to prevent memory allocation frequently.
        int getMoreCellCountToExtend() const { return moreCellCntToExtend_; }

        void clear();
        void reinit(float resolution);

        void setMoreCellCountToExtend(int moreCellCntToExtend);

        void reserveByCellIdxRect(const rpos::core::RectangleI& cellIdxRect);
        void reserveByArea(const rpos::core::RectangleF& reqArea);

        void setMapData(float x, float y, float resolution, int dimensionX, int dimensionY, const cell_value_t* srcDat);
        void setMapData(const rpos::features::location_provider::Map& hMap);

    public:
        const rpos::core::RectangleI& getValidCellIdxRect() const { return validCellIdxRect_; }
        const rpos::core::RectangleF& getValidMapArea() const { return validMapArea_; }

        // returns the cell index rect of actually filled cells
        rpos::core::RectangleI fillRosMapMsg(const rpos::core::RectangleI& reqIdxRect, nav_msgs::GetMap::Response& msgMap) const;
        rpos::core::RectangleI fillRosMapMsg(const rpos::core::RectangleF& reqArea, nav_msgs::GetMap::Response& msgMap) const;
        rpos::core::RectangleI fillRosMapMsg(nav_msgs::GetMap::Response& msgMap) const;

    public:
        rpos::core::RectangleF calcAreaByCellIdxRect(const rpos::core::RectangleI& cellIdxRect) const { return sfCalcAreaByCellIdxRect(resolution_, cellIdxRect); }

        rpos::core::RectangleI calcMinBoundingCellIdxRect(const rpos::core::RectangleF& reqArea) const { return sfCalcMinBoundingCellIdxRect(resolution_, reqArea); }
        rpos::core::RectangleF calcMinBoundingArea(const rpos::core::RectangleF& reqArea) const { return sfCalcMinBoundingArea(resolution_, reqArea); }

        rpos::core::RectangleI calcRoundedCellIdxRect(const rpos::core::RectangleF& reqArea) const { return sfCalcRoundedCellIdxRect(resolution_, reqArea); }
        rpos::core::RectangleF calcRoundedArea(const rpos::core::RectangleF& reqArea) const { return sfCalcRoundedArea(resolution_, reqArea); }

    public:
        static bool sfIsCellIdxRectEmpty(const rpos::core::RectangleI& cellIdxRect)
        {
            return (cellIdxRect.width() <= 0 || cellIdxRect.height() <= 0);
        }
        static bool sfDoesCellIdxRectContain(const rpos::core::RectangleI& cellIdxRect, int cellIdxX, int cellIdxY)
        {
            return (0 < cellIdxRect.width() && 0 < cellIdxRect.height()
                && cellIdxRect.x() <= cellIdxX && cellIdxX < (cellIdxRect.x() + cellIdxRect.width())
                && cellIdxRect.y() <= cellIdxY && cellIdxY < (cellIdxRect.y() + cellIdxRect.height())
                );
        }
        static bool sfDoesCellIdxRectContain(const rpos::core::RectangleI& cellIdxRect, const rpos::core::RectangleI& objRect);
        static rpos::core::RectangleI sfMergeCellIdxRect(const rpos::core::RectangleI& idxRectA, const rpos::core::RectangleI& idxRectB);
        static rpos::core::RectangleI sfIntersectionOfCellIdxRect(const rpos::core::RectangleI& idxRectA, const rpos::core::RectangleI& idxRectB);

        static rpos::core::RectangleF sfCalcAreaByCellIdxRect(float resolution, const rpos::core::RectangleI& cellIdxRect);

        static rpos::core::RectangleI sfCalcMinBoundingCellIdxRect(float resolution, const rpos::core::RectangleF& reqArea);
        static rpos::core::RectangleF sfCalcMinBoundingArea(float resolution, const rpos::core::RectangleF& reqArea);

        static rpos::core::RectangleI sfCalcRoundedCellIdxRect(float resolution, const rpos::core::RectangleF& reqArea);
        static rpos::core::RectangleF sfCalcRoundedArea(float resolution, const rpos::core::RectangleF& reqArea);

    private:
        void checkToExtendMap_(const rpos::core::RectangleI& cellIdxRectToUp);

    private:
        static void sfCheckResolutionValidity_(float resolution);
        static void sfCheckResolutionEquality_(float resA, float resB);

        // returns aligned floor cell index, may be negative.
        static int sfGetAlignedFloorCellIdx_(int inCellIdx);
        // returns aligned ceil size (>= 0).
        static int sfGetAlignedCeilSize_(int inSize);
        
        // returns cell index rect that position is aligned floor and size is aligned ceil.
        static rpos::core::RectangleI sfCalcAdjustedCellIdxRect_(const rpos::core::RectangleI& idxRect);

    private:
        int moreCellCntToExtend_;

        float resolution_;
        rpos::core::RectangleI availCellIdxRect_;
        rpos::core::RectangleF availMapArea_;
        rpos::core::RectangleI validCellIdxRect_;
        rpos::core::RectangleF validMapArea_;
        map_data_t mapDat_;
    };

}
