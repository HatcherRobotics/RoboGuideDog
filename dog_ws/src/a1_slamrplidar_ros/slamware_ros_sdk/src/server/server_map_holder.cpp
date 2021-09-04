
#include "server_map_holder.h"

#include <boost/assert.hpp>

#include <cmath>
#include <cstring>
#include <stdexcept>

namespace slamware_ros_sdk {

    const float ServerMapHolder::C_DEFAULT_RESOLUTION = 0.05f;
    const std::uint32_t ServerMapHolder::C_MAP_DATA_SIZE_ALIGNMENT = 16u;
    const int ServerMapHolder::C_DEFAULT_MORE_CELL_CNT_TO_EXTEND = 32;

    ServerMapHolder::ServerMapHolder()
        : moreCellCntToExtend_(C_DEFAULT_MORE_CELL_CNT_TO_EXTEND)
        , resolution_(C_DEFAULT_RESOLUTION)
    {
        //
    }

    ServerMapHolder::~ServerMapHolder()
    {
        //
    }

    void ServerMapHolder::clear()
    {
        availCellIdxRect_ = rpos::core::RectangleI();
        availMapArea_ = rpos::core::RectangleF();
        validCellIdxRect_ = rpos::core::RectangleI();
        validMapArea_ = rpos::core::RectangleF();
        map_data_t().swap(mapDat_);
    }

    void ServerMapHolder::reinit(float resolution)
    {
        sfCheckResolutionValidity_(resolution);

        clear();
        resolution_ = resolution;
    }

    void ServerMapHolder::setMoreCellCountToExtend(int moreCellCntToExtend)
    {
        moreCellCntToExtend_ = (0 <= moreCellCntToExtend ? moreCellCntToExtend : C_DEFAULT_MORE_CELL_CNT_TO_EXTEND);
    }

    void ServerMapHolder::reserveByCellIdxRect(const rpos::core::RectangleI& cellIdxRect)
    {
        if (sfIsCellIdxRectEmpty(cellIdxRect))
            return;
        if (sfDoesCellIdxRectContain(availCellIdxRect_, cellIdxRect))
            return;

        auto newAvailCellIdxRect = sfMergeCellIdxRect(availCellIdxRect_, cellIdxRect);
        newAvailCellIdxRect = sfCalcAdjustedCellIdxRect_(newAvailCellIdxRect);

        const size_t newDatSize = newAvailCellIdxRect.width() * newAvailCellIdxRect.height();
        map_data_t newMapDat(newDatSize);
        std::memset(newMapDat.data(), 0, (newDatSize * sizeof(cell_value_t)));

        if (!sfIsCellIdxRectEmpty(validCellIdxRect_))
        {
            const int destOffX = validCellIdxRect_.x() - newAvailCellIdxRect.x();
            BOOST_ASSERT(0 <= destOffX && destOffX < newAvailCellIdxRect.width());
            BOOST_ASSERT(destOffX + validCellIdxRect_.width() <= newAvailCellIdxRect.width());

            const int destOffY = validCellIdxRect_.y() - newAvailCellIdxRect.y();
            BOOST_ASSERT(0 <= destOffY && destOffY < newAvailCellIdxRect.height());
            BOOST_ASSERT(destOffY + validCellIdxRect_.height() <= newAvailCellIdxRect.height());

            const size_t srcBytesPerLine = validCellIdxRect_.width() * sizeof(cell_value_t);
            size_t srcCellOffset = (validCellIdxRect_.y() - availCellIdxRect_.y()) * availCellIdxRect_.width() + (validCellIdxRect_.x() - availCellIdxRect_.x());
            size_t destCellOffset = destOffY * newAvailCellIdxRect.width() + destOffX;
            for (int j = 0; j < validCellIdxRect_.height(); ++j)
            {
                std::memcpy(&newMapDat[destCellOffset], &mapDat_[srcCellOffset], srcBytesPerLine);

                srcCellOffset += availCellIdxRect_.width();
                destCellOffset += newAvailCellIdxRect.width();
            }
        }

        availCellIdxRect_ = newAvailCellIdxRect;
        availMapArea_ = calcAreaByCellIdxRect(availCellIdxRect_);
        mapDat_.swap(newMapDat);

    #if 0
        ROS_INFO("ServerMapHolder::reserveByCellIdxRect(), avail rect: ((%d, %d), (%d, %d)), avail area: ((%f, %f), (%f, %f))."
            , availCellIdxRect_.x(), availCellIdxRect_.y(), availCellIdxRect_.width(), availCellIdxRect_.height()
            , availMapArea_.x(), availMapArea_.y(), availMapArea_.width(), availMapArea_.height()
            );
    #endif
    }

    void ServerMapHolder::reserveByArea(const rpos::core::RectangleF& reqArea)
    {
        if (reqArea.empty())
            return;

        const auto minBoundingCellIdxRect = calcMinBoundingCellIdxRect(reqArea);
        reserveByCellIdxRect(minBoundingCellIdxRect);
    }

    void ServerMapHolder::setMapData(float x, float y, float resolution, int dimensionX, int dimensionY, const cell_value_t* srcDat)
    {
        if (dimensionX < 1 || dimensionY < 1)
            return;
        BOOST_ASSERT(nullptr != srcDat);

        sfCheckResolutionEquality_(resolution, resolution_);

        const rpos::core::RectangleI cellIdxRectToUp(static_cast<int>(std::round(x / resolution_))
            , static_cast<int>(std::round(y / resolution_))
            , dimensionX
            , dimensionY
            );
        checkToExtendMap_(cellIdxRectToUp);

        const int destOffX = cellIdxRectToUp.x() - availCellIdxRect_.x();
        BOOST_ASSERT(0 <= destOffX && destOffX < availCellIdxRect_.width());
        BOOST_ASSERT(destOffX + dimensionX <= availCellIdxRect_.width());

        const int destOffY = cellIdxRectToUp.y() - availCellIdxRect_.y();
        BOOST_ASSERT(0 <= destOffY && destOffY < availCellIdxRect_.height());
        BOOST_ASSERT(destOffY + dimensionY <= availCellIdxRect_.height());

        const size_t srcBytesPerLine = dimensionX * sizeof(cell_value_t);
        size_t srcCellOffset = 0;
        size_t destCellOffset = destOffY * availCellIdxRect_.width() + destOffX;
        for (int j = 0; j < dimensionY; ++j)
        {
            std::memcpy(&mapDat_[destCellOffset], &srcDat[srcCellOffset], srcBytesPerLine);

            srcCellOffset += dimensionX;
            destCellOffset += availCellIdxRect_.width();
        }

        validCellIdxRect_ = sfMergeCellIdxRect(validCellIdxRect_, cellIdxRectToUp);
        validMapArea_ = calcAreaByCellIdxRect(validCellIdxRect_);
    }

    void ServerMapHolder::setMapData(const rpos::features::location_provider::Map& hMap)
    {
        if (!hMap)
            return;

        const auto& mapPosition = hMap.getMapPosition();
        const auto& mapDimension = hMap.getMapDimension();
        const auto& mapResolution = hMap.getMapResolution();
        const auto& mapDat = hMap.getMapData();

        if (!rpos::system::types::fequal(mapResolution.x(), mapResolution.y()))
            throw std::runtime_error("map resolution is different on x and y.");
        if (static_cast<int>(mapDat.size()) < (mapDimension.x() * mapDimension.y()))
            throw std::runtime_error("map data size is less than (dimensionX * dimensionY).");

        setMapData(mapPosition.x(), mapPosition.y(), mapResolution.x(), mapDimension.x(), mapDimension.y(), mapDat.data());
    }

    rpos::core::RectangleI ServerMapHolder::fillRosMapMsg(const rpos::core::RectangleI& reqIdxRect, nav_msgs::GetMap::Response& msgMap) const
    {
        const auto resIdxRect = reqIdxRect;
        const auto resArea = calcAreaByCellIdxRect(resIdxRect);

        const auto intersecRect = sfIntersectionOfCellIdxRect(validCellIdxRect_, resIdxRect);

        msgMap.map.info.resolution = resolution_;
        msgMap.map.info.origin.position.x = resArea.x();
        msgMap.map.info.origin.position.y = resArea.y();
        msgMap.map.info.origin.position.z = 0.0;
        msgMap.map.info.origin.orientation.x = 0.0;
        msgMap.map.info.origin.orientation.y = 0.0;
        msgMap.map.info.origin.orientation.z = 0.0;
        msgMap.map.info.origin.orientation.w = 1.0;

        msgMap.map.info.width = 0;
        msgMap.map.info.height = 0;
        msgMap.map.data.clear();
        if (!sfIsCellIdxRectEmpty(resIdxRect))
        {
            msgMap.map.info.width = resIdxRect.width();
            msgMap.map.info.height = resIdxRect.height();
            const size_t destDatSize = msgMap.map.info.width * msgMap.map.info.height;
            msgMap.map.data.resize(destDatSize, -1);
            //
            if (!sfIsCellIdxRectEmpty(intersecRect))
            {
                const int srcOffsetX = intersecRect.x() - availCellIdxRect_.x();
                const int srcOffsetY = intersecRect.y() - availCellIdxRect_.y();
                size_t srcRowOffset = srcOffsetY * availCellIdxRect_.width() + srcOffsetX;
                //
                const int destOffsetX = intersecRect.x() - resIdxRect.x();
                const int destOffsetY = intersecRect.y() - resIdxRect.y();
                size_t destRowOffset = destOffsetY * resIdxRect.width() + destOffsetX;
                //
                for (int j = 0; j < intersecRect.height(); ++j)
                {
                    size_t tSrcIdx = srcRowOffset;
                    size_t tDestIdx = destRowOffset;
                    for (int i = 0; i < intersecRect.width(); ++i)
                    {
                        const auto tVal = mapDat_[tSrcIdx];
                        if (0 == tVal)
                            msgMap.map.data[tDestIdx] = -1;
                        else if (tVal <= 127)
                            msgMap.map.data[tDestIdx] = 0;
                        else if (127 < tVal)
                            msgMap.map.data[tDestIdx] = 100;
                        //
                        ++tSrcIdx;
                        ++tDestIdx;
                    }
                    //
                    srcRowOffset += availCellIdxRect_.width();
                    destRowOffset += resIdxRect.width();
                }
            }
        }

        return resIdxRect;
    }

    rpos::core::RectangleI ServerMapHolder::fillRosMapMsg(const rpos::core::RectangleF& reqArea, nav_msgs::GetMap::Response& msgMap) const
    {
        rpos::core::RectangleI reqIdxRect;
        if (!reqArea.empty())
            reqIdxRect = calcMinBoundingCellIdxRect(reqArea);
        return fillRosMapMsg(reqIdxRect, msgMap);
    }

    rpos::core::RectangleI ServerMapHolder::fillRosMapMsg(nav_msgs::GetMap::Response& msgMap) const
    {
        return fillRosMapMsg(validCellIdxRect_, msgMap);
    }

    bool ServerMapHolder::sfDoesCellIdxRectContain(const rpos::core::RectangleI& cellIdxRect, const rpos::core::RectangleI& objRect)
    {
        if (sfIsCellIdxRectEmpty(cellIdxRect))
            return false;
        const int xEnd = cellIdxRect.x() + cellIdxRect.width();
        const int yEnd = cellIdxRect.y() + cellIdxRect.height();

        const int objXEnd = (0 < objRect.width() ? (objRect.x() + objRect.width()) : objRect.x());
        const int objYEnd = (0 < objRect.height() ? (objRect.y() + objRect.height()) : objRect.y());
        return (cellIdxRect.x() <= objRect.x() && objRect.x() < xEnd
            && objXEnd <= xEnd
            && cellIdxRect.y() <= objRect.y() && objRect.y() < yEnd
            && objYEnd <= yEnd
            );
    }

    rpos::core::RectangleI ServerMapHolder::sfMergeCellIdxRect(const rpos::core::RectangleI& idxRectA, const rpos::core::RectangleI& idxRectB)
    {
        if (sfIsCellIdxRectEmpty(idxRectB))
            return idxRectA;
        if (sfIsCellIdxRectEmpty(idxRectA))
            return idxRectB;

        const int xMin = std::min<int>(idxRectA.x(), idxRectB.x());
        const int yMin = std::min<int>(idxRectA.y(), idxRectB.y());

        const int aXEnd = idxRectA.x() + idxRectA.width();
        const int aYEnd = idxRectA.y() + idxRectA.height();
        const int bXEnd = idxRectB.x() + idxRectB.width();
        const int bYEnd = idxRectB.y() + idxRectB.height();
        const int xEnd = std::max<int>(aXEnd, bXEnd);
        const int yEnd = std::max<int>(aYEnd, bYEnd);

        BOOST_ASSERT(xMin < xEnd);
        BOOST_ASSERT(yMin < yEnd);
        return rpos::core::RectangleI(xMin
            , yMin
            , (xEnd - xMin)
            , (yEnd - yMin)
            );
    }

    rpos::core::RectangleI ServerMapHolder::sfIntersectionOfCellIdxRect(const rpos::core::RectangleI& idxRectA, const rpos::core::RectangleI& idxRectB)
    {
        if (sfIsCellIdxRectEmpty(idxRectA)
            || sfIsCellIdxRectEmpty(idxRectB)
            )
        {
            return rpos::core::RectangleI();
        }

        const int xMin = std::max<int>(idxRectA.x(), idxRectB.x());
        const int yMin = std::max<int>(idxRectA.y(), idxRectB.y());

        const int aXEnd = idxRectA.x() + idxRectA.width();
        const int aYEnd = idxRectA.y() + idxRectA.height();
        const int bXEnd = idxRectB.x() + idxRectB.width();
        const int bYEnd = idxRectB.y() + idxRectB.height();
        const int xEnd = std::min<int>(aXEnd, bXEnd);
        const int yEnd = std::min<int>(aYEnd, bYEnd);

        return rpos::core::RectangleI(xMin
            , yMin
            , (xMin < xEnd ? (xEnd - xMin) : 0)
            , (yMin < yEnd ? (yEnd - yMin) : 0)
            );
    }

    rpos::core::RectangleF ServerMapHolder::sfCalcAreaByCellIdxRect(float resolution, const rpos::core::RectangleI& cellIdxRect)
    {
        BOOST_ASSERT(FLT_EPSILON < resolution);
        return rpos::core::RectangleF(resolution * cellIdxRect.x()
            , resolution * cellIdxRect.y()
            , resolution * cellIdxRect.width()
            , resolution * cellIdxRect.height()
            );
    }

    rpos::core::RectangleI ServerMapHolder::sfCalcMinBoundingCellIdxRect(float resolution, const rpos::core::RectangleF& reqArea)
    {
        BOOST_ASSERT(FLT_EPSILON < resolution);
        
        int idxXMin = static_cast<int>(std::floor(reqArea.x() / resolution));
        if (reqArea.x() < resolution * idxXMin)
            --idxXMin;

        int idxYMin = static_cast<int>(std::floor(reqArea.y() / resolution));
        if (reqArea.y() < resolution * idxYMin)
            --idxYMin;

        const float srcXMax = (0.0f < reqArea.width() ? (reqArea.x() + reqArea.width()) : reqArea.x());
        int idxXMax = static_cast<int>(std::floor(srcXMax / resolution));
        if (resolution * (idxXMax + 1) < srcXMax)
            ++idxXMax;

        const float srcYMax = (0.0f < reqArea.height() ? (reqArea.y() + reqArea.height()) : reqArea.y());
        int idxYMax = static_cast<int>(std::floor(srcYMax / resolution));
        if (resolution * (idxYMax + 1) < srcYMax)
            ++idxYMax;

        BOOST_ASSERT(idxXMin <= idxXMax);
        BOOST_ASSERT(idxYMin <= idxYMax);
        return rpos::core::RectangleI(idxXMin
            , idxYMin
            , (idxXMax - idxXMin + 1)
            , (idxYMax - idxYMin + 1)
            );
    }

    rpos::core::RectangleF ServerMapHolder::sfCalcMinBoundingArea(float resolution, const rpos::core::RectangleF& reqArea)
    {
        BOOST_ASSERT(FLT_EPSILON < resolution);
        const auto minBoundingCellIdxRect = sfCalcMinBoundingCellIdxRect(resolution, reqArea);
        return sfCalcAreaByCellIdxRect(resolution, minBoundingCellIdxRect);
    }

    rpos::core::RectangleI ServerMapHolder::sfCalcRoundedCellIdxRect(float resolution, const rpos::core::RectangleF& reqArea)
    {
        BOOST_ASSERT(FLT_EPSILON < resolution);
        return rpos::core::RectangleI(static_cast<int>(std::round(reqArea.x() / resolution))
            , static_cast<int>(std::round(reqArea.y() / resolution))
            , static_cast<int>(std::round(reqArea.width() / resolution))
            , static_cast<int>(std::round(reqArea.height() / resolution))
            );
    }

    rpos::core::RectangleF ServerMapHolder::sfCalcRoundedArea(float resolution, const rpos::core::RectangleF& reqArea)
    {
        BOOST_ASSERT(FLT_EPSILON < resolution);
        const auto roundedCellIdxRect = sfCalcRoundedCellIdxRect(resolution, reqArea);
        return sfCalcAreaByCellIdxRect(resolution, roundedCellIdxRect);
    }

    void ServerMapHolder::checkToExtendMap_(const rpos::core::RectangleI& cellIdxRectToUp)
    {
        BOOST_ASSERT(0 <= moreCellCntToExtend_);
        BOOST_ASSERT(0 <= availCellIdxRect_.width());
        BOOST_ASSERT(0 <= availCellIdxRect_.height());

        if (sfIsCellIdxRectEmpty(cellIdxRectToUp))
            return;
        if (sfDoesCellIdxRectContain(availCellIdxRect_, cellIdxRectToUp))
            return;

        int tXMin = availCellIdxRect_.x();
        if (cellIdxRectToUp.x() < tXMin)
            tXMin = cellIdxRectToUp.x() - moreCellCntToExtend_;

        int tYMin = availCellIdxRect_.y();
        if (cellIdxRectToUp.y() < tYMin)
            tYMin = cellIdxRectToUp.y() - moreCellCntToExtend_;

        const int upXEnd = cellIdxRectToUp.x() + cellIdxRectToUp.width();
        int tXEnd = availCellIdxRect_.x() + availCellIdxRect_.width();
        if (tXEnd < upXEnd)
            tXEnd = upXEnd + moreCellCntToExtend_;

        const int upYEnd = cellIdxRectToUp.y() + cellIdxRectToUp.height();
        int tYEnd = availCellIdxRect_.y() + availCellIdxRect_.height();
        if (tYEnd < upYEnd)
            tYEnd = upYEnd + moreCellCntToExtend_;

        BOOST_ASSERT(tXMin < tXEnd);
        BOOST_ASSERT(tYMin < tYEnd);
        const auto tCellIdxRect = rpos::core::RectangleI(tXMin
            , tYMin
            , (tXEnd - tXMin)
            , (tYEnd - tYMin)
            );
        reserveByCellIdxRect(tCellIdxRect);
    }

    void ServerMapHolder::sfCheckResolutionValidity_(float resolution)
    {
        if (resolution <= FLT_EPSILON)
            throw std::runtime_error("invalid resolution.");
    }

    void ServerMapHolder::sfCheckResolutionEquality_(float resA, float resB)
    {
        if (!rpos::system::types::fequal(resA, resB))
            throw std::runtime_error("inconsistent resolution.");

        if (resA <= FLT_EPSILON)
            throw std::runtime_error("invalid resolution.");
        if (resB <= FLT_EPSILON)
            throw std::runtime_error("invalid resolution.");
    }

    int ServerMapHolder::sfGetAlignedFloorCellIdx_(int inCellIdx)
    {
        if (inCellIdx < 0)
        {
            std::uint32_t tmpIdx = static_cast<std::uint32_t>(-inCellIdx);
            --tmpIdx;
            tmpIdx /= C_MAP_DATA_SIZE_ALIGNMENT;
            ++tmpIdx;
            tmpIdx *= C_MAP_DATA_SIZE_ALIGNMENT;
            return (-static_cast<int>(tmpIdx));
        }
        else
        {
            std::uint32_t tmpIdx = static_cast<std::uint32_t>(inCellIdx);
            tmpIdx /= C_MAP_DATA_SIZE_ALIGNMENT;
            tmpIdx *= C_MAP_DATA_SIZE_ALIGNMENT;
            return static_cast<int>(tmpIdx);
        }
    }

    int ServerMapHolder::sfGetAlignedCeilSize_(int inSize)
    {
        if (inSize < 0)
            throw std::runtime_error("invalid input size.");

        std::uint32_t tmpSize = static_cast<std::uint32_t>(inSize);
        tmpSize += (C_MAP_DATA_SIZE_ALIGNMENT - 1);
        tmpSize /= C_MAP_DATA_SIZE_ALIGNMENT;
        tmpSize *= C_MAP_DATA_SIZE_ALIGNMENT;
        return static_cast<int>(tmpSize);
    }
    
    rpos::core::RectangleI ServerMapHolder::sfCalcAdjustedCellIdxRect_(const rpos::core::RectangleI& idxRect)
    {
        if (sfIsCellIdxRectEmpty(idxRect))
            return rpos::core::RectangleI();

        const int xMin = sfGetAlignedFloorCellIdx_(idxRect.x());
        BOOST_ASSERT(xMin <= idxRect.x());
        const int yMin = sfGetAlignedFloorCellIdx_(idxRect.y());
        BOOST_ASSERT(yMin <= idxRect.y());

        const int reqXEnd = idxRect.x() + idxRect.width();
        const int reqYEnd = idxRect.y() + idxRect.height();
        const int tWidth = sfGetAlignedCeilSize_(reqXEnd - xMin);
        BOOST_ASSERT(reqXEnd <= xMin + tWidth);
        const int tHeight = sfGetAlignedCeilSize_(reqYEnd - yMin);
        BOOST_ASSERT(reqYEnd <= yMin + tHeight);

        return rpos::core::RectangleI(xMin, yMin, tWidth, tHeight);
    }

}
