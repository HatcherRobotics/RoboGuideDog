/*
* i_encoder.h
* IEncoder defines the abstract interface of encoders
*
* Created by Tony Huang (tony@slamtec.com) at 2018-10-04
* Copyright 2018 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include <cstdint>
#include <rpos/core/rpos_core_config.h>

namespace rpos { namespace system { namespace encoding {

    /**
    * Indicate where the feeding data is in the source stream
    */
    enum StreamPosition {
        /**
        * The feeding data is at the beginning of the source stream
        */
        StreamPositionBegin = 1,

        /**
        * The feeding data is in the body of the source stream
        */
        StreamPositionBody = 2,

        /**
        * The feeding data is at the end of the source stream
        */
        StreamPositionEnd = 4
    };

    /**
    * Abstract interface of encoders
    *
    * Encoders process data streamly to reduce memory footprint
    */
    class RPOS_CORE_API IEncoder {
    public:
        virtual ~IEncoder();

    public:
        /**
        * Encode data from source buffer to dest buffer
        *
        * @param srcBuffer        The source buffer
        * @param srcSize          The size of source buffer
        * @param destBuffer       The buffer to store encoded data
        * @param destSize         The size of dest buffer
        * @param dataPosition     Indicate the position of data in the source stream (for encoders like base64, we need this to decide use one or two pads)
        * @param consumedSrcSize  Bytes encoded in the source stream
        * @param consumedDestSize Bytes written to the dest stream
        * @return The encoding is successful or not
        */
        virtual bool encode(
            const void* srcBuffer, size_t srcSize,
            void* destBuffer, size_t destSize,
            StreamPosition dataPosition,
            size_t& consumedSrcSize, size_t& consumedDestSize
        ) = 0;

        /**
        * Estimate encoded size
        *
        * @param srcBuffer  The source buffer
        * @param srcSize    The size of source
        *
        * Estimated size of encoded data (this should be equal to or greater than the actual encoded size)
        */
        virtual size_t estimateEncodedSize(const void* srcBuffer, size_t srcSize) = 0;

        /**
        * Estimate encoded size
        *
        * @param srcSize    The size of source
        *
        * Estimated size of encoded data (this should be equal to or greater than the actual encoded size)
        */
        virtual size_t estimateEncodedSize(size_t srcSize) = 0;

        /**
        * Reset encoder to its initial status
        */
        virtual void reset() = 0;
    };

} } }
