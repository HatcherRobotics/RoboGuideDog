/*
* base64_encoder.h
* Base64 Encoder
*
* Created by Tony Huang (tony@slamtec.com) at 2018-10-04
* Copyright 2018 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include "i_encoder.h"

namespace rpos { namespace system { namespace encoding {

    /**
    * Base64 Encoder
    */
    class RPOS_CORE_API Base64Encoder : public IEncoder {
    public:
        Base64Encoder();
        virtual ~Base64Encoder();

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
            size_t& comsumedSrcSize, size_t& consumedDestSize
        );

        /**
        * Estimate encoded size
        *
        * @param srcBuffer  The source buffer
        * @param srcSize    The size of source
        *
        * Estimated size of encoded data (this should be equal to or greater than the actual encoded size)
        */
        virtual size_t estimateEncodedSize(const void* srcBuffer, size_t srcSize);

        /**
        * Estimate encoded size
        *
        * @param srcSize    The size of source
        *
        * Estimated size of encoded data (this should be equal to or greater than the actual encoded size)
        */
        virtual size_t estimateEncodedSize(size_t srcSize);

        /**
        * Reset Encoder
        */
        virtual void reset();
    };

} } }
