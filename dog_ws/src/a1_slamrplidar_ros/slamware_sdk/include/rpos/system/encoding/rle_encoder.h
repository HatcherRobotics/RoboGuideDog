/*
* rle_encoder.h
* RLE Encoder
*
* Created by Tony Huang (tony@slamtec.com) at 2018-10-05
* Copyright 2018 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include "i_encoder.h"

namespace rpos { namespace system { namespace encoding {

    /**
    * RLE Encoder
    *
    * The RLE Encoder will not write the RLE header to the output stream, please do it yourself
    */
    class RPOS_CORE_API RleEncoder : public IEncoder
    {
    public:
        typedef std::uint8_t byte_t;

    public:
        /**
        * Constructor
        *
        * @param sentinel1 Sentinel 1 (usually 129)
        * @param sentinel2 Sentinel 2 (usually 127)
        */
        RleEncoder(byte_t sentinel1 = 129, byte_t sentinel2 = 127);

        /**
        * Destructor
        */
        virtual ~RleEncoder();

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
        * Reset encoder
        */
        virtual void reset();

    private:
        byte_t initialSentinel1_;
        byte_t initialSentinel2_;
        byte_t sentinel1_;
        byte_t sentinel2_;
    };

} } }
