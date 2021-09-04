/*
* rle_decoder.h
* Rle Decoder
*
* Created by Tony Huang (tony@slamtec.com) at 2018-10-5
* Copyright 2018 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include "i_encoder.h"

namespace rpos { namespace system { namespace encoding {

    /**
    * RLE decoder
    *
    * Notice: RLE decoder doesn't decode the RLE header, please handle it by yourself
    */
    class RPOS_CORE_API RleDecoder : public IEncoder
    {
    public:
        typedef unsigned char byte_t;

    public:
        RleDecoder(byte_t sentinel1 = 129, byte_t sentinel2 = 127);
        virtual ~RleDecoder();

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

    public:
        virtual void reset();
        virtual void reset(byte_t sentinel1, byte_t sentinel2);

    private:
        byte_t initialSentinel1_, initialSentinel2_;
        byte_t sentinel1_, sentinel2_;
    };

} } }
