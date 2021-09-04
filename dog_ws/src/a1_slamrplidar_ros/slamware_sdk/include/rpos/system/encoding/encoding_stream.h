/*
* encoding_stream.h
* Encoding stream
*
* Created by Tony Huang (tony@slamtec.com) at 2018-10-8
* Copyright 2018 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include "../io/i_stream.h"
#include "i_encoder.h"

#include <boost/thread/recursive_mutex.hpp>

#include <vector>
#include <cstdint>

namespace rpos { namespace system { namespace encoding {

    static const size_t kEncodingStreamDefaultSrcBufferSize = 10 * 1024;
    static const size_t kEncodingStreamDefaultDestBufferSize = 10 * 1024;

    /**
    * Encoding stream
    *
    * Use an encoder to encode data. Write raw data via write() calls, and read encoded data via read() calls
    * When you finished writing all data, please invoke endWrite() to make sure it's all done
    */
    class RPOS_CORE_API EncodingStream
        : public io::IStream
    {
    public:
        /**
        * Constructor
        *
        * @param encoder        The encoder used to encode data
        * @param srcBufferSize  The buffer size to store raw data
        * @param destBufferSize The buffer size to store encoded data
        */
        explicit EncodingStream(
            IEncoder& encoder, 
            size_t srcBufferSize = kEncodingStreamDefaultSrcBufferSize, 
            size_t destBufferSize = kEncodingStreamDefaultDestBufferSize
        );

        /**
        * Destructor
        */
        virtual ~EncodingStream();

        // IStream APIs
    public:
        virtual bool isOpen();
        virtual bool canRead();
        virtual bool canWrite();
        virtual bool canSeek();

    public:
        virtual void close();

    public:
        virtual bool endOfStream();

    public:
        virtual int read(void* buffer, size_t size);
        virtual int write(const void* buffer, size_t size);
        virtual size_t tell();
        virtual void seek(io::SeekType type, int offset);

        // Specific APIs
    public:
        /**
        * Finish the write (it will make encode call with StreamPositionEnd parameter)
        */
        virtual void endWrite();

        /**
        * Reset all status (it will also clear all buffer)
        */
        virtual void reset();

        virtual void setSrcBufferSize(size_t size);
        virtual void setDestBufferSize(size_t size);

    private:
        virtual void encode();

    private:
        mutable boost::recursive_mutex lock_;

        IEncoder& encoder_;

        bool isBegining_;
        bool isWritable_;

        std::vector<std::uint8_t> srcBuffer_;
        std::vector<std::uint8_t> destBuffer_;
    };

} } }
