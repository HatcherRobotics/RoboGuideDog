/*
* pipe.h
* Pipe connects streams
*
* Created by Tony Huang (tony@slamtec.com) at 2018-10-8
* Copyright 2018 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include "i_stream.h"
#include <vector>
#include <cstdint>

namespace rpos { namespace system { namespace io {

    static const size_t kPipeDefaultBufferSize = 30 * 1024;

    /**
    * Pipe connects streams, it read data from src and write to dest
    */
    class RPOS_CORE_API Pipe {
    public:
        /**
        * Constructor
        *
        * @param src        The upstream
        * @param dest       The downstream
        * @param bufferSize Buffer size
        */
        Pipe(IStream& src, IStream& dest, size_t bufferSize = kPipeDefaultBufferSize);

        /**
        * Destructor
        */
        virtual ~Pipe();

    public:
        /**
        * Pump data once (limited by the buffer size)
        *
        * @return true for all data has been pumped, false for not
        */
        bool pumpOnce();

        /**
        * Pump data until all data have been read and written
        *
        * @return true for all data has been pumped, false for not
        */
        void pumpTillEnd();

        /**
        * Indicate if the Pipe's internal buffer is empty
        */
        bool bufferEmpty() const;

    private:
        /**
        * read as much data as possible from upstream to fill the internal buffer
        *
        * @return negative for read error, otherwise the bytes filled to internal buffer
        */
        int fillInternalBuffer_();

        /**
        * write as much data as possible to downstream to flush the internal buffer
        *
        * @return negative for write error, zero for downstream buffer full, otherwise the bytes flushed to downstream
        */
        int flushInternalBuffer_();

    public:
        IStream& src_;
        IStream& dest_;

        std::vector<std::uint8_t> buffer_;
        size_t begin_;
        size_t size_;
    };

} } }
