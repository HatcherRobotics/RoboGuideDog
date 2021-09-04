/*
* memory_read_stream.h
* MemoryReadStream use a buffer in memory to read from
*
* Created by Tony Huang (tony@slamtec.com) at 2017-4-13
* Copyright 2017 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include "i_stream.h"

#include <vector>
#include <cstdint>
#include <string>
#include <stdio.h>

namespace rpos { namespace system { namespace io {

    class RPOS_CORE_API MemoryReadStream : public IStream {
    public:

        /**
        * Options to construct memory read stream
        */
        enum MemoryReadStreamFlag
        {
            // Copy the buffer to local buffer
            MemoryReadStreamFlagCopyBuffer = 0,

            // Borrow buffer from the parameter and DO NOT free in the destructor
            MemoryReadStreamFlagBorrowBuffer = 1,

            // Borrow buffer from the parameter and take the ownership, use free to release in the destructor
            MemoryReadStreamFlagFreeAfterUse = 2,

            // Borrow buffer from the parameter and take the ownership, use delete[] to release in the destructor
            MemoryReadStreamFlagDeleteArrayAfterUse = 3,

            MemoryReadStreamFlagDefault = MemoryReadStreamFlagCopyBuffer
        };

        MemoryReadStream(const void* buffer, size_t size, MemoryReadStreamFlag flags = MemoryReadStreamFlagDefault);
        MemoryReadStream(const std::vector<std::uint8_t>& buffer, MemoryReadStreamFlag flags = MemoryReadStreamFlagDefault);
        explicit MemoryReadStream(std::vector<std::uint8_t>&& buffer);
        virtual ~MemoryReadStream();

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
        virtual void seek(SeekType type, int offset);

    public:
        size_t size() const;
        const std::uint8_t* buffer() const;

    private:
        MemoryReadStreamFlag flag_;
        bool bufferInVector_;

        const std::uint8_t* buffer_;
        size_t bufferSize_;
        std::vector<std::uint8_t> vectorHolder_;
        size_t offset_;
    };

} } }
