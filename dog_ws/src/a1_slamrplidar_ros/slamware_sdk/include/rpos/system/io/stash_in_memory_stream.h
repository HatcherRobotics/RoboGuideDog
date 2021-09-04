/*
*
* Created by chengwei (chengwei920412@outlook.com) at 2017-09-27 17:50:55
* Copyright 2017 (c) Shanghai Slamtec Co., Ltd.
*/
#pragma once
#include <boost/circular_buffer.hpp>
#include "i_stream.h"
#include "memory_write_stream.h"
#include <deque>
#include <cstdint>

namespace rpos {namespace system {namespace io {
    class RPOS_CORE_API StashInMemoryStream : public IStream{
    public:
        struct Options {
            size_t max_size; 
        };

    public:
        StashInMemoryStream(Options &option);
        virtual ~StashInMemoryStream();
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
        int size() const;
        void clear();
    private:
        Options option_;
        boost::circular_buffer<std::uint8_t> buffers_;
        size_t size_;
    };
}}}
