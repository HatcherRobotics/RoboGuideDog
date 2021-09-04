/*
* snapshot_loop_buffer_memory_write_stream.h
* This is an implementation of memory stream but has a limited memory, and featuring a fast snapshot function
*
* Created by Tony Huang (tony@slamtec.com) at 2017-5-14
* Copyright 2017 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include "memory_write_stream.h"
#include <boost/shared_ptr.hpp>
#include <list>
#include <deque>

namespace rpos { namespace system { namespace io {

    class RPOS_CORE_API SnapshotLoopBufferMemoryWriteStream : public IStream {
    public:
        struct Options {
            size_t capcity;
            size_t split_size;
        };

        SnapshotLoopBufferMemoryWriteStream(const Options& options);
        virtual ~SnapshotLoopBufferMemoryWriteStream();

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
        virtual void markSplit();

    public:
        size_t size() const;
        void writeTo(IStream& target) const;
        void writeToFile(const std::string& filename) const;

    public:
        class RPOS_CORE_API Snapshot {
        public:
            Snapshot();
            Snapshot(const Snapshot& rhs);
            Snapshot(Snapshot&& rhs);
            ~Snapshot();

        private:
            friend class SnapshotLoopBufferMemoryWriteStream;

        private:
            Snapshot(SnapshotLoopBufferMemoryWriteStream& stream);

        public:
            Snapshot& operator=(const Snapshot& rhs);
            Snapshot& operator=(Snapshot&& rhs);

            operator bool() const;

        public:
            void copy(const Snapshot& rhs);
            void swap(Snapshot& rhs);

            void release();

        public:
            size_t size() const;
            int read(size_t offset, std::uint8_t* buffer, size_t bytesToRead) const;
            void writeTo(IStream& target) const;
            void writeToFile(const std::string& filename) const;

        private:
            std::list<boost::shared_ptr<MemoryWriteStream>> buffers_;
            size_t size_;
        };

    public:
        Snapshot snapshot();

    private:
        friend class Snapshot;

        void split();

    private:
        Options options_;
        std::deque<boost::shared_ptr<MemoryWriteStream>> buffers_;
        size_t size_;
    };

} } }
