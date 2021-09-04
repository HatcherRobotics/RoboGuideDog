#pragma once
/*
* i_stream.h
* IStream is the abstract interface of a byte stream
*
* Created by Tony Huang (tony@slamtec.com) at 2016-6-13
* Copyright 2016 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include <stdint.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <rpos/core/rpos_core_config.h>

namespace rpos { namespace system { namespace io {

    enum SeekType {
        SeekTypeSet,
        SeekTypeEnd,
        SeekTypeOffset
    };

	class RPOS_CORE_API IStream {
	public:
		IStream();
		virtual ~IStream();

	public:
		virtual bool isOpen() = 0;
		virtual bool canRead() = 0;
		virtual bool canWrite() = 0;
        virtual bool canSeek() = 0;

	public:
		virtual void close() = 0;

	public:
		virtual bool endOfStream() = 0;

	public:
		virtual int read(void* buffer, size_t size) = 0;
		virtual int write(const void* buffer, size_t size) = 0;
        virtual size_t tell() = 0;
        virtual void seek(SeekType type, int offset) = 0;

	public:
		// throw std::runtime_error if failed to read/write exactly size bytes
		void exactRead(void* buffer, size_t size);
		void exactWrite(const void* buffer, size_t size);
	};

    class RPOS_CORE_API ISerializable
    {
    public:
        virtual ~ISerializable() {}
        virtual bool readFromStream(IStream &in) = 0;
        virtual bool writeToStream(IStream &out) const = 0;
    };

#define DECLARE_ISTREAM_READ_WRITE( T ) \
    RPOS_CORE_API IStream& operator<<(IStream&out, const T &a); \
    RPOS_CORE_API IStream& operator>>(IStream&in, T &a);

    DECLARE_ISTREAM_READ_WRITE(bool)
    DECLARE_ISTREAM_READ_WRITE(uint8_t)
    DECLARE_ISTREAM_READ_WRITE(int8_t)
    DECLARE_ISTREAM_READ_WRITE(uint16_t)
    DECLARE_ISTREAM_READ_WRITE(int16_t)
    DECLARE_ISTREAM_READ_WRITE(uint32_t)
    DECLARE_ISTREAM_READ_WRITE(int32_t)
    DECLARE_ISTREAM_READ_WRITE(uint64_t)
    DECLARE_ISTREAM_READ_WRITE(int64_t)
    DECLARE_ISTREAM_READ_WRITE(float)
    DECLARE_ISTREAM_READ_WRITE(double)
    DECLARE_ISTREAM_READ_WRITE(std::vector<uint8_t>)

    DECLARE_ISTREAM_READ_WRITE(std::string)

} } }
