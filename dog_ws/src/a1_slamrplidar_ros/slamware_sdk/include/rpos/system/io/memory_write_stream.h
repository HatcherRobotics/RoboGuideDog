/*
* memory_write_stream.h
* MemoryWriteStream use memory to store data in memory
*
* Created by Tony Huang (tony@slamtec.com) at 2016-6-13
* Copyright 2016 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include "i_stream.h"

#include <vector>
#include <string>
#include <cstdint>
#include <stdio.h>

namespace rpos { namespace system { namespace io {

	static const size_t DefaultMemoryStreamInitialCapacity = 512;

	class RPOS_CORE_API MemoryWriteStream : public IStream {
	public:
		MemoryWriteStream(size_t initialCapacity = DefaultMemoryStreamInitialCapacity);
		virtual ~MemoryWriteStream();

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
        const std::vector<std::uint8_t>& asByteVector() const;
		void writeTo(IStream& target) const;
		void writeToFile(const std::string& filename) const;

	private:
		std::vector<std::uint8_t> buffer_;
		size_t size_;
	};

} } }
