/*
* file_stream.h
* FileStream implements the IStream interface and is used to manipulate files
*
* Created by Tony Huang (tony@slamtec.com) at 2016-6-13
* Copyright 2016 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include "i_stream.h"

#include <stdio.h>
#include <string>
#include <boost/filesystem/path.hpp>

namespace rpos { namespace system { namespace io {

	enum OpenFileMode
	{
		OpenFileModeRead = 1,
		OpenFileModeWrite = 2,
        OpenFileModeReadWrite = 3
	};

	class RPOS_CORE_API FileStream : public IStream
	{
	public:
		FileStream();
		virtual ~FileStream();

	public:
		bool open(const std::string& filename, OpenFileMode openFileMode);
        bool open(const std::wstring& filename, OpenFileMode openFileMode);

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
		int read(void* buffer, size_t size);
		int write(const void* buffer, size_t size);
        virtual size_t tell();
        virtual void seek(SeekType type, int offset);

    public:
        void flush();

	private:
		boost::filesystem::path filename_;
		bool open_;
		bool canRead_;
		bool canWrite_;
		long fileSize_;
		FILE* file_;
	};

} } }
