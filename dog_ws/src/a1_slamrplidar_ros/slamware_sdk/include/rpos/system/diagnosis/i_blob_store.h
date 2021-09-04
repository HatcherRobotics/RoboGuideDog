/*
* i_blob_store.h
* IBlobStore is the abstract interface to store blobs
*
* Created by Tony Huang (tony@slamtec.com) at 2016-6-13
* Copyright 2016 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include <list>
#include <string>

#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>

#include "../io/i_stream.h"
#include "../io/memory_write_stream.h"

namespace rpos { namespace system { namespace diagnosis {

	class RPOS_CORE_API IBlobStore : boost::noncopyable {
	public:
		IBlobStore();
		virtual ~IBlobStore();

	public:
		virtual void enumerate(std::list<std::string>& blobIds) = 0;
		virtual bool has(const std::string& blobId) = 0;
		virtual boost::shared_ptr<io::IStream> load(const std::string& blobId) = 0;
		virtual std::string store(const io::MemoryWriteStream& blob) = 0;
	};

} } }
