/*
* test_logging_store.h
* Test logging store is a data store to store test logs (such as maps, test cases, laser scans and etc...)
*
* Created by Tony Huang (tony@slamtec.com) at 2016-6-13
* Copyright 2016 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include "i_blob_store.h"
#include <boost/thread/mutex.hpp>

namespace rpos { namespace system { namespace diagnosis {

	class RPOS_CORE_API LocalFileSystemBlobStore : public IBlobStore {
	public:
		LocalFileSystemBlobStore(const std::string& folder);
		virtual ~LocalFileSystemBlobStore();

	public:
		virtual void enumerate(std::list<std::string>& blobIds);
		virtual bool has(const std::string& blobId);
		virtual boost::shared_ptr<io::IStream> load(const std::string& blobId);
		virtual std::string store(const io::MemoryWriteStream& blob);

	private:
		std::string folder_;
		boost::mutex lock_;
	};

} } }
