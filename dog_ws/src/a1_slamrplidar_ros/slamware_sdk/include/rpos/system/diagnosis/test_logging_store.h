/*
* test_logging_store.h
* Test logging store is a data store to store test logs (such as maps, test cases, laser scans and etc...)
*
* Created by Tony Huang (tony@slamtec.com) at 2016-6-13
* Copyright 2016 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include "i_blob_store.h"
#include "diagnosis_serialization.h"

#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>

namespace rpos { namespace system { namespace diagnosis {

	class RPOS_CORE_API TestLoggingStore : boost::noncopyable {
	public:
		TestLoggingStore(const std::string& folder);

	public:
		~TestLoggingStore();

	public:
		static boost::shared_ptr<TestLoggingStore> createSharedStore(const std::string& folder);
		static boost::shared_ptr<TestLoggingStore> sharedStore();

	public:
		void enumerate(const std::string& storeName, std::list<std::string>& objectIds);
		bool has(const std::string& storeName, const std::string& objectId);

		template < class T >
		bool load(const std::string& storeName, const std::string& objectId, T& object)
		{
			boost::lock_guard<boost::mutex> guard(lock_);

			boost::shared_ptr<io::IStream> stream = getBlobStore(storeName)->load(objectId);
			if (!stream)
				return false;
			serialization::read(*stream, object);
			return true;
		}

		template < class T >
		std::string store(const std::string& storeName, const T& object)
		{
			boost::lock_guard<boost::mutex> guard(lock_);

			io::MemoryWriteStream ms;
			serialization::write(ms, object);
			return getBlobStore(storeName)->store(ms);
		}

	private:
		boost::shared_ptr<IBlobStore> getBlobStore(const std::string& name);

	private:
		std::string folder_;
		std::map<std::string, boost::shared_ptr<IBlobStore> > cachedBlobStores_;
		boost::mutex lock_;
	};
	
} } }
