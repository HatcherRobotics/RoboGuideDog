/*
* message.h
* Envolop of messages
*
* Was drv_slamsdp/rp/slamware/message/message.h
*
* Created by Tony Huang at 2016-11-15
* Copyright 2016 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include <rpos/core/rpos_core_config.h>

#include <cstdint>

#define RPOS_MESSAGE_INVALID_SEQ_NUM        ((::std::int64_t)-1)

namespace rpos { namespace message {

	typedef std::uint64_t message_timestamp_t;
	typedef std::int64_t message_seq_num_t;

    struct message_seq_num_range_t
    {
        message_seq_num_t first;
        message_seq_num_t last;

        message_seq_num_range_t(): first(RPOS_MESSAGE_INVALID_SEQ_NUM), last(RPOS_MESSAGE_INVALID_SEQ_NUM) {}
    };

    inline bool isValidMessageSeqNum(message_seq_num_t seqNum) { return 0 < seqNum; }

	template < typename TPayload >
	struct Message
	{
		Message()
			: timestamp(0)
		{
		}

		explicit Message(const TPayload& that)
			: timestamp(0)
			, payload(that)
		{
		}

		message_timestamp_t timestamp;
		TPayload payload;

		inline TPayload& operator*()
		{
			return payload;
		}

		inline TPayload* operator->()
		{
			return &payload;
		}

		inline const TPayload& operator*() const
		{
			return payload;
		}

		inline const TPayload* operator->() const
		{
			return &payload;
		}

		Message<TPayload>& operator=(const TPayload& that)
		{
			payload = that;
			return *this;
		}
	};

	template < typename TPayload >
	static inline bool updateIfNewer(Message<TPayload>& target, const Message<TPayload>& ref)
	{
		if (ref.timestamp > target.timestamp)
		{
			target = ref;
			return true;
		}

		return false;
	}

	template < typename TPayload>
	static inline bool updateIfDifferent(Message<TPayload>& target, const Message<TPayload>& ref)
	{
		if (ref.timestamp != target.timestamp)
		{
			target = ref;
			return true;
		}

		return false;
	}

} }
