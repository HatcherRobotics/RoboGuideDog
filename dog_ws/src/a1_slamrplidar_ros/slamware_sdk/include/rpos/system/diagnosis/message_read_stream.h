/*
* message_read_stream.h
* Message read stream
*
* Created by Tony Huang (tony@slamtec.com) at 2017-4-13
* Copyright 2017 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include <rpos/core/rpos_core_config.h>
#include <rpos/system/io/memory_read_stream.h>
#include <rpos/system/diagnosis/message_stream_datatypes.h>
#include <boost/shared_ptr.hpp>
#include <boost/unordered_map.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>
#include <cstdint>
#include "diagnosis_serialization.h"

namespace rpos { namespace system { namespace diagnosis {

    class RPOS_CORE_API MessageReadStream {
    public:
        MessageReadStream(boost::shared_ptr<io::IStream> underlyingStream);
        virtual ~MessageReadStream();

    public:
        bool hasNextMessage() const;
        std::string nextMessageTopic() const;
        std::string nextMessageType() const;
        rpos::message::message_timestamp_t nextMessageTimestamp() const;
        bool isNextMessageOfType(const std::type_index& typeIndex) const;

        template < class PayloadT >
        bool isNextMessage() const
        {
            boost::lock_guard<boost::mutex> guard(lock_);
            return isNextMessageUnlock<PayloadT>();
        }

        template < class PayloadT >
        bool isNextMessage(const std::string& topic) const
        {
            boost::lock_guard<boost::mutex> guard(lock_);
            return isNextMessageUnlock<PayloadT>(topic);
        }

        template < class PayloadT >
        bool read(rpos::message::Message<PayloadT>& outMessage, std::string& outTopic)
        {
            boost::lock_guard<boost::mutex> guard(lock_);

            if (!isNextMessageUnlock<PayloadT>())
                return false;

            outTopic = upcomingTopic_;
            outMessage.timestamp = upcomingTimestamp_;
            size_t payloadLength = upcomingMessageLength_ - sizeof(upcomingTimestamp_);
            std::vector<std::uint8_t> buffer(payloadLength);

            if (payloadLength)
            {
                int readBytes = stream_->read(&buffer[0], payloadLength);
                if (readBytes != (int)payloadLength)
                {
                    clearUpcomingMessage_();
                    return false;
                }
            }

            io::MemoryReadStream ms(std::move(buffer));
            serialization::read(ms, outMessage.payload);

            readToNextMessage_();

            return true;
        }

        void skip();
        void moveToFirstMessage();
        
    private:
        bool isNextMessageOfTypeUnlock(const std::type_index& typeIndex) const;

        template < class PayloadT >
        bool isNextMessageUnlock() const
        {
            return isNextMessageOfTypeUnlock(typeid(PayloadT));
        }

        template < class PayloadT >
        bool isNextMessageUnlock(const std::string& topic) const
        {
            return isNextMessageUnlock<PayloadT>() && upcomingTopic_ == topic;
        }

    private:
        void readToNextMessage_();
        void clearUpcomingMessage_();

    private:
        mutable boost::mutex lock_;
        boost::shared_ptr<io::IStream> stream_;
        boost::unordered_map<std::uint8_t, detail::StreamDescription> streamIdMap_;
        detail::MessageStreamHeader streamHeader_;
        bool isHeaderValid_;

        std::uint32_t upcomingMessageLength_;
        std::uint8_t upcomingStreamId_;
        rpos::message::message_timestamp_t upcomingTimestamp_;
        std::string upcomingTopic_;
    };

} } }
