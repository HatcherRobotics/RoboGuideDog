/*
* i_message_write_stream.h
* IMessageWriteStream
* This file is an abstraction of original MessageWriteStream
*
* Created by Tony Huang (tony@slamtec.com) at 2017-5-13
* Copyright 2017 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include <rpos/core/rpos_core_config.h>
#include <rpos/system/io/memory_write_stream.h>
#include <boost/shared_ptr.hpp>
#include <boost/unordered_map.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>
#include <cstdint>
#include "diagnosis_serialization.h"

namespace rpos { namespace system { namespace diagnosis {

    enum MessageWriteFlags {
        MessageWriteFlagNone = 0,

        /**
        * @brief The message should be write to metadata stream instead of body stream, so the message will not be swaped out when the loop buffer is full
        * This feature is very useful when we are trying to store messages like merged config and etc.
        */
        MessageWriteFlagPersistent = 1
    };

    class RPOS_CORE_API IMessageWriteStream {
    protected:
        IMessageWriteStream();
        virtual ~IMessageWriteStream();

    public:
        template < typename PayloadT >
        void write(const std::string& topic, const rpos::message::Message<PayloadT>& message, MessageWriteFlags flags = MessageWriteFlagNone)
        {
            io::MemoryWriteStream ms;
            serialization::write(ms, message);

            writeMessage(topic, typeid(PayloadT), ms, flags);
        }

    protected:
        virtual void writeMessage(const std::string& topic, const std::type_index& typeIndex, const io::MemoryWriteStream& body, MessageWriteFlags flags) = 0;
    };

} } }
