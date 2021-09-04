#pragma once
#include <vector>
#include <rpos/system/types.h>
#include <rpos/core/rpos_core_config.h>
#include <rpos/system/io/i_stream.h>

namespace rpos { namespace system { namespace serialization {

    class RPOS_CORE_API BufferStreamAdaptor : public rpos::system::io::IStream
    {
    public:
        explicit BufferStreamAdaptor(std::vector<system::types::_u8> *buf);
        virtual ~BufferStreamAdaptor();

        virtual bool isOpen();
        virtual bool canRead();
        virtual bool canWrite();
        virtual bool canSeek();

        virtual void close();

        virtual bool endOfStream();

        virtual int read(void *buffer, size_t count);
        virtual int write(const void *buffer, size_t count);
        virtual size_t tell();
        virtual void seek(rpos::system::io::SeekType type, int offset);

    private:
        std::vector<system::types::_u8> *buf_;
        size_t head_;
    };

} } }
