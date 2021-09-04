#pragma once

#include <rpos/system/io/i_stream.h>
#include <rpos/robot_platforms/objects/composite_map.h>

#include <boost/noncopyable.hpp>

namespace rpos { namespace robot_platforms { namespace objects {

    class CompositeMapWriterImpl;

    class RPOS_SLAMWARE_API CompositeMapWriter : private boost::noncopyable
    {
    public:
        CompositeMapWriter(void);
        ~CompositeMapWriter(void);

    public:
        // throw exception if error occurs

        void saveFile(const std::string& rcFilePath, const CompositeMap& rcCmpstMap);
        void saveFile(const std::wstring& rcFilePath, const CompositeMap& rcCmpstMap);

        void saveStream(rpos::system::io::IStream& outStream, const CompositeMap& rcCmpstMap);

    public:
        // returns true if succeed, and "rErrMsg" will be empty;
        // returns false if error occurs, and error message will be in "rErrMsg".

        bool saveFile(std::string& rErrMsg, const std::string& rcFilePath, const CompositeMap& rcCmpstMap);
        bool saveFile(std::string& rErrMsg, const std::wstring& rcFilePath, const CompositeMap& rcCmpstMap);

        bool saveStream(std::string& rErrMsg, rpos::system::io::IStream& outStream, const CompositeMap& rcCmpstMap);

    private:
        bool doSaveToStream_(std::string& rErrMsg, rpos::system::io::IStream& outStream, const CompositeMap& rcCmpstMap);

    private:
        CompositeMapWriterImpl* m_pImpl;
    };

}}}
