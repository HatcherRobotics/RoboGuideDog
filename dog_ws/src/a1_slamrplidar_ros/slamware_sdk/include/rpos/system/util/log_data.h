
#pragma once

#include <rpos/rpos_config.h>
#include <rpos/core/rpos_core_config.h>
#include <rpos/system/target_info.h>
#include <string>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <memory>

namespace Json
{
    class Value;
}

namespace rpos { namespace system { namespace util {

    enum LogLevel {
        LogLevelDebug,
        LogLevelInfo,
        LogLevelWarn,
        LogLevelError,
        LogLevelFatal
    };

    typedef std::unique_ptr<Json::Value>            JsonValue_UniquePtr;
    
    RPOS_CORE_API JsonValue_UniquePtr makeUniqueJsonLog();

    struct RPOS_CORE_API LogData
    {
        std::string logSource;
        rpos::system::util::LogLevel logLevel;

    public:
        LogData();
        LogData(const std::string& rcLogSrc, LogLevel tLogLvl);
        LogData(const std::string& rcLogSrc, LogLevel tLogLvl, const char* textLog, size_t textLogLen);
        LogData(const std::string& rcLogSrc, LogLevel tLogLvl, const char* textLog);
        LogData(const std::string& rcLogSrc, LogLevel tLogLvl, const std::string& textLog);
        LogData(const std::string& rcLogSrc, LogLevel tLogLvl, JsonValue_UniquePtr& upJsnLog);

        bool isJsonLog() const { return isJsonLog_; }
        const Json::Value& getJsonOfLog() const { return *jsonOfLog_; }

        const std::string& getStringOfLog() const { return stringOfLog_; }

        void setJsonLog(const Json::Value& jsnLog);
        void setJsonLog(JsonValue_UniquePtr& upJsnLog);
        void setJsonLogByStringOfJson(const char* pcStrOfJson, size_t szStrLen);
        void setJsonLogByStringOfJson(const char* pcStrOfJson);
        void setJsonLogByStringOfJson(const std::string& strOfJson);

        void setTextLog(const char* pcTextLog, size_t szTextLen);
        void setTextLog(const char* pcTextLog);
        void setTextLog(const std::string& textLog);

    private:
        static const boost::shared_ptr<const Json::Value> s_spNullJsonLog;

    private:
        void fastJsonToString_(const Json::Value& jsn, std::string& dest) const;

    private:
        bool isJsonLog_;
        boost::shared_ptr<const Json::Value> jsonOfLog_;
        std::string stringOfLog_;
    };
    typedef boost::shared_ptr<LogData>                  LogData_SharedPtr;
    typedef boost::shared_ptr<const LogData>            ConstLogData_SharedPtr;

}}}
