#pragma once

#include "config_parser.h"
#include <rpos/core/pose.h>
#include <rpos/core/geometry.h>
#include <rpos/system/util/log.h>

namespace rpos { namespace system { namespace config {

    template <>
    struct RPOS_CORE_API ConfigParser < core::Pose > {
        static bool parse(const Json::Value& config, core::Pose& that);
        static Json::Value generate(const core::Pose& srcCfg, bool withComment = false);
    };

    template <>
    struct RPOS_CORE_API ConfigParser < core::RectangleF > {
        static bool parse(const Json::Value& config, core::RectangleF& that);
        static Json::Value generate(const core::RectangleF& srcCfg, bool withComment = false);
    };

    template <>
    struct RPOS_CORE_API ConfigParser < util::LogConfig > {
        static bool parse(const Json::Value& config, util::LogConfig& that);
        static Json::Value generate(const util::LogConfig& srcCfg, bool withComment = false);
    };

    template <>
    struct RPOS_CORE_API ConfigParser < util::LogAppenderConfig > {
        static bool parse(const Json::Value& config, util::LogAppenderConfig& that);
        static Json::Value generate(const util::LogAppenderConfig& srcCfg, bool withComment = false);
    };

    template <>
    struct RPOS_CORE_API ConfigParser < util::FileLogAppenderConfig > {
        static bool parse(const Json::Value& config, util::FileLogAppenderConfig& that);
        static Json::Value generate(const util::FileLogAppenderConfig& srcCfg, bool withComment = false);
    };

    template <>
    struct RPOS_CORE_API ConfigParser < util::DiagnosisPublisherLogAppenderConfig > {
        static bool parse(const Json::Value& config, util::DiagnosisPublisherLogAppenderConfig& that);
        static Json::Value generate(const util::DiagnosisPublisherLogAppenderConfig& srcCfg, bool withComment = false);
    };

    template <>
    struct RPOS_CORE_API ConfigParser < util::LogLevel > {
        static bool parse(const Json::Value& config, util::LogLevel& that);
        static Json::Value generate(const util::LogLevel& srcCfg, bool withComment = false);
    };

} } }
