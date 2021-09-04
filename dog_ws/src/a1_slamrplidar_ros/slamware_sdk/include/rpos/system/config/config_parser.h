#pragma once

#if defined(_WIN32) && !defined(_CRT_SECURE_NO_WARNINGS)
#   define _CRT_SECURE_NO_WARNINGS
#endif

#include <rpos/core/rpos_core_config.h>
#include <json/json.h>
#include <vector>
#include <map>
#include <list>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <rpos/system/util/uom.h>
#include <openssl/rsa.h>
#include <openssl/bio.h>
#include <openssl/pem.h>
#include <boost/optional.hpp>

#include <boost/assert.hpp>
#include <boost/optional.hpp>

#define RPOS_CONFIG_NOT_USED(x)       ((void)(x))

#define RPOS_CONFIG_GEN_MEMBER(mbName)                      ::rpos::system::config::generateJsonOfConfig(srcCfg.mbName, jsnRes, #mbName, withComment)
#define RPOS_CONFIG_GEN_MEMBER_BY_VAL(mbName, mbVal)        ::rpos::system::config::generateJsonOfConfig(mbVal, jsnRes, #mbName, withComment)

#define RPOS_CONFIG_GEN_COMMENT_MEMBER(mbName, strComment, commentPos)                      ::rpos::system::config::generateAndCommentJsonOfConfig(srcCfg.mbName, withComment, #mbName, strComment, commentPos, jsnRes)
#define RPOS_CONFIG_GEN_COMMENT_MEMBER_BY_VAL(mbName, mbVal, strComment, commentPos)        ::rpos::system::config::generateAndCommentJsonOfConfig(mbVal, withComment, #mbName, strComment, commentPos, jsnRes)

#define RPOS_CONFIG_GEN_COMMENT_BEFORE_MEMBER(mbName, strComment)                   RPOS_CONFIG_GEN_COMMENT_MEMBER(mbName, strComment, Json::commentBefore)
#define RPOS_CONFIG_GEN_COMMENT_ON_MEMBER(mbName, strComment)                       RPOS_CONFIG_GEN_COMMENT_MEMBER(mbName, strComment, Json::commentAfterOnSameLine)
#define RPOS_CONFIG_GEN_COMMENT_AFTER_MEMBER(mbName, strComment)                    RPOS_CONFIG_GEN_COMMENT_MEMBER(mbName, strComment, Json::commentAfter)

#define RPOS_CONFIG_GEN_COMMENT_BEFORE_MEMBER_BY_VAL(mbName, mbVal, strComment)                     RPOS_CONFIG_GEN_COMMENT_MEMBER_BY_VAL(mbName, mbVal, strComment, Json::commentBefore)
#define RPOS_CONFIG_GEN_COMMENT_ON_MEMBER_BY_VAL(mbName, mbVal, strComment)                         RPOS_CONFIG_GEN_COMMENT_MEMBER_BY_VAL(mbName, mbVal, strComment, Json::commentAfterOnSameLine)
#define RPOS_CONFIG_GEN_COMMENT_AFTER_MEMBER_BY_VAL(mbName, mbVal, strComment)                      RPOS_CONFIG_GEN_COMMENT_MEMBER_BY_VAL(mbName, mbVal, strComment, Json::commentAfter)

namespace rpos { namespace system { namespace config {

    extern RPOS_CORE_API bool rawConfigError(const char* msg, ...);
    extern RPOS_CORE_API bool configError(const Json::Value& config, const char* msg, ...);
    extern RPOS_CORE_API Json::Value decryptConfigFile(const std::string & filename);

    inline Json::Value integerToJson(Json::Int val) { return Json::Value(val); }
    inline Json::Value integerToJson(Json::UInt val) { return Json::Value(val); }
    RPOS_CORE_API Json::Value integerToJson(std::int64_t val);
    RPOS_CORE_API Json::Value integerToJson(std::uint64_t val);

    template<class T>
    struct ConfigParser {
        static bool parse(const Json::Value& config, T& v);
        static Json::Value generate(const T& srcCfg, bool withComment = false);
    };

    template<class T>
    bool parseConfig(const Json::Value& config, T& that)
    {
        return ConfigParser<T>::parse(config, that);
    }

    template<class T>
    Json::Value generateJsonValueOfConfig(const T& srcCfg, bool withComment = false)
    {
        return ConfigParser<T>::generate(srcCfg, withComment);
    }

    template<class T>
    void generateJsonOfConfig(const T& srcMbCfg, Json::Value& jsnRes, const std::string& mbName, bool withComment = false)
    {
        jsnRes[mbName] = generateJsonValueOfConfig(srcMbCfg, withComment);
    }
    template<class T>
    void generateJsonOfConfig(const boost::optional<T> & srcMbCfg, Json::Value& jsnRes, const std::string& mbName, bool withComment = false)
    {
        if (srcMbCfg)
            jsnRes[mbName] = generateJsonValueOfConfig(*srcMbCfg, withComment);
        else
            jsnRes.removeMember(mbName);
    }

    RPOS_CORE_API void adjustCommentForJson(std::string& strInOutComment, Json::CommentPlacement commentPos);

    template<class T>
    void generateAndCommentJsonOfConfig(const T& srcMbCfg, bool withComment, const std::string& mbName, const std::string& strComment, Json::CommentPlacement commentPos, Json::Value& jsnRes)
    {
        auto& jsnMember = jsnRes[mbName];
        jsnMember = generateJsonValueOfConfig(srcMbCfg, withComment);
        if (withComment)
        {
            std::string tmpComment = strComment;
            adjustCommentForJson(tmpComment, commentPos);
            jsnMember.setComment(tmpComment, commentPos);
        }
    }
    template<class T>
    void generateAndCommentJsonOfConfig(const boost::optional<T> & srcMbCfg, bool withComment, const std::string& mbName, const std::string& strComment, Json::CommentPlacement commentPos, Json::Value& jsnRes)
    {
        if (srcMbCfg)
            generateAndCommentJsonOfConfig(*srcMbCfg, withComment, mbName, strComment, commentPos, jsnRes);
        else
            jsnRes.removeMember(mbName);
    }

    template<class T>
    bool parseConfigFromFile(const std::string& filename, T& that)
    {
        FILE* file = fopen(filename.c_str(), "rb");

        if (!file)
            return rawConfigError("Failed to open config file: %s", filename.c_str());

        fseek(file, 0, SEEK_END);

        size_t fileSize = ftell(file);

        char* buffer = (char*)malloc(fileSize);

        if (!buffer)
        {
            fclose(file);
            return rawConfigError("Failed to allocate read buffer of %d bytes for file: %s", (int)fileSize, filename.c_str());
        }

        fseek(file, 0, SEEK_SET);
        size_t read = fread(buffer, 1, fileSize, file);

        if (read != fileSize)
        {
            free(buffer);
            fclose(file);
            return rawConfigError("Failed to read config file: %s", filename.c_str());
        }

        fclose(file);

        Json::Value config;
        Json::Reader reader;

        if (!reader.parse(buffer, buffer + fileSize, config, false))
        {
            free(buffer);
            return rawConfigError("Failed to parse config file: %s", filename.c_str());
        }
        
        free(buffer);

        return parseConfig(config, that);
    }

    template<class T>
    bool parseConfigFromEncryptedFile(const std::string& filename, T& that)
    {
        auto config = decryptConfigFile(filename);
        return (!config.empty()) && parseConfig(config, that);
    }

    template<class T>
    bool parseChildConfigWithDefault(const Json::Value& config, const std::string& key, T& that, const T& defaultValue)
    {
        if (!config.isObject())
            return configError(config, "Failed to parse child `%s': config node is not a JSON object", key.c_str());

        if (!config.isMember(key))
        {
            that = defaultValue;
            return true;
        }

        return parseConfig(config[key], that);
    }

    template<class T>
    bool parseChildConfig(const Json::Value& config, const std::string& key, T& that)
    {
        if (!config.isObject())
            return configError(config, "Failed to parse child `%s': config node is not a JSON object", key.c_str());

        if (!config.isMember(key))
        {
            if (parseConfig(Json::Value(), that))
                return true;

            return configError(config, "Child `%s' not found", key.c_str());
        }

        return parseConfig(config[key], that);
    }

#define CONFIG_PARSE_CHILD(Name) \
    do { \
        if (!parseChildConfig(config, #Name, that.Name)) \
            return configError(config, "Field: %s", #Name); \
    } while (false)

#define CONFIG_PARSE_CHILD_WITH_DEFAULT(Name, Default) \
    do { \
        if (!parseChildConfigWithDefault(config, #Name, that.Name, Default)) \
            return configError(config, "Field: %s", #Name); \
    } while (false)

    //
    // ------- Begin Composite Types Data Parsers -------
    //
    template<class T>
    struct ConfigParser < std::vector<T> > {
        static bool parse(const Json::Value& config, std::vector<T>& v)
        {
            if (!config.isArray())
                return configError(config, "Failed to parse array: config node is not a JSON array");

            v.clear();
            for (size_t i = 0; i < config.size(); i++)
            {
                T item;

                if (!ConfigParser<T>::parse(config[i], item))
                    return false;

                v.push_back(item);
            }

            return true;
        }

        static Json::Value generate(const std::vector<T> & srcCfg, bool withComment = false)
        {
            Json::Value jsnRes(Json::arrayValue);
            for (auto cit = srcCfg.cbegin(), citEnd = srcCfg.cend(); citEnd != cit; ++cit)
            {
                auto tmpJsn = ConfigParser<T>::generate(*cit, withComment);
                jsnRes.append(tmpJsn);
            }
            return jsnRes;
        }
    };

    template<class T>
    struct ConfigParser < std::list<T> > {
        static bool parse(const Json::Value& config, std::list<T>& v)
        {
            if (!config.isArray())
                return configError(config, "Failed to parse array: config node is not a JSON array");

            v.clear();
            for (size_t i = 0; i < config.size(); i++)
            {
                T item;

                if (!ConfigParser<T>::parse(config[i], item))
                    return false;

                v.push_back(item);
            }

            return true;
        }

        static Json::Value generate(const std::list<T> & srcCfg, bool withComment = false)
        {
            Json::Value jsnRes(Json::arrayValue);
            for (auto cit = srcCfg.cbegin(), citEnd = srcCfg.cend(); citEnd != cit; ++cit)
            {
                auto tmpJsn = ConfigParser<T>::generate(*cit, withComment);
                jsnRes.append(tmpJsn);
            }
            return jsnRes;
        }
    };

    template<class T>
    struct ConfigParser < std::map<std::string, T> > {
        static bool parse(const Json::Value& config, std::map<std::string, T>& v)
        {
            if (!config.isObject())
                return configError(config, "Failed to parse key value store: config node is not a JSON object");

            Json::Value::Members members = config.getMemberNames();
            
            v.clear();
            for (Json::Value::Members::iterator iter = members.begin(); iter != members.end(); iter++)
            {
                T item;

                if (!ConfigParser<T>::parse(config[*iter], item))
                    return false;
                
                v[*iter] = item;
            }

            return true;
        }

        static Json::Value generate(const std::map<std::string, T> & srcCfg, bool withComment = false)
        {
            Json::Value jsnRes(Json::objectValue);
            for (auto cit = srcCfg.cbegin(), citEnd = srcCfg.cend(); citEnd != cit; ++cit)
            {
                auto tmpJsn = ConfigParser<T>::generate(cit->second, withComment);
                jsnRes[cit->first] = tmpJsn;
            }
            return jsnRes;
        }
	};

    template < class T >
    struct ConfigParser < boost::optional<T> > {
        static bool parse(const Json::Value& config, boost::optional<T>& that)
        {
            if (config.isNull())
            {
                that.reset();
                return true;
            }
            else
            {
                T value;
                bool result = ConfigParser<T>::parse(config, value);

                if (!result)
                    return result;

                that = value;
                return true;
            }
        }
    };

	//
	// ------- Begin Basic Types Data Parsers -------
	//
#define SIGNED_INT_CONFIG_PARSER(Type) \
    template<> struct ConfigParser < Type > { \
        static bool parse(const Json::Value& config, Type& v) {\
			if (config.isString()) { \
				if (!util::try_parse_with_unit(config.asString(), v)) \
					return configError(config, "Failed to parse numberic value: string cannot be parsed with uom subsystem"); \
				else \
					return true; \
			} else if (!config.isNumeric()) { \
				return configError(config, "Failed to parse numberic value: config node is not a number"); \
			} \
            v = (Type)config.asInt(); \
            return true; \
        } \
         \
        static Json::Value generate(const Type& srcCfg, bool withComment = false) \
        { \
            RPOS_CONFIG_NOT_USED(withComment); \
            return integerToJson(srcCfg); \
        } \
    };

#define UNSIGNED_INT_CONFIG_PARSER(Type) \
    template<> struct ConfigParser < Type > { \
        static bool parse(const Json::Value& config, Type& v) {\
			if (config.isString()) { \
				if (!util::try_parse_with_unit(config.asString(), v)) \
					return configError(config, "Failed to parse numberic value: string cannot be parsed with uom subsystem"); \
				else \
					return true; \
			} else if (!config.isNumeric()) { \
				return configError(config, "Failed to parse numberic value: config node is not a number"); \
			} \
            v = (Type)config.asUInt(); \
            return true; \
        } \
        \
        static Json::Value generate(const Type& srcCfg, bool withComment = false) \
        { \
            RPOS_CONFIG_NOT_USED(withComment); \
            return integerToJson(srcCfg); \
        } \
    };

#define FLOAT_CONFIG_PARSER(Type) \
    template<> struct ConfigParser < Type > { \
        static bool parse(const Json::Value& config, Type& v) {\
			if (config.isString()) { \
				if (!util::try_parse_with_unit(config.asString(), v)) \
					return configError(config, "Failed to parse numberic value: string cannot be parsed with uom subsystem"); \
				else \
					return true; \
			} else if (!config.isNumeric()) { \
				return configError(config, "Failed to parse numberic value: config node is not a number"); \
			} \
            v = (Type)config.asDouble(); \
            return true; \
        } \
        \
        static Json::Value generate(const Type& srcCfg, bool withComment = false) \
        { \
            RPOS_CONFIG_NOT_USED(withComment); \
            return Json::Value(srcCfg); \
        } \
    };

    SIGNED_INT_CONFIG_PARSER(signed char);
    SIGNED_INT_CONFIG_PARSER(short);
    SIGNED_INT_CONFIG_PARSER(int);
    SIGNED_INT_CONFIG_PARSER(std::int64_t);

    UNSIGNED_INT_CONFIG_PARSER(unsigned char);
    UNSIGNED_INT_CONFIG_PARSER(unsigned short);
    UNSIGNED_INT_CONFIG_PARSER(unsigned int);
    UNSIGNED_INT_CONFIG_PARSER(std::uint64_t);

    FLOAT_CONFIG_PARSER(float);
    FLOAT_CONFIG_PARSER(double);

    template<> struct ConfigParser < std::string > {
        static bool parse(const Json::Value& config, std::string& v)
        {
            if (!config.isString())
                return configError(config, "Failed to parse string value: config node is not a string");

            v = config.asString();

            return true;
        }

        static Json::Value generate(const std::string& srcCfg, bool withComment = false)
        {
            RPOS_CONFIG_NOT_USED(withComment);
            return Json::Value(srcCfg);
        }
    };

    template<> struct ConfigParser < bool > {
        static bool parse(const Json::Value& config, bool& v)
        {
            if (!config.isBool())
                return configError(config, "Failed to parse boolean value: config node is not a bool");

            v = config.asBool();

            return true;
        }

        static Json::Value generate(const bool& srcCfg, bool withComment = false)
        {
            RPOS_CONFIG_NOT_USED(withComment);
            return Json::Value(srcCfg);
        }
    };

    template<> struct ConfigParser < Json::Value > {
        static bool parse(const Json::Value& config, Json::Value& v) {
            v = config;
            return true;
        }

        static Json::Value generate(const Json::Value& srcCfg, bool withComment = false)
        {
            RPOS_CONFIG_NOT_USED(withComment);
            return srcCfg;
        }
    };


    template < class TConfig >
    struct ServiceConfig {

        static bool sfIsDefaultEnabled();

        bool enabled;
        TConfig config;
    };
    template<class TConfig>
    inline bool ServiceConfig<TConfig>::sfIsDefaultEnabled()
    {
        return false;
    }

    template < class TConfig >
    struct ConfigParser < ServiceConfig<TConfig> > {
        static bool parse(const Json::Value& config, ServiceConfig<TConfig>& that)
        {
            const bool bDefaultEnabled = ServiceConfig<TConfig>::sfIsDefaultEnabled();
            CONFIG_PARSE_CHILD_WITH_DEFAULT(enabled, bDefaultEnabled);
            return ConfigParser<TConfig>::parse(config, that.config);
        }

        static Json::Value generate(const ServiceConfig<TConfig> & srcCfg, bool withComment = false)
        {
            Json::Value jsnRes = ConfigParser<TConfig>::generate(srcCfg.config, withComment);
            RPOS_CONFIG_GEN_MEMBER(enabled);
            return jsnRes;
        }
    };

} } }

#define DECLARE_CONFIG_PARSER(Type) \
    template<> \
    struct ConfigParser < Type > \
    { \
        static bool parse(const Json::Value& config, Type& that); \
        static Json::Value generate(const Type & srcCfg, bool withComment = false); \
    };

#define BEGIN_CONFIG_PARSER(Type) bool ConfigParser < Type > :: parse(const Json::Value& config, Type& that) {
#define END_CONFIG_PARSER }

