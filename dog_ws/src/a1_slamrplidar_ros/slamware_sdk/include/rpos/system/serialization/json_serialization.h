﻿/*
* json_serialization.h
* RPOS JSON Serialization Utilities
*
* Created By Tony Huang (cnwzhjs@gmail.com) at 2014-12-11
* Copyright 2014 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include <rpos/rpos_config.h>
#include <rpos/core/rpos_core_config.h>
#include <rpos/core/pose.h>
#include <json/json.h>
#include <vector>
#include <list>
#include <map>
#include <cstdint>
#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#define RPOS_LIB_NAME rpos_deps_jsoncpp
#define RPOS_AUTO_LINK_NO_VERSION
#	include <rpos/system/util/auto_link.h>
#undef RPOS_AUTO_LINK_NO_VERSION
#undef RPOS_LIB_NAME

namespace rpos { namespace system { namespace serialization { namespace json {

    template <class T>
    struct Serializer
    {
        static Json::Value serialize(const T& v);
        static T deserialize(const Json::Value& jsonValue);
    };

    template <class T>
    Json::Value serialize(const T& v)
    {
        return Serializer<T>::serialize(v);
    }

    template <class T>
    T deserialize(const Json::Value& v)
    {
        return Serializer<T>::deserialize(v);
    }

    template <>
    struct Serializer < std::nullptr_t >
    {
        static Json::Value serialize(const std::nullptr_t& v)
        {
            return Json::Value();
        }

        static std::nullptr_t deserialize(const Json::Value& v)
        {
            return nullptr;
        }
    };

    template <>
    struct Serializer < Json::Value >
    {
        static Json::Value serialize(const Json::Value& v)
        {
            return v;
        }

        static Json::Value deserialize(const Json::Value& v)
        {
            return v;
        }
    };

    template <>
    struct Serializer < std::string >
    {
        static Json::Value serialize(const std::string& v)
        {
            return Json::Value(v);
        }

        static std::string deserialize(const Json::Value& v)
        {
            return v.asString();
        }
    };

    template <>
    struct Serializer < int >
    {
        static Json::Value serialize(const int& v)
        {
            return Json::Value(Json::Int(v));
        }

        static int deserialize(const Json::Value& v)
        {
            return v.asInt();
        }
    };

    template <>
    struct Serializer < long >
    {
        static Json::Value serialize(const long& v)
        {
            return Json::Value(Json::Int(v));
        }

        static long deserialize(const Json::Value& v)
        {
            return v.asInt();
        }
    };

    template <>
    struct Serializer < long long >
    {
        static Json::Value serialize(const long long& v)
        {
            return Json::Value(Json::Int(v));
        }

        static long long deserialize(const Json::Value& v)
        {
            return v.asInt();
        }
    };

    template <>
    struct Serializer < unsigned int >
    {
        static Json::Value serialize(const unsigned int& v)
        {
            return Json::Value(Json::UInt(v));
        }

        static unsigned int deserialize(const Json::Value& v)
        {
            return v.asUInt();
        }
    };

    template <>
    struct Serializer < unsigned long >
    {
        static Json::Value serialize(const unsigned long& v)
        {
            return Json::Value(Json::UInt(v));
        }

        static unsigned long deserialize(const Json::Value& v)
        {
            return v.asUInt();
        }
    };

    template <>
    struct Serializer < unsigned long long >
    {
        static Json::Value serialize(const unsigned long long& v)
        {
            return Json::Value(Json::UInt(v));
        }

        static unsigned long long deserialize(const Json::Value& v)
        {
            return v.asUInt();
        }
    };

    template <>
    struct Serializer < bool >
    {
        static Json::Value serialize(const bool& v)
        {
            return Json::Value(v);
        }

        static bool deserialize(const Json::Value& v)
        {
            return v.asBool();
        }
    };

    template <>
    struct Serializer < float >
    {
        static Json::Value serialize(const float& v)
        {
            return Json::Value(v);
        }

        static float deserialize(const Json::Value& v)
        {
            return (float)v.asDouble();
        }
    };

    template <>
    struct Serializer < double >
    {
        static Json::Value serialize(const double& v)
        {
            return Json::Value(v);
        }

        static double deserialize(const Json::Value& v)
        {
            return (double)v.asDouble();
        }
    };

    template <class T>
    struct Serializer < std::map<std::string, T> >
    {
        static Json::Value serialize(const std::map<std::string, T>& v)
        {
            Json::Value output;

            for (auto iter = v.begin(); iter != v.end(); iter++)
            {
                output[iter->first] = json::serialize(iter->second);
            }
            return output;
        }

        static std::map<std::string, T> deserialize(const Json::Value& v)
        {
            std::map<std::string, T> output;

            std::vector<std::string> memberNames = v.getMemberNames();

            for (auto iter = memberNames.begin(); iter != memberNames.end(); iter++)
            {
                output[*iter] = json::deserialize<T>(v[*iter]);
            }

            return output;
        }
    };

    template <class T>
    struct Serializer < std::vector<T> >
    {
        static Json::Value serialize(const std::vector<T>& v)
        {
            Json::Value output(Json::arrayValue);

            for (auto iter = v.begin(); iter != v.end(); iter++)
            {
                output.append(json::serialize(*iter));
            }

            return output;
        }

        static std::vector<T> deserialize(const Json::Value& v)
        {
            std::vector<T> output;

            for (size_t i = 0; i < v.size(); i++)
            {
                output.push_back(json::deserialize<T>(v[i]));
            }

            return output;
        }
    };

    template<class ValT>
    struct Serializer< boost::shared_ptr< std::vector<ValT> > >
    {
    public:
        typedef ValT                                value_t;
        typedef std::vector<value_t>                vector_t;
        typedef boost::shared_ptr<vector_t>         vector_shared_ptr;
        typedef boost::shared_ptr<const vector_t>   const_vector_shared_ptr;

    public:
        static Json::Value serialize(const const_vector_shared_ptr& vals)
        {
            if (vals)
            {
                Json::Value jsnVal(Json::arrayValue);
                serializeHelp_(*vals, jsnVal);
                return jsnVal;
            }
            return Json::Value(Json::nullValue);
        }
        static Json::Value serialize(const vector_shared_ptr& vals)
        {
            if (vals)
            {
                Json::Value jsnVal(Json::arrayValue);
                serializeHelp_(*vals, jsnVal);
                return jsnVal;
            }
            return Json::Value(Json::nullValue);
        }

        static vector_shared_ptr deserialize(const Json::Value& jsnVal)
        {
            if (jsnVal.isNull())
                return nullptr;

            auto vals = boost::make_shared<vector_t>();
            const auto cnt = jsnVal.size();
            vals->reserve(cnt);
            for (Json::Value::UInt u = 0; u < cnt; ++u)
            {
                vals->push_back(json::deserialize<value_t>(jsnVal[u]));
            }
            return vals;
        }

    private:
        static void serializeHelp_(const vector_t& vals, Json::Value& jsnDest)
        {
            for (auto cit = vals.begin(), citEnd = vals.end(); citEnd != cit; ++cit)
            {
                jsnDest.append(json::serialize(*cit));
            }
        }
    };

    template <class T>
    struct Serializer < std::list<T> >
    {
        static Json::Value serialize(const std::list<T>& v)
        {
            Json::Value output(Json::arrayValue);

            for (auto iter = v.begin(); iter != v.end(); iter++)
            {
                output.append(json::serialize(*iter));
            }

            return output;
        }

        static std::list<T> deserialize(const Json::Value& v)
        {
            std::list<T> output;

            for (size_t i = 0; i < v.size(); i++)
            {
                output.push_back(json::deserialize<T>(v[i]));
            }

            return output;
        }
    };

    template <class T>
    struct Serializer < boost::optional<T> >
    {
        static Json::Value serialize(const boost::optional<T>& v)
        {
            Json::Value output;

            output = json::serialize(*v);
            output["optional"] = bool(v);

            return output;
        }

        static boost::optional<T> deserialize(const Json::Value& v)
        {
            boost::optional<T> output;
            bool optional = true;

            if(v.isMember("optional"))
            {
                optional = v["optional"].asBool();
            }

            if(optional)
            {
                output = json::deserialize<T>(v);
            }

            return output;
        }
    };

    template <>
    struct RPOS_CORE_API Serializer < std::vector < rpos::core::Location > >
    {
        static Json::Value serialize(const std::vector< rpos::core::Location >& v);
        static std::vector< rpos::core::Location > deserialize(const Json::Value& v);
    };


    template <>
    struct RPOS_CORE_API Serializer < std::vector < std::uint8_t > >
    {
        static Json::Value serialize(const std::vector< std::uint8_t >& v);
        static std::vector< std::uint8_t > deserialize(const Json::Value& v);
    };

    //////////////////////////////////////////////////////////////////////////

    RPOS_CORE_API Json::Value uint64ToJsonStringValue(std::uint64_t ui64Val);
    RPOS_CORE_API std::uint64_t jsonStringValueToUInt64(const Json::Value& jsnVal);

    RPOS_CORE_API Json::Value int64ToJsonStringValue(std::int64_t i64Val);
    RPOS_CORE_API std::int64_t jsonStringValueToInt64(const Json::Value& jsnVal);

}}}}
