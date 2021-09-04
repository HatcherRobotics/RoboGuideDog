/*
* diagnosis_serialization.h
* Templated function and struct to read/write objects from/to streams for diagnose
*
* Created by Tony Huang (tony@slamtec.com) at 2016-6-13
* Copyright 2016 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include "../io/i_stream.h"
#include "../../message/message.h"
#include "../../core/geometry.h"
#include <rpos/system/util/log.h>
#include <boost/optional.hpp>

#include <stdint.h>
#include <string>
#include <vector>
#include <list>
#include <map>

namespace rpos { namespace system { namespace diagnosis { namespace serialization {

	template < class T >
	struct Serializer
	{
		static void read(io::IStream& stream, T& v);
		static void write(io::IStream& stream, const T& v);
	};

#define RP_SLAMWARE_TEST_LOGGING_DECLARE_SERIALIZER(T) \
	template <> struct RPOS_CORE_API Serializer<T> { \
		static void read(io::IStream& stream, T& v); \
		static void write(io::IStream& stream, const T& v); \
	};

	RP_SLAMWARE_TEST_LOGGING_DECLARE_SERIALIZER(uint8_t);
	RP_SLAMWARE_TEST_LOGGING_DECLARE_SERIALIZER(uint16_t);
	RP_SLAMWARE_TEST_LOGGING_DECLARE_SERIALIZER(uint32_t);
	RP_SLAMWARE_TEST_LOGGING_DECLARE_SERIALIZER(uint64_t);

	RP_SLAMWARE_TEST_LOGGING_DECLARE_SERIALIZER(int8_t);
	RP_SLAMWARE_TEST_LOGGING_DECLARE_SERIALIZER(int16_t);
	RP_SLAMWARE_TEST_LOGGING_DECLARE_SERIALIZER(int32_t);
	RP_SLAMWARE_TEST_LOGGING_DECLARE_SERIALIZER(int64_t);

	RP_SLAMWARE_TEST_LOGGING_DECLARE_SERIALIZER(float);
	RP_SLAMWARE_TEST_LOGGING_DECLARE_SERIALIZER(double);

	RP_SLAMWARE_TEST_LOGGING_DECLARE_SERIALIZER(bool);
	RP_SLAMWARE_TEST_LOGGING_DECLARE_SERIALIZER(std::string);

    RP_SLAMWARE_TEST_LOGGING_DECLARE_SERIALIZER(rpos::core::Vector3f);
    RP_SLAMWARE_TEST_LOGGING_DECLARE_SERIALIZER(rpos::core::Vector2f);
    RP_SLAMWARE_TEST_LOGGING_DECLARE_SERIALIZER(rpos::core::Vector2i);

	template < class T >
	struct Serializer < std::vector<T> >
	{
		static void read(io::IStream& stream, std::vector<T>& v)
		{
			uint32_t size;
			Serializer<uint32_t>::read(stream, size);
			v.resize(size);

			for (size_t i = 0; i < size; i++)
			{
				Serializer<T>::read(stream, v[i]);
			}
		}

		static void write(io::IStream& stream, const std::vector<T>& v)
		{
			Serializer<uint32_t>::write(stream, (uint32_t)v.size());

			for (size_t i = 0; i < v.size(); i++)
			{
				Serializer<T>::write(stream, v[i]);
			}
		}
	};

	template < class T >
	struct Serializer < std::list<T> >
	{
		static void read(io::IStream& stream, std::list<T>& v)
		{
			uint8_t flag;
			T ele;

			v.clear();

			while (true)
			{
				Serializer<uint8_t>::read(stream, flag);

				if (flag == 0 /* end of block */)
					break;

				Serializer<T>::read(stream, ele);
				v.push_back(v);
			}
		}

		static void write(io::IStream& stream, const std::list<T>& v)
		{
			for (auto iter = v.begin(); iter != v.end(); iter++)
			{
				Serializer<uint8_t>::write(stream, 0x1e /* record */);
				Serializer<T>::write(stream, *iter);
			}

			Serializer<uint8_t>::write(stream, 0);
		}
	};

    template < class K, class T >
    struct Serializer < std::pair<K, T> >
    {
        static void read(io::IStream& stream, std::pair<K, T>& v)
        {
            Serializer<K>::read(stream, v.first);
            Serializer<T>::read(stream, v.second);
        }

        static void write(io::IStream& stream, const std::pair<K, T>& v)
        {
            Serializer<K>::write(stream, v.first);
            Serializer<T>::write(stream, v.second);
        }
    };

	template < class K, class T >
	struct Serializer < std::map<K, T> >
	{
		static void read(io::IStream& stream, std::map<K, T>& v)
		{
			uint8_t flag;
			K key;
			T ele;

			v.clear();

			while (true)
			{
				Serializer<uint8_t>::read(stream, flag);

				if (flag == 0 /* end of block */)
					break;

                std::pair<K, T> keyValuePair;
                Serializer<std::pair<K, T>>::read(stream, keyValuePair);
                v.insert(keyValuePair);
			}
		}

		static void write(io::IStream& stream, const std::map<K, T>& v)
		{
			for (auto iter = v.begin(); iter != v.end(); iter++)
			{
				Serializer<uint8_t>::write(stream, 0x1e /* record */);
                Serializer<std::pair<K, T>>::write(stream, *iter);
			}

			Serializer<uint8_t>::write(stream, 0);
		}
	};

	template < >
	struct RPOS_CORE_API Serializer < std::vector<uint8_t> >
	{
		static void read(io::IStream& stream, std::vector<uint8_t>& v);
		static void write(io::IStream& stream, const std::vector<uint8_t>& v);
	};

    template < typename PayloadT >
    struct Serializer < rpos::message::Message<PayloadT> >
    {
        static void read(io::IStream& stream, rpos::message::Message<PayloadT>& v)
        {
            Serializer<rpos::message::message_timestamp_t>::read(stream, v.timestamp);
            Serializer<PayloadT>::read(stream, v.payload);
        }

        static void write(io::IStream& stream, const rpos::message::Message<PayloadT>& v)
        {
            Serializer<rpos::message::message_timestamp_t>::write(stream, v.timestamp);
            Serializer<PayloadT>::write(stream, v.payload);
        }
    };

    template <>
    struct RPOS_CORE_API Serializer < rpos::system::util::diagnosis::LogData >
    {
        static void read(io::IStream& stream, rpos::system::util::diagnosis::LogData& v);
        static void write(io::IStream& stream, const rpos::system::util::diagnosis::LogData& v);
    };

    template < class T >
    struct RPOS_CORE_API Serializer < boost::optional<T> >
    {
        static void read(io::IStream& stream, boost::optional<T>& v)
        {
            bool hasData;
            Serializer<bool>::read(stream, hasData);

            if (!hasData)
            {
                v = boost::optional<T>();
            }
            else
            {
                T value;
                Serializer<T>::read(stream, value);
                v = value;
            }
        }

        static void write(io::IStream& stream, const boost::optional<T>& v)
        {
            if (v)
            {
                Serializer<bool>::write(stream, true);
                Serializer<T>::write(stream, *v);
            }
            else
            {
                Serializer<bool>::write(stream, false);
            }
        }
    };

	template < class T >
	static inline void read(io::IStream& stream, T& v)
	{
		Serializer<T>::read(stream, v);
	}

	template < class T >
	static inline void write(io::IStream& stream, const T& v)
	{
		Serializer<T>::write(stream, v);
	}

} } } }
