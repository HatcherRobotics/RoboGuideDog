/*
* slamware_http_exception.h
* Exceptions shared by all Slamware https client
*
* Originally created by Gabrial He (ykhe@slamtec.com) at 2016-05-09
*
* Copyright (c) 2016~2016 Shanghai SlamTec Co., Ltd.
*/

#pragma once

#include <rpos/robot_platforms/slamware_common_exception.h>
#include <rpos/core/rpos_core_config.h>
#include <rpos/robot_platforms/slamware_sdp_platform_config.h>


namespace rpos { namespace robot_platforms { namespace http {

    class RPOS_SLAMWARE_API HttpException : public rpos::system::detail::ExceptionBase
    {
    public:
        HttpException(const int statusCode, const std::string &msg) throw()
            : rpos::system::detail::ExceptionBase(msg),
             statusCode_(statusCode)
        {}

        virtual ~HttpException() throw()
        {}

        const int getStatus()
        {
            return statusCode_;
        }

    protected:
        int statusCode_;
    };

    class RPOS_SLAMWARE_API RequestHttpException : public HttpException
    {
    public:
        RequestHttpException(const int statusCode, const std::string &msg) throw()
            : HttpException(statusCode, msg)
        {}

        virtual ~RequestHttpException() throw()
        {}
    };

    class RPOS_SLAMWARE_API ServerHttpException : public HttpException
    {
    public:
        ServerHttpException(const int statusCode, const std::string &msg) throw()
            : HttpException(statusCode, msg)
        {}

        virtual ~ServerHttpException() throw()
        {}
    };

} } }
