/*
* exception.h
* Exception infrastructure of rpos
*
* Created by Jacky Li (eveningwear@gmail.com) at 2014-08-22
* Copyright (c) 2014 Shanghai SlamTec Co., Ltd.
*/

#pragma once

#include <rpos/core/rpos_core_config.h>
#include <exception>
#include <string>

//e.Init(__FILE__, __PRETTY_FUNCTION__, __LINE__);       
#define RPOS_THROW(ExClass, argv, ...)                             \
do                                                         \
{                                                          \
    ExClass e(argv);                                       \
    throw e;                                               \
}                                                          \
while (false)

#define RP_DEFINE_EXCEPTION(ExClass, Base)                     \
ExClass(const std::string& msg = "") throw()               \
    : Base(msg)                                            \
{}                                                         \
                                                           \
~ExClass() throw() {}                                        \
                                                           \
/* override */ std::string getClassName() const            \
{                                                          \
    return #ExClass;                                       \
}

namespace rpos { namespace system { namespace detail {

    class RPOS_CORE_API ExceptionBase : public std::exception
    {
    public:
        ExceptionBase(const std::string& msg = "") throw();
             
        virtual ~ExceptionBase() throw();
        
        void Init(const char* file, const char* func, int line);
        
        virtual std::string getClassName() const;
        
        virtual std::string getMessage() const;
        
        const char* what() const throw();
        
        const std::string& toString() const;
             
        //std::string getStackTrace() const;
             
    protected:
        std::string mMsg;
        const char* mFile;
        const char* mFunc;
        int mLine;
             
    private:
        enum { MAX_STACK_TRACE_SIZE = 50 };
        void* mStackTrace[MAX_STACK_TRACE_SIZE];
        size_t mStackTraceSize;
        mutable std::string mWhat;
    };

    class ExceptionDerived : public ExceptionBase
    {
    public:
        RP_DEFINE_EXCEPTION(ExceptionDerived, ExceptionBase);
    };

} } }
