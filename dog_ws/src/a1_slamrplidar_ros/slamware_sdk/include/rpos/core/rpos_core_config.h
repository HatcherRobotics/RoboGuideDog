/*
* rpos_core_config.h
* Configuration of rpos core module build
*
* Created by Tony Huang (cnwzhjs@gmail.com) at 2012-09-03
* Copyright 2012 (c) www.robopeak.com
*/

#pragma once

#include <rpos/rpos_config.h>

#ifdef RPOS_CORE_DLL
#	ifdef RPOS_CORE_EXPORT
#		define RPOS_CORE_API RPOS_MODULE_EXPORT
#	else
#		define RPOS_CORE_API RPOS_MODULE_IMPORT
#	endif
#else
#	define RPOS_CORE_API
#endif


#if !defined(RPOS_CORE_EXPORT) && !defined(RPOS_CORE_STATIC)
#define RPOS_LIB_NAME rpos_core
#include <rpos/system/util/auto_link.h>
#undef RPOS_LIB_NAME

#define RPOS_LIB_NAME rpos_deps_jsoncpp
#define RPOS_AUTO_LINK_NO_VERSION
#include <rpos/system/util/auto_link.h>
#undef RPOS_AUTO_LINK_NO_VERSION
#undef RPOS_LIB_NAME

#define RPOS_LIB_NAME rpos_deps_base64
#define RPOS_AUTO_LINK_NO_VERSION
#include <rpos/system/util/auto_link.h>
#undef RPOS_AUTO_LINK_NO_VERSION
#undef RPOS_LIB_NAME

#define RPOS_LIB_NAME rpos_deps_rlelib
#define RPOS_AUTO_LINK_NO_VERSION
#include <rpos/system/util/auto_link.h>
#undef RPOS_AUTO_LINK_NO_VERSION
#undef RPOS_LIB_NAME
#endif

/*
* Disable warning C4251: class 'some_template_class' needs to have dll-interface to be used by clients of class 'some_other_class'
* or we will continuously get these warning while compiling to DLL:
* warning C4251: 'rpos::system::util::FileLogAppender::lock_' : class 'boost::mutex' needs to have dll-interface to be used by clients of class 'rpos::system::util::FileLogAppender'
*
* Disable warning C4275:
* It is a problem in boost::noncopyable, the solution to it is to use compilers that support defaulted and deleted methods (which is available at VC14 (Visual C++ 2015)), so, before that
* we have to disable it. BUT it is dangerous to do this, so I add a warning on compilers on windows thats support above feature, so we can re-enable this warning then
*/
#ifdef _WIN32
#   pragma warning(disable: 4251)
#   if defined(BOOST_NO_CXX11_DEFAULTED_FUNCTIONS) || defined(BOOST_NO_CXX11_NON_PUBLIC_DEFAULTED_FUNCTIONS)
#       pragma warning(disable: 4275)
#   endif
#endif
