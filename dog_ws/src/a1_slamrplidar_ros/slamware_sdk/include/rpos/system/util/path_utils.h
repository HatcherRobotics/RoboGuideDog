#pragma once

#include <rpos/core/rpos_core_config.h>
#include <string>
#include <vector>

#ifdef _WIN32
#   define RPOS_SYSTEM_UTIL_PATH_SPLITTER       '\\'
#   define RPOS_SYSTEM_UTIL_OTHER_PATH_SPLITTER '/'
#else
#   define RPOS_SYSTEM_UTIL_PATH_SPLITTER       '/'
#   define RPOS_SYSTEM_UTIL_OTHER_PATH_SPLITTER '\\'
#endif

namespace rpos { namespace system { namespace util {

    /**
    * \brief Normailize path
    */
    RPOS_CORE_API std::string normalize_path(const std::string& path);

    /**
    * \brief Simplify path (this will eliminate all '..' and '.' part of path)
    */
    RPOS_CORE_API std::string simplify_path(const std::string& path);

    /**
    * \brief Get parent path
    */
    RPOS_CORE_API std::string get_parent_path(const std::string& path);

    /**
    * \brief Get filename
    */
    RPOS_CORE_API std::string get_filename(const std::string& path);

    /**
    * \brief Get extension filename (xxoo.xls => xls)
    */
    RPOS_CORE_API std::string get_extension(const std::string& path);

    /**
    * \brief Get path parts
    */
    RPOS_CORE_API std::vector<std::string> path_split(const std::string& path);

    /**
    * \brief Concat path
    */
    RPOS_CORE_API std::string path_concat(const std::string& a, const std::string& b);

    /**
    * \brief Concat path
    */
    RPOS_CORE_API std::string path_concat(const std::vector<std::string>& parts);

} } }
