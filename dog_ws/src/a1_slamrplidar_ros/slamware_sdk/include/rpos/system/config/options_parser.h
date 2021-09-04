#pragma once

#include <string>
#include <map>
#include <vector>
#include <boost/noncopyable.hpp>
#include <rpos/core/rpos_core_config.h>

namespace rpos { namespace system { namespace config {

    struct RPOS_CORE_API Option {
        Option()
            : acceptArgument(false)
            , exists(false)
        {}

        std::string name;
        std::string description;
        std::string shortOption;
        std::string longOption;
        bool acceptArgument;

        bool exists;
        std::string argument;
    };

    class RPOS_CORE_API OptionParser : private boost::noncopyable {
    public:
        OptionParser& addOption(Option* option);
        
        bool parse(int argc, const char* argv[], std::vector<std::string>& outRestArguments);
		void printHelp();

    public:
        OptionParser& addVersion(Option* option);
        OptionParser& addHelp(Option* option);

    private:
        std::map<std::string, Option*> shortOptions_;
        std::map<std::string, Option*> longOptions_;
    };

} } }
