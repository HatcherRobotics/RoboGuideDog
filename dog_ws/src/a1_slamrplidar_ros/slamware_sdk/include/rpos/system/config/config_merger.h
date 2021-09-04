#pragma once

#include <rpos/core/rpos_core_config.h>
#include <json/json.h>


namespace rpos { namespace system { namespace config {

    class RPOS_CORE_API IConfigItemCombiner
    {
    public:
        virtual ~IConfigItemCombiner() {}

        // returns the combined json value of items.
        virtual Json::Value combineItem(const Json::Value& customized, const Json::Value& ref) = 0;
    };

    class RPOS_CORE_API DefaultConfigItemCombiner: public IConfigItemCombiner
    {
    public:
        DefaultConfigItemCombiner() {}
        virtual ~DefaultConfigItemCombiner() {}

        virtual Json::Value combineItem(const Json::Value& customized, const Json::Value& ref);
    };

    class RPOS_CORE_API UniqueIdConfigItemCombiner: public IConfigItemCombiner
    {
    public:
        UniqueIdConfigItemCombiner();
        virtual ~UniqueIdConfigItemCombiner() {}

        virtual Json::Value combineItem(const Json::Value& customized, const Json::Value& ref);

    public:
        const std::string& getKey() const { return key_; }
        void setKey(const std::string& key) { key_ = key; }

    private:
        void uniqueOverwriteToArray_(const Json::Value& jsnElem, Json::Value& jsnDestArr) const;

    private:
        std::string key_;
    };

    //////////////////////////////////////////////////////////////////////////

    class RPOS_CORE_API ConfigMerger
    {
    private:
        static DefaultConfigItemCombiner s_defaultCfgItemCombiner;

    private:
        static IConfigItemCombiner* getDefaultConfigItemCombiner();

    public:
        /*merge reference config into ccustomized config according to white list policy
        @customized: config file from customers
        @ref: our reference config
        @whiteList: policy written in Json format that specify whether customer has the authority to config a field.
                    only fields that marked "accept" can be used from @customized
                    if a field ia json array type and marked in whiteList as "combine", then merged config will contain a union of the array
        */
        static bool mergeConfigInplace(Json::Value& customized, const Json::Value& ref
            , const Json::Value& whiteList, const std::vector<std::string>& ignoreList
            , IConfigItemCombiner* cfgItemCombiner = nullptr
            );
		static bool mergeConfigInplace(Json::Value& customized, const Json::Value& ref
            , const Json::Value& whiteList, const std::string& path, const std::vector<std::string>& ignoreList
            , IConfigItemCombiner* cfgItemCombiner = nullptr
            );
        static bool overwriteConfigInplace(const Json::Value& customized, Json::Value& ref);

        static Json::Value mergeConfig(const Json::Value& customized, const Json::Value& ref
            , const Json::Value& whiteList, const std::vector<std::string>& ignoreList
            , IConfigItemCombiner* cfgItemCombiner = nullptr
            );
        //overwrite all contents in ref with customized
        static Json::Value overwriteConfig(const Json::Value& customized, const Json::Value& ref);
    };

} } }