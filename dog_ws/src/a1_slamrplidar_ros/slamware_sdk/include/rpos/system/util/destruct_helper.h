#pragma once

#include <boost/function.hpp>
#include <boost/noncopyable.hpp>

namespace rpos { namespace system { namespace util {

    class DestructHelper : boost::noncopyable
    {
    public:
        typedef boost::function< void() >           destruct_fun_t;

    public:
        DestructHelper()
        {
            //
        }

        explicit DestructHelper(const destruct_fun_t& destructFun)
            : destructFun_(destructFun)
        {
            //
        }

        ~DestructHelper()
        {
            if (destructFun_)
                destructFun_();
        }

        void resetDestructFun()
        {
            destructFun_ = destruct_fun_t();
        }

        void setDestructFun(const destruct_fun_t& destructFun)
        {
            destructFun_ = destructFun;
        }

    private:
        destruct_fun_t destructFun_;
    };

}}}
