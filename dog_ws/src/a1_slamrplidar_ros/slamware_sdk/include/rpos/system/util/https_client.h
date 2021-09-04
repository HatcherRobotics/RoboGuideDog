/**
* https_client.h
*
* Created By Gabriel He @ 2016-02-02
* Copyright (c) 2016 Shanghai SlamTec Co., Ltd.
*/

#pragma once

#include <rpos/core/rpos_core_config.h>
#include <rpos/system/types.h>
#include <rpos/system/util/string_utils.h>
#include <rpos/system/util/log.h>

#include <boost/thread/condition_variable.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/make_shared.hpp>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/unordered_map.hpp>

#include <curl/curl.h>

#ifdef WIN32
#pragma comment(lib, "Ws2_32.lib")
#pragma comment(lib, "Wldap32.lib")
#endif

#include <map>
#include <string>
#include <vector>

#define RPOS_LIB_NAME rpos_deps_libcurl
#define RPOS_AUTO_LINK_NO_VERSION
#	include <rpos/system/util/auto_link.h>
#undef RPOS_AUTO_LINK_NO_VERSION
#undef RPOS_LIB_NAME

#define SLEEP_MILLISECOND_100 100

namespace rpos { namespace system { namespace util {

    class RPOS_CORE_API HttpsInit : public boost::noncopyable
    {
    public:
        static void init();
        static void destroy();

    private:
        HttpsInit();
        ~HttpsInit();

        static HttpsInit* init_;
        static boost::mutex lock_;
    };

    struct RPOS_CORE_API HttpsClientConfig
    {
        HttpsClientConfig(bool enable_verify_ssl, std::string cafile_path);

        bool enable_verify_ssl;
        std::string cafile_path;
    };

    template <class HttpsClientHandlerT>
    class HttpsClient : public boost::enable_shared_from_this<HttpsClient<HttpsClientHandlerT>>, private boost::noncopyable
    {
    public:
        typedef boost::shared_ptr<HttpsClient<HttpsClientHandlerT>> Pointer;

    public:
        HttpsClient(const HttpsClientConfig& config)
            : done_(false)
            , logger_("rpos.system.util.HttpsClient")
            , config_(config)
        {
            HttpsInit::init();
        }

        virtual ~HttpsClient()
        {
            stop();
        }

        void start()
        {
            curl_m_ = curl_multi_init();

            if (selectThread_.joinable())
            {
                return;
            }

            selectThread_ = boost::move(boost::thread(boost::bind(&HttpsClient::worker_, this->shared_from_this())));
        }

        void stop()
        {
            {
                boost::lock_guard<boost::mutex> guard(lock_);

                if(done_)
                {
                    return;
                }
                else
                {
                    done_ = true;
                }
            }

            if (selectThread_.joinable())
            {
                selectThread_.join();
            }

            curl_multi_cleanup(curl_m_);
        }

        void send(unsigned int requestId, std::string& method, std::string& uri, std::vector<std::string>& headers, std::vector<system::types::_u8>& body)
        {
            auto context = new CurlContext();

            context->received_size = 0;
            context->request_id = requestId;

            context->request.method_ = method;
            context->request.uri_ = uri;

            struct curl_slist* curl_header = NULL;
            for(auto it = headers.begin(); it != headers.end(); it++)
            {
                std::string header = (*it);
                curl_header = curl_slist_append(curl_header, header.c_str());
            }
            curl_header = curl_slist_append(curl_header, "Expect:");
            context->request.headers_ = curl_header;


            context->request.body_ = body;

            context->response.status_.clear();
            context->response.headers_.clear();
            context->response.body_.clear();

            context->pointer = this->shared_from_this();
            context->handler_ = new HttpsClientHandlerT();

            CURL* curl = curl_easy_handler_get(context, 500);
            {
                boost::lock_guard<boost::mutex> guard(contexts_lock_);
                contexts_[curl] = context;
            }
            curl_multi_add_handle(curl_m_, curl);
        }

    private:
        class Request
        {
        public:
            Request()
            {}

            ~Request()
            {
                curl_slist_free_all(headers_);
            }

            std::string method_;
            std::string uri_;
            struct curl_slist* headers_;
            std::vector<system::types::_u8> body_;
        };

        class Response
        {
        public:
            Response()
            {}

            std::string status_;
            std::vector<std::string> headers_;
            std::vector<system::types::_u8> body_;
        };

        class CurlContext
        {
        public:
            CurlContext()
            {}

            ~CurlContext()
            {
                delete handler_;
            }

            unsigned int received_size;
            unsigned int request_id;
            std::string header_buffer;

            Request request;
            Response response;

            Pointer pointer;
            HttpsClientHandlerT* handler_;
        };

    private:
        void worker_()
        {
            CURLMsg *msg;
            int msgs_left;

            int running_handles = 0;

            while(true)
            {
                logger_.debug_out("sleep: %d ms", SLEEP_MILLISECOND_100);
                boost::this_thread::sleep(boost::posix_time::milliseconds(SLEEP_MILLISECOND_100));
                {
                    logger_.debug_out("guard(lock_)");
                    boost::lock_guard<boost::mutex> guard(lock_);
                    logger_.debug_out("lock");
                    if (done_)
                    {
                        logger_.debug_out("done, unlock");
                        break;
                    }
                    logger_.debug_out("unlock");
                }

                logger_.debug_out("while(CURLM_CALL_MULTI_PERFORM == curl_multi_perform(curl_m_, &running_handles))");
                while(CURLM_CALL_MULTI_PERFORM == curl_multi_perform(curl_m_, &running_handles))
                {
                    logger_.debug_out("sleep: %d ms", SLEEP_MILLISECOND_100);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(SLEEP_MILLISECOND_100));
                }

                CURLMcode rs = my_curl_multi_wait_(curl_m_);
                logger_.debug_out("curl_multi_wait: %s(%d)", curl_multi_strerror(rs), rs);

                logger_.debug_out("while((msg = curl_multi_info_read(curl_m_, &msgs_left)))");
                while((msg = curl_multi_info_read(curl_m_, &msgs_left)))
                {
                    logger_.debug_out("CURLMSG_DONE == msg->msg");
                    if(CURLMSG_DONE == msg->msg)
                    {
                        logger_.debug_out("curl_multi_remove_handle(curl_m_, msg->easy_handle);");
                        curl_multi_remove_handle(curl_m_, msg->easy_handle);
                        {
                            logger_.debug_out("guard(contexts_lock_)");
                            boost::lock_guard<boost::mutex> guard(contexts_lock_);
                            logger_.debug_out("lock");
                            CurlContext* context = contexts_[msg->easy_handle];
                            if (context != nullptr)
                            {
                                auto response = context->response;
                                logger_.debug_out("receiveComplete");
                                context->handler_->receiveComplete(context->pointer, context->request_id, response.status_, response.headers_, response.body_);
                                logger_.debug_out("contexts_.erase(msg->easy_handle);");
                                contexts_.erase(msg->easy_handle);
                                logger_.debug_out("delete context;");
                                delete context;
                                logger_.debug_out("unlock");
                            }
                            else
                            {
                                logger_.debug_out("contexts_.erase nullptr");
                                contexts_.erase(msg->easy_handle);
                            }
                        }
                        logger_.debug_out("curl_easy_cleanup");
                        curl_easy_cleanup(msg->easy_handle);
                    }
                }
            }
        }

        CURLMcode my_curl_multi_wait_(CURLM* curl_m)
        {
            logger_.debug_out("my_curl_multi_wait_");
            CURLMcode ret = CURLM_OK;

            int curl_timeout_ms = 1000;
            logger_.debug_out("curl_multi_wait");
            ret = curl_multi_wait(curl_m, nullptr, 0, curl_timeout_ms, nullptr);

            return ret;
        }

        static size_t curl_writer(void *buffer, size_t size, size_t count, void *stream)
        {
            std::string *pStream = static_cast<std::string *>(stream);
            pStream->append(static_cast<char *>(buffer), size * count);
            return size * count;
        }

        static size_t curl_head_writer(void *buffer, size_t size, size_t count, void *pUser)
        {
            int rs;

            auto context = (CurlContext*)pUser;


            std::string str;

            rs = curl_writer(buffer, size, count, &str);

            context->header_buffer.insert(context->header_buffer.end(), str.begin(), str.end());

            std::string sEnd = "\r\n";

            auto last = context->header_buffer.find(sEnd+sEnd);
            if (last == (context->header_buffer.length()-4))
            {
                unsigned int p;

                p = context->header_buffer.find(sEnd);

                context->response.status_ = context->header_buffer.substr(0, p);
                context->header_buffer.erase(0, p+2);

                while((p=context->header_buffer.find(sEnd)) > 0)
                {
                    auto str = context->header_buffer.substr(0, p);

                    context->response.headers_.push_back(str);

                    context->header_buffer.erase(0, p+2);
                }
                context->header_buffer.clear();
            }
            return rs;
        }

        static size_t curl_body_writer(void *buffer, size_t size, size_t count, void *pUser)
        {
            int rs;
            auto context = (CurlContext*)pUser;

            std::string str;
            rs = curl_writer(buffer, size, count, &str);
            if (rs > 0)
            {
                context->received_size += rs;
                context->response.body_.insert(context->response.body_.end(), str.begin(), str.end());
            }

            return rs;
        }

        CURL* curl_easy_handler_get(
            void *pUser,
            unsigned int uiTimeout)
        {
            auto context = (CurlContext*)pUser;

            CURL *curl = curl_easy_init();
#ifdef _DEBUG
            curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);
#endif
            if (config_.enable_verify_ssl)
            {
                curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 2L);
                curl_easy_setopt(curl, CURLOPT_CAINFO, config_.cafile_path.c_str());
            }
            else
            {
                curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);

            }
            curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
            curl_easy_setopt(curl, CURLOPT_MAXREDIRS, 5); // max redirection times

            auto& method     = context->request.method_;
            auto& url        = context->request.uri_;
            auto headers    = context->request.headers_;
            auto& body       = context->request.body_;

            // set method
            curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, method.c_str());

            //set url
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1);

            //set header
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

            //set body
            if (body.size() > 0)
            {
                auto body_size = body.size();
                curl_easy_setopt(curl, CURLOPT_POST, 1L);
                curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, body_size);
                curl_easy_setopt(curl, CURLOPT_COPYPOSTFIELDS, &body[0]);
            }

            if(uiTimeout > 0)
            {
                curl_easy_setopt(curl, CURLOPT_TIMEOUT, uiTimeout);
            }

            curl_easy_setopt(curl, CURLOPT_HEADERFUNCTION, curl_head_writer);
            curl_easy_setopt(curl, CURLOPT_HEADERDATA, pUser);

            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_body_writer);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, pUser);

            return curl;
        }

    protected:
        CURLM *curl_m_;

    private:
        boost::thread selectThread_;
        boost::mutex lock_;
        boost::mutex contexts_lock_;
        bool done_;
        std::map<void*, CurlContext*> contexts_;

        rpos::system::util::LogScope logger_;
        HttpsClientConfig config_;
    };
}}}
