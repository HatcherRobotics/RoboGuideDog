#pragma once

#include <rpos/core/rpos_core_config.h>
#include "thread_pool_worker.h"
#include "../thread_priority.h"
#include <vector>

namespace rpos { namespace system { namespace thread_pool {

    class RPOS_CORE_API ThreadPool : public boost::enable_shared_from_this<ThreadPool>, private boost::noncopyable {
    public:
        ThreadPool(ThreadPriority priority = ThreadPriorityNormal);
        ~ThreadPool();

    public:
        int getMaxThreads() const;
        void setMaxThreads(int v);

        int getAliveThreads() const;
        int getWorkingThreads() const;

        int getId() const;

    public:
        void pushTask(boost::function<void()> task);
        void pushTask(const std::string& name, boost::function<void()> task);
        void recycle();
        void clearTaskQueue();

        void dispose();

		void stopAliveThreads();

    private:
        friend class ThreadPoolWorker;

        void giveBirthToWorkers_();

        int id_;

        ThreadPriority priority_;
        boost::shared_ptr<TaskQueue> taskQueue_;
        std::vector<boost::shared_ptr<ThreadPoolWorker>> workers_;
        int nextWorkerId_;
        int maxThreads_;
        mutable boost::mutex lock_;
        bool disposed_;
    };

} } }
