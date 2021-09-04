
#include "server_worker_base.h"
#include "slamware_ros_sdk_server.h"

#include <boost/assert.hpp>

namespace slamware_ros_sdk {

    ServerWorkerBase::ServerWorkerBase(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const boost::chrono::milliseconds& triggerInterval
        )
        : rosSdkServer_(pRosSdkServer)
        , workerName_(wkName)
        , isWorkLoopInitOk_(false)
        , triggerInterval_(triggerInterval)
    {
        BOOST_ASSERT(nullptr != rosSdkServer_);
    }

    ServerWorkerBase::~ServerWorkerBase()
    {
        //
    }

    void ServerWorkerBase::resetOnWorkLoopBegin()
    {
        isWorkLoopInitOk_ = false;
    }

    bool ServerWorkerBase::reinitWorkLoop(slamware_platform_t& /*pltfm*/)
    {
        nextTimepointToTrigger_ = time_point_t();

        isWorkLoopInitOk_ = true;
        return isWorkLoopInitOk_;
    }

    void ServerWorkerBase::checkToPerform(slamware_platform_t& pltfm)
    {
        const auto tpNow = clock_t::now();
        if (nextTimepointToTrigger_ <= tpNow)
        {
            nextTimepointToTrigger_ = tpNow + triggerInterval_;

            doPerform(pltfm);
        }
    }
    
    ros::NodeHandle& ServerWorkerBase::rosNodeHandle() const
    {
        return rosSdkServer_->rosNodeHandle_();
    }

    const ServerParams& ServerWorkerBase::serverParams() const
    {
        return rosSdkServer_->serverParams_();
    }

    tf::TransformBroadcaster& ServerWorkerBase::tfBroadcaster() const
    {
        return rosSdkServer_->tfBroadcaster_();
    }

    ServerWorkData_ConstPtr ServerWorkerBase::workData() const
    {
        return rosSdkServer_->safeGetWorkData_();
    }
    
    ServerWorkData_Ptr ServerWorkerBase::mutableWorkData()
    {
        return rosSdkServer_->safeGetMutableWorkData_();
    }

}
