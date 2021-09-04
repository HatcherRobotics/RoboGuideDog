
#pragma once

#include "server_params.h"
#include "server_work_data.h"

#include <tf/transform_broadcaster.h>

#include <rpos/robot_platforms/slamware_core_platform.h>

#include <boost/chrono.hpp>

namespace slamware_ros_sdk {

    class SlamwareRosSdkServer;

    class ServerWorkerBase
    {
    public:
        typedef boost::chrono::steady_clock         clock_t;
        typedef clock_t::time_point                 time_point_t;

        typedef rpos::robot_platforms::SlamwareCorePlatform     slamware_platform_t;

    public:
        ServerWorkerBase(SlamwareRosSdkServer* pRosSdkServer
            , const std::string& wkName
            , const boost::chrono::milliseconds& triggerInterval
            );
        virtual ~ServerWorkerBase();

        const std::string& getWorkerName() const { return workerName_; }

        time_point_t getNextTimepointToTrigger() const { return nextTimepointToTrigger_; }
        
        bool isWorkLoopInitOk() const { return isWorkLoopInitOk_; }
        virtual void resetOnWorkLoopBegin();
        virtual bool reinitWorkLoop(slamware_platform_t& pltfm);

        virtual void checkToPerform(slamware_platform_t& pltfm);

    protected:
        virtual void doPerform(slamware_platform_t& pltfm) = 0;

    protected:
        SlamwareRosSdkServer* rosSdkServer() const { return rosSdkServer_; }

        ros::NodeHandle& rosNodeHandle() const;
        const ServerParams& serverParams() const;
        tf::TransformBroadcaster& tfBroadcaster() const;

        ServerWorkData_ConstPtr workData() const;
        ServerWorkData_Ptr mutableWorkData();

    private:
        SlamwareRosSdkServer* rosSdkServer_;
        std::string workerName_;

    protected:
        bool isWorkLoopInitOk_;
        boost::chrono::milliseconds triggerInterval_;
        time_point_t nextTimepointToTrigger_;
    };

    typedef boost::shared_ptr<ServerWorkerBase>         ServerWorkerBase_Ptr;
    
}
