/**
 kint.zhao  huasheng_zyh@163.com
   2017.0721

  modified by yun.li@slamtec.com, 2019.
*/

#include  "slamware_ros_sdk_server.h"

#include <rpos/system/io/memory_read_stream.h>
#include <rpos/system/io/memory_write_stream.h>
#include <rpos/robot_platforms/objects/composite_map_reader.h>
#include <rpos/robot_platforms/objects/composite_map_writer.h>

#include <boost/assert.hpp>

#include <stdexcept>
#include <cmath>

namespace slamware_ros_sdk {

    //////////////////////////////////////////////////////////////////////////

    SlamwareRosSdkServer::SlamwareRosSdkServer()
        : state_(ServerStateNotInit)
        , isStopRequested_(false)
        , nh_("~")
    {
        //
    }

    SlamwareRosSdkServer::~SlamwareRosSdkServer()
    {
        cleanup_();
    }

    bool SlamwareRosSdkServer::startRun(std::string& errMsg)
    {
        errMsg.clear();

        const auto oldState = state_.load();
        if (ServerStateNotInit != oldState && ServerStateStopped != oldState)
        {
            errMsg = "it is running or already initialized.";
            return false;
        }

        isStopRequested_.store(false);
        bool bRet = init_(errMsg);
        if (bRet)
        {
            state_.store(ServerStateRunning);
            workThread_ = boost::move(boost::thread(boost::bind(&SlamwareRosSdkServer::workThreadFun_, this)));
        }

        if (!bRet)
        {
            cleanup_();
        }
        return bRet;
    }

    void SlamwareRosSdkServer::requestStop()
    {
        isStopRequested_.store(true);
    }

    void SlamwareRosSdkServer::waitUntilStopped()
    {
        if (workThread_.joinable())
            workThread_.join();
        BOOST_ASSERT(!isRunning_());
    }

    void SlamwareRosSdkServer::requestSyncMap()
    {
        auto wkDat = safeGetMutableWorkData_();
        if (wkDat)
            wkDat->syncMapRequested.store(true);
    }

    boost::chrono::milliseconds SlamwareRosSdkServer::sfConvFloatSecToBoostMs_(float fSec)
    {
        if (fSec < 0.0f)
            throw std::runtime_error("invalid float value of seconds.");

        const std::uint32_t uMs = static_cast<std::uint32_t>(std::floor(fSec * 1000));
        return boost::chrono::milliseconds(uMs);
    }

    bool SlamwareRosSdkServer::shouldContinueRunning_() const
    {
        return (!isStopRequested_.load());
    }

    ServerWorkData_ConstPtr SlamwareRosSdkServer::safeGetWorkData_() const
    {
        boost::lock_guard<boost::mutex> lkGuard(workDatLock_);
        return workDat_;
    }

    ServerWorkData_Ptr SlamwareRosSdkServer::safeGetMutableWorkData_()
    {
        boost::lock_guard<boost::mutex> lkGuard(workDatLock_);
        return workDat_;
    }

    bool SlamwareRosSdkServer::safeIsSlamwarePlatformConnected_() const
    {
        boost::lock_guard<boost::mutex> lkGuard(slamwarePltfmLock_);
        return bool(slamwarePltfm_);
    }

    SlamwareRosSdkServer::slamware_platform_t SlamwareRosSdkServer::safeGetSlamwarePlatform_() const
    {
        boost::lock_guard<boost::mutex> lkGuard(slamwarePltfmLock_);
        return slamwarePltfm_;
    }

    void SlamwareRosSdkServer::safeSetSlamwarePlatform_(const slamware_platform_t& pltfm)
    {
        auto tmpPltfm = pltfm;
        {
            boost::lock_guard<boost::mutex> lkGuard(slamwarePltfmLock_);
            std::swap(slamwarePltfm_, tmpPltfm);
        }
    }

    void SlamwareRosSdkServer::safeReleaseSlamwarePlatform_()
    {
        slamware_platform_t tmpPltfm;
        {
            boost::lock_guard<boost::mutex> lkGuard(slamwarePltfmLock_);
            std::swap(slamwarePltfm_, tmpPltfm);
        }
        disconnectSlamwarePlatform_(tmpPltfm);
    }
    
    SlamwareRosSdkServer::slamware_platform_t SlamwareRosSdkServer::connectSlamwarePlatform_(const std::string& ip, int port) const
    {
        try
        {
            auto pltfm = slamware_platform_t::connect(ip, port);
            return pltfm;
        }
        catch (const std::exception& excp)
        {
            ROS_ERROR("connectSlamwarePlatform_(), exception: %s.", excp.what());
        }
        catch (...)
        {
            ROS_ERROR("connectSlamwarePlatform_(), unknown exception.");
        }
        return slamware_platform_t();
    }
    
    void SlamwareRosSdkServer::disconnectSlamwarePlatform_(slamware_platform_t& pltfm) const
    {
        if (pltfm)
        {
            try
            {
                pltfm.disconnect();
            }
            catch (const std::exception& excp)
            {
                ROS_ERROR("disconnectSlamwarePlatform_(), exception: %s.", excp.what());
            }
            catch (...)
            {
                ROS_ERROR("disconnectSlamwarePlatform_(), unknown exception.");
            }
        }
    }

    bool SlamwareRosSdkServer::init_(std::string& /*errMsg*/)
    {
        params_.resetToDefault();
        params_.setBy(nh_);

        {
            boost::lock_guard<boost::mutex> lkGuard(workDatLock_);
            workDat_ = boost::make_shared<ServerWorkData>();
        }

        // init all workers
        {
            serverWorkers_.clear();

            const auto defaultUpdateIntervalForNoneUpdateWorkers = boost::chrono::milliseconds(1000u * 60u);

            {
                auto svrWk = boost::make_shared<ServerRobotDeviceInfoWorker>(this, "RobotDeviceInfo", defaultUpdateIntervalForNoneUpdateWorkers);
                serverWorkers_.push_back(svrWk);
            }

            if (0 < params_.robot_pose_pub_period)
            {
                auto svrWk = boost::make_shared<ServerRobotPoseWorker>(this, "RobotPose", sfConvFloatSecToBoostMs_(params_.robot_pose_pub_period));
                serverWorkers_.push_back(svrWk);
            }

            if (0 < params_.map_update_period)
            {
                auto svrWk = boost::make_shared<ServerExploreMapUpdateWorker>(this, "ExploreMapUpdate", sfConvFloatSecToBoostMs_(params_.map_update_period));
                serverWorkers_.push_back(svrWk);
            }

            if (0 < params_.map_pub_period)
            {
                auto svrWk = boost::make_shared<ServerExploreMapPublishWorker>(this, "ExploreMapPublish", sfConvFloatSecToBoostMs_(params_.map_pub_period));
                serverWorkers_.push_back(svrWk);
            }

            if (0 < params_.scan_pub_period)
            {
                auto svrWk = boost::make_shared<ServerLaserScanWorker>(this, "LaserScan", sfConvFloatSecToBoostMs_(params_.scan_pub_period));
                serverWorkers_.push_back(svrWk);
            }

            if (0 < params_.basic_sensors_info_update_period)
            {
                auto svrWk = boost::make_shared<ServerBasicSensorsInfoWorker>(this, "BasicSensorsInfo", sfConvFloatSecToBoostMs_(params_.basic_sensors_info_update_period));
                serverWorkers_.push_back(svrWk);
            }

            if (0 < params_.basic_sensors_values_pub_period)
            {
                auto svrWk = boost::make_shared<ServerBasicSensorsValuesWorker>(this, "BasicSensorsValues", sfConvFloatSecToBoostMs_(params_.basic_sensors_values_pub_period));
                serverWorkers_.push_back(svrWk);
            }

            if (0 < params_.path_pub_period)
            {
                auto svrWk = boost::make_shared<ServerPlanPathWorker>(this, "PlanPath", sfConvFloatSecToBoostMs_(params_.path_pub_period));
                serverWorkers_.push_back(svrWk);
            }

            if (0 < params_.robot_basic_state_pub_period)
            {
                auto svrWk = boost::make_shared<ServerRobotBasicStateWorker>(this, "RobotBasicState", sfConvFloatSecToBoostMs_(params_.robot_basic_state_pub_period));
                serverWorkers_.push_back(svrWk);
            }

            if (0 < params_.virtual_walls_pub_period)
            {
                ServerArtifactLinesWorker::params_t tParams;
                tParams.usage.usage = ArtifactUsage::VIRTUAL_WALL;
                tParams.topic = "virtual_walls";
                auto svrWk = boost::make_shared<ServerArtifactLinesWorker>(this, "VirtualWalls", sfConvFloatSecToBoostMs_(params_.virtual_walls_pub_period), tParams);
                serverWorkers_.push_back(svrWk);
            }

            if (0 < params_.virtual_tracks_pub_period)
            {
                ServerArtifactLinesWorker::params_t tParams;
                tParams.usage.usage = ArtifactUsage::VIRTUAL_TRACK;
                tParams.topic = "virtual_tracks";
                auto svrWk = boost::make_shared<ServerArtifactLinesWorker>(this, "VirtualTracks", sfConvFloatSecToBoostMs_(params_.virtual_tracks_pub_period), tParams);
                serverWorkers_.push_back(svrWk);
            }
        }

        // init all subscriptions
        {
//            subRobotControl_ = subscribe_T_<geometry_msgs::Twist>(params_.vel_control_topic, 10U, &SlamwareRosSdkServer::msgCbRobotControl_);
//            subMoveToGoal_ = subscribe_T_<geometry_msgs::PoseStamped>(params_.goal_topic, 1U, &SlamwareRosSdkServer::msgCbMoveToGoal_);
            
            subSyncMap_ = subscribe_T_<SyncMapRequest>("sync_map", 1U, &SlamwareRosSdkServer::msgCbSyncMap_);
            subSetPose_ = subscribe_T_<geometry_msgs::Pose>("set_pose", 1U, &SlamwareRosSdkServer::msgCbSetPose_);

            subRecoverLocalization_ = subscribe_T_<RecoverLocalizationRequest>("recover_localization", 1U, &SlamwareRosSdkServer::msgCbRecoverLocalization_);
            subClearMap_ = subscribe_T_<ClearMapRequest>("clear_map", 1U, &SlamwareRosSdkServer::msgCbClearMap_);
            subSetMapUpdate_ = subscribe_T_<SetMapUpdateRequest>("set_map_update", 1U, &SlamwareRosSdkServer::msgCbSetMapUpdate_);
            subSetMapLocalization_ = subscribe_T_<SetMapLocalizationRequest>("set_map_localization", 1U, &SlamwareRosSdkServer::msgCbSetMapLocalization_);

            subMoveByDirection_ = subscribe_T_<MoveByDirectionRequest>("move_by_direction", 1U, &SlamwareRosSdkServer::msgCbMoveByDirection_);
            subMoveByTheta_ = subscribe_T_<MoveByThetaRequest>("move_by_theta", 1U, &SlamwareRosSdkServer::msgCbMoveByTheta_);
            subMoveTo_ = subscribe_T_<MoveToRequest>("move_to", 1U, &SlamwareRosSdkServer::msgCbMoveTo_);
            subMoveToLocations_ = subscribe_T_<MoveToLocationsRequest>("move_to_locations", 1U, &SlamwareRosSdkServer::msgCbMoveToLocations_);
            subRotateTo_ = subscribe_T_<RotateToRequest>("rotate_to", 1U, &SlamwareRosSdkServer::msgCbRotateTo_);
            subRotate_ = subscribe_T_<RotateRequest>("rotate", 1U, &SlamwareRosSdkServer::msgCbRotate_);

            subGoHome_ = subscribe_T_<GoHomeRequest>("go_home", 1U, &SlamwareRosSdkServer::msgCbGoHome_);
            subCancelAction_ = subscribe_T_<CancelActionRequest>("cancel_action", 1U, &SlamwareRosSdkServer::msgCbCancelAction_);

            subAddLine_ = subscribe_T_<AddLineRequest>("add_line", 1U, &SlamwareRosSdkServer::msgCbAddLine_);
            subAddLines_ = subscribe_T_<AddLinesRequest>("add_lines", 1U, &SlamwareRosSdkServer::msgCbAddLines_);
            subRemoveLine_ = subscribe_T_<RemoveLineRequest>("remove_line", 1U, &SlamwareRosSdkServer::msgCbRemoveLine_);
            subClearLines_ = subscribe_T_<ClearLinesRequest>("clear_lines", 1U, &SlamwareRosSdkServer::msgCbClearLines_);
            subMoveLine_ = subscribe_T_<MoveLineRequest>("move_line", 1U, &SlamwareRosSdkServer::msgCbMoveLine_);
            subMoveLines_ = subscribe_T_<MoveLinesRequest>("move_lines", 1U, &SlamwareRosSdkServer::msgCbMoveLines_);
        }

        // init all services
        {
            srvSyncGetStcm_ = advertiseService_T_<SyncGetStcm>("sync_get_stcm", &SlamwareRosSdkServer::srvCbSyncGetStcm_);
            srvSyncSetStcm_ = advertiseService_T_<SyncSetStcm>("sync_set_stcm", &SlamwareRosSdkServer::srvCbSyncSetStcm_);
        }

        return true;
    }

    void SlamwareRosSdkServer::cleanup_()
    {
        if (isRunning_())
            requestStop();
        waitUntilStopped();

        safeReleaseSlamwarePlatform_();

        // de-init all services
        {
            srvSyncGetStcm_ = ros::ServiceServer();
            srvSyncSetStcm_ = ros::ServiceServer();
        }

        // de-init all subscriptions
        {
            subRobotControl_ = ros::Subscriber();
            subMoveToGoal_ = ros::Subscriber();
            
            subSyncMap_ = ros::Subscriber();
            subSetPose_ = ros::Subscriber();

            subRecoverLocalization_ = ros::Subscriber();
            subClearMap_ = ros::Subscriber();
            subSetMapUpdate_ = ros::Subscriber();
            subSetMapLocalization_ = ros::Subscriber();

            subMoveByDirection_ = ros::Subscriber();
            subMoveByTheta_ = ros::Subscriber();
            subMoveTo_ = ros::Subscriber();
            subMoveToLocations_ = ros::Subscriber();
            subRotateTo_ = ros::Subscriber();
            subRotate_ = ros::Subscriber();

            subGoHome_ = ros::Subscriber();
            subCancelAction_ = ros::Subscriber();

            subAddLine_ = ros::Subscriber();
            subAddLines_ = ros::Subscriber();
            subRemoveLine_ = ros::Subscriber();
            subClearLines_ = ros::Subscriber();
            subMoveLine_ = ros::Subscriber();
            subMoveLines_ = ros::Subscriber();
        }

        // de-init all publishers
        {
            serverWorkers_.clear();
        }

        {
            boost::lock_guard<boost::mutex> lkGuard(workDatLock_);
            workDat_.reset();
        }

        state_.store(ServerStateNotInit);
    }

    void SlamwareRosSdkServer::workThreadFun_()
    {
        BOOST_ASSERT(ServerStateRunning == state_.load());
        ROS_INFO("SlamwareRosSdkServer, work thread begin.");

        while (shouldContinueRunning_()
            && ros::ok()
            )
        {
            if (!safeIsSlamwarePlatformConnected_())
            {
                loopTryConnectToSlamwarePlatform_();

                if (!safeIsSlamwarePlatformConnected_())
                    continue;
            }
            BOOST_ASSERT(safeIsSlamwarePlatformConnected_());

            try
            {
                loopWork_();
            }
            catch (const std::exception& excp)
            {
                ROS_FATAL("loopWork_(), exception: %s.", excp.what());
            }
            catch (...)
            {
                ROS_FATAL("loopWork_(), unknown exception.");
            }

            safeReleaseSlamwarePlatform_();
            if (shouldContinueRunning_())
            {
                const std::uint32_t maxSleepMs = (1000u * 3u);
                ROS_INFO("wait %u ms to reconnect and restart work loop.", maxSleepMs);
                roughSleepWait_(maxSleepMs, 100U);
            }
        }

        ROS_INFO("SlamwareRosSdkServer, work thread end.");
        state_.store(ServerStateStopped);
    }

    void SlamwareRosSdkServer::roughSleepWait_(std::uint32_t maxSleepMs, std::uint32_t onceSleepMs)
    {
        const auto durOnceSleep = boost::chrono::milliseconds(onceSleepMs);
        auto tpNow = boost::chrono::steady_clock::now();
        const auto maxSleepTimepoint = tpNow + boost::chrono::milliseconds(maxSleepMs);
        while (shouldContinueRunning_()
            && tpNow < maxSleepTimepoint
            )
        {
            boost::this_thread::sleep_for(durOnceSleep);
            tpNow = boost::chrono::steady_clock::now();
        }
    }

    void SlamwareRosSdkServer::loopTryConnectToSlamwarePlatform_()
    {
        std::uint32_t tryCnt = 1;
        while (shouldContinueRunning_())
        {
            {
                ROS_INFO("try to connect to %s:%d, tryCnt: %u.", params_.ip_address.c_str(), params_.robot_port, tryCnt);
                auto pltfm = connectSlamwarePlatform_(params_.ip_address, params_.robot_port);
                if (pltfm)
                {
                    ROS_INFO("connect to %s:%d, OK, tryCnt: %u.", params_.ip_address.c_str(), params_.robot_port, tryCnt);
                    safeSetSlamwarePlatform_(pltfm);
                    return;
                }
                ROS_ERROR("connect to %s:%d, FAILED, tryCnt: %u, wait %d ms to retry."
                    , params_.ip_address.c_str(), params_.robot_port
                    , tryCnt, params_.reconn_wait_ms);
            }

            const std::uint32_t maxSleepMs = (0 <= params_.reconn_wait_ms ? (std::uint32_t)params_.reconn_wait_ms : 0U);
            roughSleepWait_(maxSleepMs, 100U);
            ++tryCnt;
        }
    }

    bool SlamwareRosSdkServer::reinitWorkLoop_(slamware_platform_t& pltfm)
    {
        const std::uint32_t cntWorkers = static_cast<std::uint32_t>(serverWorkers_.size());

        ROS_INFO("reset all %u workers on work loop begin.", cntWorkers);
        for (auto it = serverWorkers_.begin(), itEnd = serverWorkers_.end(); itEnd != it; ++it)
        {
            const auto& svrWk = (*it);
            const auto wkName = svrWk->getWorkerName();
            try
            {
                svrWk->resetOnWorkLoopBegin();
            }
            catch (const std::exception& excp)
            {
                ROS_ERROR("worker: %s, resetOnWorkLoopBegin(), exception: %s.", wkName.c_str(), excp.what());
                return false;
            }
        }

        const std::uint32_t maxSleepMs = (1000u * 2u);
        const std::uint32_t maxLoopTryCnt = 3;
        for (std::uint32_t t = 1; (t <= maxLoopTryCnt && shouldContinueRunning_()); ++t)
        {
            std::uint32_t cntOk = 0;
            for (auto it = serverWorkers_.begin(), itEnd = serverWorkers_.end(); itEnd != it; ++it)
            {
                const auto& svrWk = (*it);
                const auto wkName = svrWk->getWorkerName();
                try
                {
                    if (svrWk->isWorkLoopInitOk())
                    {
                        ++cntOk;
                    }
                    else
                    {
                        if (svrWk->reinitWorkLoop(pltfm))
                            ++cntOk;
                        else
                            ROS_WARN("failed to init work loop, woker: %s.", wkName.c_str());
                    }
                }
                catch (const rpos::robot_platforms::ConnectionLostException& excp)
                {
                    ROS_ERROR("worker: %s, reinitWorkLoop(), exception: %s.", wkName.c_str(), excp.what());
                    throw;
                }
                catch (const rpos::robot_platforms::ConnectionTimeOutException& excp)
                {
                    ROS_ERROR("worker: %s, reinitWorkLoop(), exception: %s.", wkName.c_str(), excp.what());
                    throw;
                }
                catch (const rpos::robot_platforms::ConnectionFailException& excp)
                {
                    ROS_ERROR("worker: %s, reinitWorkLoop(), exception: %s.", wkName.c_str(), excp.what());
                    throw;
                }
                catch (const std::exception& excp)
                {
                    ROS_ERROR("worker: %s, reinitWorkLoop(), exception: %s.", wkName.c_str(), excp.what());
                }
            }
            // check if all workers are ok.            
            if (cntWorkers == cntOk)
            {
                return true;
            }
            else if (t < maxLoopTryCnt)
            {
                ROS_WARN("(%u / %u) cntWorkers: %u, cntOk: %u, wait %u ms to retry.", t, maxLoopTryCnt, cntWorkers, cntOk, maxSleepMs);
                roughSleepWait_(maxSleepMs, 100U);
            }
            else
            {
                ROS_WARN("(%u / %u) cntWorkers: %u, cntOk: %u.", t, maxLoopTryCnt, cntWorkers, cntOk);
            }
        }
        return false;
    }

    void SlamwareRosSdkServer::loopWork_()
    {
        auto pltfm = safeGetSlamwarePlatform_();
        BOOST_ASSERT(pltfm);

        if (reinitWorkLoop_(pltfm))
        {
            ROS_INFO("successed to reinit all workers on work loop begin.");
        }
        else
        {
            ROS_ERROR("failed or cancelled to reinit work loop.");
            return;
        }

        while (shouldContinueRunning_())
        {
            boost::chrono::steady_clock::time_point minNextTriggerTimepoint = boost::chrono::steady_clock::now() + boost::chrono::milliseconds(100U);

            for (auto it = serverWorkers_.begin(), itEnd = serverWorkers_.end(); itEnd != it; ++it)
            {
                const auto& svrWk = (*it);
                const auto wkName = svrWk->getWorkerName();
                bool shouldReconnect = false;
                try
                {
                    svrWk->checkToPerform(pltfm);
                }
                catch (const rpos::robot_platforms::ConnectionLostException& excp)
                {
                    shouldReconnect = true;
                    ROS_ERROR("worker name: %s, exception: %s.", wkName.c_str(), excp.what());
                }
                catch (const rpos::robot_platforms::ConnectionTimeOutException& excp)
                {
                    shouldReconnect = true;
                    ROS_ERROR("worker name: %s, exception: %s.", wkName.c_str(), excp.what());
                }
                catch (const rpos::robot_platforms::ConnectionFailException& excp)
                {
                    shouldReconnect = true;
                    ROS_ERROR("worker name: %s, exception: %s.", wkName.c_str(), excp.what());
                }
                catch (const rpos::robot_platforms::OperationFailException& excp)
                {
                    ROS_WARN("worker name: %s, exception: %s.", wkName.c_str(), excp.what());
                }
                catch (const rpos::system::detail::ExceptionBase& excp)
                {
                    ROS_ERROR("worker name: %s, exception: %s.", wkName.c_str(), excp.what());
                }
                catch (const std::exception& excp)
                {
                    ROS_ERROR("worker name: %s, exception: %s.", wkName.c_str(), excp.what());
                }
                catch (...)
                {
                    ROS_ERROR("worker name: %s, unknown exception.", wkName.c_str());
                }

                if (shouldReconnect)
                {
                    ROS_ERROR("it should reconnect to slamware.");
                    return;
                }

                const auto tmpNextTp = svrWk->getNextTimepointToTrigger();
                if (tmpNextTp < minNextTriggerTimepoint)
                    minNextTriggerTimepoint = tmpNextTp;
            }

            auto tpNow = boost::chrono::steady_clock::now();
            if (tpNow <= minNextTriggerTimepoint)
            {
                const auto durSleep = boost::chrono::duration_cast<boost::chrono::milliseconds>(minNextTriggerTimepoint - tpNow);
                boost::this_thread::sleep_for(durSleep);
            }
        }
    }

    //////////////////////////////////////////////////////////////////////////

    template<class MsgT>
    void SlamwareRosSdkServer::msgCbWrapperFun_T_(typename msg_cb_help_t<MsgT>::msg_cb_perform_fun_t mfpCbPerform
        , const std::string& msgTopic
        , const typename msg_cb_help_t<MsgT>::const_msg_shared_ptr & msg
        )
    {
        BOOST_ASSERT(nullptr != mfpCbPerform);
        auto pltfm = safeGetSlamwarePlatform_();
        if (!pltfm)
        {
            ROS_ERROR("process msgTopic: %s, not connected.", msgTopic.c_str());
            return;
        }

        try
        {
            (this->*mfpCbPerform)(pltfm, msg);
        }
        catch (const std::exception& excp)
        {
            ROS_ERROR("process msgTopic: %s, exception: %s.", msgTopic.c_str(), excp.what());
        }
        catch (...)
        {
            ROS_ERROR("process msgTopic: %s, unknown exception.", msgTopic.c_str());
        }
    }

    template<class MsgT>
    ros::Subscriber SlamwareRosSdkServer::subscribe_T_(const std::string& msgTopic
        , std::uint32_t queueSize
        , typename msg_cb_help_t<MsgT>::msg_cb_perform_fun_t mfpCbPerform
        )
    {
        typedef msg_cb_help_t<MsgT>     TheMsgCbHelpT;

        typename TheMsgCbHelpT::ros_cb_fun_t rosCbFun(
            boost::bind(&SlamwareRosSdkServer::msgCbWrapperFun_T_<MsgT>, this, mfpCbPerform, msgTopic, _1)
            );
        return nh_.subscribe(msgTopic, queueSize, rosCbFun);
    }

    void SlamwareRosSdkServer::msgCbRobotControl_(slamware_platform_t& pltfm, const geometry_msgs::Twist::ConstPtr& msg)
    {
        if(velocityControllAction_.isEmpty())
        try
        {
            velocityControllAction_ = slamwarePltfm_.velocityControl();
        }
        catch(...)
        {
            ROS_ERROR("Can not construct velocity controll action");
            return;
        }

        try
        {
            velocityControllAction_.setVelocity(msg->linear.x, msg->linear.y, msg->angular.z);
        }
        catch(...)
        {
            ROS_ERROR("Reconstruct velocity controll action");
            velocityControllAction_ = slamwarePltfm_.velocityControl();
            velocityControllAction_.setVelocity(msg->linear.x, msg->linear.y, msg->angular.z);
            throw;
        }
    }

    void SlamwareRosSdkServer::msgCbMoveToGoal_(slamware_platform_t& pltfm, const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        const rpos::core::Location tLoc(msg->pose.position.x, msg->pose.position.y);
        const float fYaw = tf::getYaw(msg->pose.orientation);
        
        rpos::features::motion_planner::MoveOptions optMove;
        optMove.flag = rpos::features::motion_planner::MoveOptionFlag(rpos::features::motion_planner::MoveOptionFlagMilestone
            | rpos::features::motion_planner::MoveOptionFlagPrecise
            | rpos::features::motion_planner::MoveOptionFlagWithYaw
            );

        pltfm.moveTo(tLoc, optMove, fYaw);
    }

    void SlamwareRosSdkServer::msgCbSyncMap_(slamware_platform_t& /*pltfm*/, const SyncMapRequest::ConstPtr& /*msg*/)
    {
        requestSyncMap();
    }

    void SlamwareRosSdkServer::msgCbSetPose_(slamware_platform_t& pltfm, const geometry_msgs::Pose::ConstPtr& msg)
    {
        rpos::core::Pose robotPose;
        rosMsgToSltc(*msg, robotPose);

        pltfm.setPose(robotPose);
    }

    void SlamwareRosSdkServer::msgCbRecoverLocalization_(slamware_platform_t& pltfm, const RecoverLocalizationRequest::ConstPtr& msg)
    {
        rpos::core::RectangleF area;
        rosMsgToSltc(msg->area, area);

        rpos::features::motion_planner::RecoverLocalizationOptions options;
        rosMsgToSltc(msg->options, options);

        pltfm.recoverLocalization(area, options);
    }

    void SlamwareRosSdkServer::msgCbClearMap_(slamware_platform_t& pltfm, const ClearMapRequest::ConstPtr& msg)
    {
        rpos::features::location_provider::MapKind kind;
        rosMsgToSltc(msg->kind, kind);

        pltfm.clearMap(kind);
    }

    void SlamwareRosSdkServer::msgCbSetMapUpdate_(slamware_platform_t& pltfm, const SetMapUpdateRequest::ConstPtr& msg)
    {
        rpos::features::location_provider::MapKind kind;
        rosMsgToSltc(msg->kind, kind);

        pltfm.setMapUpdate(msg->enabled, kind);
    }

    void SlamwareRosSdkServer::msgCbSetMapLocalization_(slamware_platform_t& pltfm, const SetMapLocalizationRequest::ConstPtr& msg)
    {
        pltfm.setMapLocalization(msg->enabled);
    }

    void SlamwareRosSdkServer::msgCbMoveByDirection_(slamware_platform_t& pltfm, const MoveByDirectionRequest::ConstPtr& msg)
    {
        rpos::core::ACTION_DIRECTION eDir;
        rosMsgToSltc(msg->direction, eDir);
        const auto tDir = rpos::core::Direction(eDir);

        rpos::features::motion_planner::MoveOptions tOpts;
        rosMsgToSltc(msg->options, tOpts);

        pltfm.moveBy(tDir, tOpts);
    }

    void SlamwareRosSdkServer::msgCbMoveByTheta_(slamware_platform_t& pltfm, const MoveByThetaRequest::ConstPtr& msg)
    {
        rpos::features::motion_planner::MoveOptions tOpts;
        rosMsgToSltc(msg->options, tOpts);

        pltfm.moveBy(msg->theta, tOpts);
    }

    void SlamwareRosSdkServer::msgCbMoveTo_(slamware_platform_t& pltfm, const MoveToRequest::ConstPtr& msg)
    {
        rpos::core::Location tLoc;
        rosMsgToSltc(msg->location, tLoc);

        rpos::features::motion_planner::MoveOptions tOpts;
        rosMsgToSltc(msg->options, tOpts);

        pltfm.moveTo(tLoc, tOpts, msg->yaw);
    }

    void SlamwareRosSdkServer::msgCbMoveToLocations_(slamware_platform_t& pltfm, const MoveToLocationsRequest::ConstPtr& msg)
    {
        std::vector<rpos::core::Location> vLocs;
        rosMsgToSltc(msg->locations, vLocs);

        rpos::features::motion_planner::MoveOptions tOpts;
        rosMsgToSltc(msg->options, tOpts);

        pltfm.moveTo(vLocs, tOpts, msg->yaw);
    }

    void SlamwareRosSdkServer::msgCbRotateTo_(slamware_platform_t& pltfm, const RotateToRequest::ConstPtr& msg)
    {
        rpos::core::Rotation orientation;
        rosMsgToSltc(msg->orientation, orientation);

        rpos::features::motion_planner::MoveOptions tOpts;
        rosMsgToSltc(msg->options, tOpts);

        pltfm.rotateTo(orientation, tOpts);
    }

    void SlamwareRosSdkServer::msgCbRotate_(slamware_platform_t& pltfm, const RotateRequest::ConstPtr& msg)
    {
        rpos::core::Rotation rotation;
        rosMsgToSltc(msg->rotation, rotation);

        rpos::features::motion_planner::MoveOptions tOpts;
        rosMsgToSltc(msg->options, tOpts);

        pltfm.rotate(rotation, tOpts);
    }

    void SlamwareRosSdkServer::msgCbGoHome_(slamware_platform_t& pltfm, const GoHomeRequest::ConstPtr& /*msg*/)
    {
        pltfm.goHome();
    }

    void SlamwareRosSdkServer::msgCbCancelAction_(slamware_platform_t& pltfm, const CancelActionRequest::ConstPtr& /*msg*/)
    {
        auto tAct = pltfm.getCurrentAction();
        if (tAct)
            tAct.cancel();
    }

    void SlamwareRosSdkServer::msgCbAddLine_(slamware_platform_t& pltfm, const AddLineRequest::ConstPtr& msg)
    {
        rpos::features::artifact_provider::ArtifactUsage tUsage;
        rosMsgToSltc(msg->usage, tUsage);

        rpos::core::Line tLine;
        rosMsgToSltc(msg->line, tLine);

        pltfm.addLine(tUsage, tLine);
    }

    void SlamwareRosSdkServer::msgCbAddLines_(slamware_platform_t& pltfm, const AddLinesRequest::ConstPtr& msg)
    {
        rpos::features::artifact_provider::ArtifactUsage tUsage;
        rosMsgToSltc(msg->usage, tUsage);

        std::vector<rpos::core::Line> vLines;
        rosMsgToSltc(msg->lines, vLines);

        pltfm.addLines(tUsage, vLines);
    }

    void SlamwareRosSdkServer::msgCbRemoveLine_(slamware_platform_t& pltfm, const RemoveLineRequest::ConstPtr& msg)
    {
        rpos::features::artifact_provider::ArtifactUsage tUsage;
        rosMsgToSltc(msg->usage, tUsage);

        pltfm.removeLineById(tUsage, msg->id);
    }

    void SlamwareRosSdkServer::msgCbClearLines_(slamware_platform_t& pltfm, const ClearLinesRequest::ConstPtr& msg)
    {
        rpos::features::artifact_provider::ArtifactUsage tUsage;
        rosMsgToSltc(msg->usage, tUsage);

        pltfm.clearLines(tUsage);
    }

    void SlamwareRosSdkServer::msgCbMoveLine_(slamware_platform_t& pltfm, const MoveLineRequest::ConstPtr& msg)
    {
        rpos::features::artifact_provider::ArtifactUsage tUsage;
        rosMsgToSltc(msg->usage, tUsage);

        rpos::core::Line tLine;
        rosMsgToSltc(msg->line, tLine);

        pltfm.moveLine(tUsage, tLine);
    }

    void SlamwareRosSdkServer::msgCbMoveLines_(slamware_platform_t& pltfm, const MoveLinesRequest::ConstPtr& msg)
    {
        rpos::features::artifact_provider::ArtifactUsage tUsage;
        rosMsgToSltc(msg->usage, tUsage);

        std::vector<rpos::core::Line> vLines;
        rosMsgToSltc(msg->lines, vLines);

        pltfm.moveLines(tUsage, vLines);
    }

    //////////////////////////////////////////////////////////////////////////

    template<class SrvMsgT>
    bool SlamwareRosSdkServer::srvCbWrapperFun_T_(typename srv_cb_help_t<SrvMsgT>::srv_cb_perform_fun_t mfpCbPerform
        , const std::string& srvMsgTopic
        , typename srv_cb_help_t<SrvMsgT>::request_t& req
        , typename srv_cb_help_t<SrvMsgT>::response_t& resp
        )
    {
        BOOST_ASSERT(nullptr != mfpCbPerform);
        auto pltfm = safeGetSlamwarePlatform_();
        if (!pltfm)
        {
            ROS_ERROR("process request: %s, not connected.", srvMsgTopic.c_str());
            return false;
        }

        try
        {
            return (this->*mfpCbPerform)(pltfm, req, resp);
        }
        catch (const std::exception& excp)
        {
            ROS_ERROR("process request: %s, exception: %s.", srvMsgTopic.c_str(), excp.what());
        }
        catch (...)
        {
            ROS_ERROR("process request: %s, unknown exception.", srvMsgTopic.c_str());
        }
        return false;
    }

    template<class SrvMsgT>
    ros::ServiceServer SlamwareRosSdkServer::advertiseService_T_(const std::string& srvMsgTopic
        , typename srv_cb_help_t<SrvMsgT>::srv_cb_perform_fun_t mfpCbPerform
        )
    {
        typedef srv_cb_help_t<SrvMsgT>     TheSrvCbHelpT;

        typename TheSrvCbHelpT::ros_cb_fun_t rosCbFun(
            boost::bind(&SlamwareRosSdkServer::srvCbWrapperFun_T_<SrvMsgT>, this, mfpCbPerform, srvMsgTopic, _1, _2)
            );
        return nh_.advertiseService(srvMsgTopic, rosCbFun);
    }

    bool SlamwareRosSdkServer::srvCbSyncGetStcm_(slamware_platform_t& pltfm, SyncGetStcm::Request& req, SyncGetStcm::Response& resp)
    {
        std::string strTmp;
        rpos::system::io::MemoryWriteStream tMemWS(1024 * 1024 * 8);
        try
        {
            const auto cmpstMap = pltfm.getCompositeMap();

            rpos::robot_platforms::objects::CompositeMapWriter tWriter;
            if (!tWriter.saveStream(strTmp, tMemWS, cmpstMap))
            {
                ROS_ERROR("srvCbSyncGetStcm_(), saveStream(), %s.", strTmp.c_str());
                return false;
            }
        }
        catch (const std::exception& excp)
        {
            ROS_WARN("srvCbSyncGetStcm_(), exception: %s.", excp.what());
            return false;
        }

        resp.raw_stcm.resize(tMemWS.size());
        std::memcpy(resp.raw_stcm.data(), tMemWS.buffer(), tMemWS.size());
        return true;
    }

    bool SlamwareRosSdkServer::srvCbSyncSetStcm_(slamware_platform_t& pltfm, SyncSetStcm::Request& req, SyncSetStcm::Response& resp)
    {
        rpos::core::Pose robotPose;
        rosMsgToSltc(req.robot_pose, robotPose);

        std::string strTmp;
        boost::shared_ptr<rpos::robot_platforms::objects::CompositeMap> spSmpstMap;
        {
            rpos::system::io::MemoryReadStream tMemRS((const void*)req.raw_stcm.data()
                , req.raw_stcm.size()
                , rpos::system::io::MemoryReadStream::MemoryReadStreamFlagBorrowBuffer
                );
            rpos::robot_platforms::objects::CompositeMapReader tReader;
            spSmpstMap = tReader.loadStream(strTmp, tMemRS);
            if (!spSmpstMap)
            {
                ROS_ERROR("srvCbSyncSetStcm_(), loadStream(), %s.", strTmp.c_str());
                return false;
            }
        }
        BOOST_ASSERT(spSmpstMap);

        try
        {
            pltfm.clearMap();
            auto tAct = pltfm.getCurrentAction();
            if (tAct)
                tAct.cancel();
            pltfm.setMapLocalization(false);
            pltfm.setMapUpdate(false);

            pltfm.setCompositeMap(*spSmpstMap, robotPose);

            pltfm.setMapLocalization(true);
        }
        catch (const std::exception& excp)
        {
            ROS_WARN("srvCbSyncSetStcm_(), exception: %s.", excp.what());
            return false;
        }

        requestSyncMap();
        return true;
    }

    //////////////////////////////////////////////////////////////////////////

}
