/*
* power_management.h
* Power management features
*
* Created by Tony Huang (tony@slamtec.com) at 2017-2-25
* Copyright 2017 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include <rpos/core/rpos_core_config.h>

namespace rpos { namespace features { namespace system_resource {

    /**
    * Sleep mode of device
    */
    enum SleepMode {
        /**
        * The targeting core firmware doens't support this command
        */
        SleepModeUnknown,

        /**
        * The device is awake, it will response to commands immediately
        */
        SleepModeAwake,

        /**
        * The device is waking up, please wait for some time
        */
        SleepModeWakingUp,

        /**
        * The device is asleep, it might take some time to wake device up
        */
        SleepModeAsleep
    };

    /**
    * Docking status of the robot
    */
    enum DockingStatus {
        /**
        * The docking status is unknown (doens't supported by the targeting Slamware firmware)
        */
        DockingStatusUnknown,

        /**
        * The robot is on the dock
        */
        DockingStatusOnDock,

        /**
        * The robot is not on the dock
        */
        DockingStatusNotOnDock
    };

    /**
    * Power stage of robot
    */
    enum PowerStage {
        /**
        * The robot power stage unknown.
        */
        PowerStageUnknown,
        /**
        * The robot is starting.
        */
        PowerStageStarting,
        /**
        * The robot is running.
        */
        PowerStageRunning,
        /**
        * The robot is restarting.
        */
        PowerStageRestarting,
        /**
        * The robot is shuting down.
        */
        PowerStageShutingDown,
        /**
        * The robot power stage error.
        */
        PowerStageError
    };

    /**
    * Composed power status of the device
    */
    struct RPOS_CORE_API PowerStatus {
        /**
        * The dc cord is connected
        */
        bool isDCConnected;

        /**
        * The robot is on dock
        */
        DockingStatus dockingStatus;

        /**
        * The robot is charging
        */
        bool isCharging;

        /**
        * Battery level (0 ~ 100)
        */
        int batteryPercentage;

        /**
        * Robot sleep mode
        */
        SleepMode sleepMode;

        /**
        * Robot power stage
        */
        PowerStage powerStage;

        PowerStatus();
        ~PowerStatus();
    };

} } }
