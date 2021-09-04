#pragma once

#include <string>

namespace rpos { namespace robot_platforms { namespace detail { namespace objects {

    struct UpdateInfo
    {
        std::string currentVersion;
        std::string newVersion;
        std::string newVersionReleaseDate;
        std::string newVersionChangeLog;
    };

    enum FirmwareInfoCode
    {
        INIT=0,
        SUCCESSFUL,
        CONNECT_ERROR,
        HTTP_ERROR,
        SYSTEM_ERROR,
        NO_AVAILABLE_FIRMWARE
    };

    enum UpdateFirmwareStep
    {
        UpdateFirmwarePreparing,
        UpdateFirmwarePrepareFinished,
        UpdateFirmwareDownloading,
        UpdateFirmwareDownloadFinished,
        UpdateFirmwareUpdating,
        UpdateFirmwareUpdateFinished
    };

    enum UpdateProgressStatus
    {
        UpdateProgressSuccess,
        UpdateProgressError,
        UpdateProgressInit,
        UpdateProgressUpgrade
    };

    struct UpdateProgress
    {
        UpdateFirmwareStep currentStep;     // Range from 0 to {totalSteps - 1}.
        unsigned int totalSteps;            // The number of total steps.
        std::string currentStepName;        // The name of the current step.
        unsigned int currentStepProgress;   // Expressed as a percentage.
        UpdateProgressStatus status;        // Current status
    };

}}}}
