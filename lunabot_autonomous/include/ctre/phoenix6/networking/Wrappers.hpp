/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
#pragma once

#include "ctre/phoenix/StatusCodes.h"
#include "ctre/phoenix6/networking/interfaces/DeviceEncoding_Interface.h"
#include <string>

namespace ctre {
namespace phoenix6 {
namespace networking {

    class Wrappers
    {
    public:
        /**
         * \brief Creates the device hash from provided parameters
         *
         * \param deviceID Device ID
         * \param model Device Model
         * \param canbus CAN bus name
         * \returns Hash of the device
         */
        static uint32_t CompileDeviceHash(int deviceID, const char *model, const char *canbus);
        /**
         * \brief Sets the config value of the device
         *
         * \param network What network the device is on
         * \param deviceHash The hash of the device
         * \param timeoutSeconds How long to wait for the device to acknowledge in seconds
         * \param serializedString Serialized configs to set
         * \param futureProofConfigs True to factory default the configs before setting
         * \param overrideIfDuplicate True to always configure this parameter, even if we detect it's unnecessary
         * \returns Status of the config set
         */
        static ctre::phoenix::StatusCode Device_SetConfigValues(
            const char *network,
            int deviceHash,
            double timeoutSeconds,
            const std::string &serializedString,
            bool futureProofConfigs,
            bool overrideIfDuplicate);

        /**
         * \brief Gets the config value of the device
         *
         * \param network What network the device is on
         * \param deviceHash The hash of the device
         * \param timeoutSeconds How long to wait for the device to acknowledge in seconds
         * \param serializedString Serialized configs to get
         * \returns Status of the config set
         */
        static ctre::phoenix::StatusCode Device_GetConfigValues(
            const char *network,
            int deviceHash,
            double timeoutSeconds,
            std::string &serializedString);
    };

}
}
}
