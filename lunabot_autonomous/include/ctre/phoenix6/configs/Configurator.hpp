/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
#pragma once

#include "ctre/phoenix/StatusCodes.h"
#include "ctre/phoenix/platform/Platform.hpp"
#include "ctre/phoenix6/hardware/DeviceIdentifier.hpp"
#include "ctre/phoenix6/networking/interfaces/ReportError_Interface.h"
#include <units/time.h>
#include <mutex>

namespace ctre {
namespace phoenix6 {
namespace configs {

    class ParentConfigurator
    {
    public:
        /**
         * \brief The default amount of time to wait for a config.
         */
        units::time::second_t DefaultTimeoutSeconds{0.050_s};

    private:
        hardware::DeviceIdentifier deviceIdentifier;
        mutable std::mutex _m;

    protected:
        ParentConfigurator(hardware::DeviceIdentifier deviceIdentifier) : deviceIdentifier{std::move(deviceIdentifier)}
        {
        }

        ctre::phoenix::StatusCode SetConfigsPrivate(const std::string &serializedString, units::time::second_t timeoutSeconds, bool futureProofConfigs, bool overrideIfDuplicate)
        {
            ctre::phoenix::StatusCode status;
            {
                std::lock_guard<std::mutex> lock{_m};

                status = networking::Wrappers::Device_SetConfigValues(
                    deviceIdentifier.network.c_str(),
                    deviceIdentifier.deviceHash,
                    timeoutSeconds.to<double>(),
                    serializedString,
                    futureProofConfigs,
                    overrideIfDuplicate);
            }

            if (!status.IsOK())
            {
                std::stringstream location;
                location << this->deviceIdentifier.ToString() << " Apply Config";
                c_ctre_phoenix_report_error(status.IsError(), status, 0, status.GetDescription(), location.str().c_str(), ctre::phoenix::platform::GetStackTrace(1).c_str());
            }
            return status;
        }
        ctre::phoenix::StatusCode GetConfigsPrivate(std::string &serializedString, units::time::second_t timeoutSeconds) const
        {
            ctre::phoenix::StatusCode status;
            {
                std::lock_guard<std::mutex> lock{_m};

                status = networking::Wrappers::Device_GetConfigValues(
                    deviceIdentifier.network.c_str(),
                    deviceIdentifier.deviceHash,
                    timeoutSeconds.to<double>(),
                    serializedString);
            }

            if (!status.IsOK())
            {
                std::stringstream location;
                location << this->deviceIdentifier.ToString() << " Refresh Config";
                c_ctre_phoenix_report_error(status.IsError(), status, 0, status.GetDescription(), location.str().c_str(), ctre::phoenix::platform::GetStackTrace(1).c_str());
            }
            return status;
        }
    };

}
}
}
