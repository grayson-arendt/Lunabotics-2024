/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
#pragma once

#include "ctre/phoenix6/networking/Wrappers.hpp"
#include <cstdint>
#include <sstream>
#include <string>

namespace ctre {
namespace phoenix6 {
namespace hardware {

    class DeviceIdentifier
    {
    public:
        std::string network;
        std::string model;
        int deviceID;
        uint32_t deviceHash;

        DeviceIdentifier() = default;
        DeviceIdentifier(int deviceID, std::string model, std::string canbus) : network{std::move(canbus)},
                                                                                model{std::move(model)},
                                                                                deviceID{deviceID},
                                                                                deviceHash{networking::Wrappers::CompileDeviceHash(deviceID, this->model.c_str(), this->network.c_str())}
        {
        }

        std::string ToString() const
        {
            std::stringstream ss;
            ss << model << " " << deviceID << " (" << network << ")";
            return ss.str();
        }
    };

}
}
}
