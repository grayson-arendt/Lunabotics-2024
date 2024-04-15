/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
#pragma once

#include "ctre/phoenix6/core/CorePigeon2.hpp"

namespace ctre {
namespace phoenix6 {
namespace hardware {

/**
 * Class description for the Pigeon 2 IMU sensor that measures orientation.
 */
class Pigeon2 : public core::CorePigeon2
{
public:
    /**
     * Constructs a new Pigeon 2 sensor object.
     *
     * \param deviceId    ID of the device, as configured in Phoenix Tuner.
     * \param canbus      Name of the CAN bus this device is on. Possible CAN bus strings are:
     *                    - "rio" for the native roboRIO CAN bus
     *                    - CANivore name or serial number
     *                    - SocketCAN interface (non-FRC Linux only)
     *                    - "*" for any CANivore seen by the program
     *                    - empty string (default) to select the default for the system:
     *                      - "rio" on roboRIO
     *                      - "can0" on Linux
     *                      - "*" on Windows
     */
    Pigeon2(int deviceId, std::string canbus = "") : CorePigeon2{deviceId, std::move(canbus)}
    {
    }
};

}
}
}