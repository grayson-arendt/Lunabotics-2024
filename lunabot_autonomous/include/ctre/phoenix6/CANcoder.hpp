/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
#pragma once

#include "ctre/phoenix6/core/CoreCANcoder.hpp"

namespace ctre {
namespace phoenix6 {
namespace hardware {

/**
 * Class for CANcoder, a CAN based magnetic encoder that provides absolute and
 * relative position along with filtered velocity.
 */
class CANcoder : public core::CoreCANcoder
{
public:
    /**
     * Constructs a new CANcoder object.
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
    CANcoder(int deviceId, std::string canbus = "") : CoreCANcoder{deviceId, std::move(canbus)}
    {
    }
};

}
}
}