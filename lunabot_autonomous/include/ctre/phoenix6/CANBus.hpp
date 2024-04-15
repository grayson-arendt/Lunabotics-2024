/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
#pragma once

#include "ctre/phoenix/StatusCodes.h"
#include <string>

namespace ctre {
namespace phoenix6 {

/**
 * \brief Static class for getting information about available CAN buses.
 */
class CANBus {
public:
    /**
     * \brief Contains status information about a CAN bus.
     */
    struct CANBusStatus {
        /**
         * \brief Status code response of getting the data
         */
        ctre::phoenix::StatusCode Status;

        /**
         * \brief CAN bus utilization, from 0.0 to 1.0
         */
        float BusUtilization;
        /**
         * \brief Bus off count
         */
        uint32_t BusOffCount;
        /**
         * \brief Transmit buffer full count
         */
        uint32_t TxFullCount;
        /**
         * \brief Receive Error Counter (REC)
         */
        uint32_t REC;
        /**
         * \brief Transmit Error Counter (TEC)
         */
        uint32_t TEC;
    };

    /**
     * \brief Gets whether the CAN bus is a CAN FD network.
     * 
     * \param canbus Name of the CAN bus
     * \returns True if the CAN bus is CAN FD
     */
    static bool IsNetworkFD(std::string const &canbus);
    /**
     * \brief Gets the status of the CAN bus, including the
     * bus utilization and the error counters.
     *
     * \param canbus Name of the CAN bus
     * \returns Status of the CAN bus
     */
    static CANBusStatus GetStatus(std::string const &canbus);
};

}
}
