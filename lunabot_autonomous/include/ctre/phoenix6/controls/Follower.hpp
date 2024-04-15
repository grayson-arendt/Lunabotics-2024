/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
#pragma once

#include "ctre/phoenix6/configs/Configs.hpp"
#include "ctre/phoenix6/controls/ControlRequest.hpp"
#include "ctre/phoenix6/networking/interfaces/Control_Interface.h"
#include <sstream>

#include <units/frequency.h>
#include <units/time.h>


namespace ctre {
namespace phoenix6 {
namespace controls {

/**
 * Follow the motor output of another Talon.
 * 
 * If Talon is in torque control, the torque is copied - which will increase the total torque applied. If
 * Talon is in percent supply output control, the duty cycle is matched.  Motor direction either matches
 * master's configured direction or opposes it based on OpposeMasterDirection.
 */
class Follower : public ControlRequest
{
    ctre::phoenix::StatusCode SendRequest(const char *network, uint32_t deviceHash, bool cancelOtherRequests, std::shared_ptr<ControlRequest> &req) override
    {
        if (req.get() != this)
        {
            auto const reqCast = dynamic_cast<Follower *>(req.get());
            if (reqCast != nullptr)
            {
                *reqCast = *this;
            }
            else
            {
                req = std::make_shared<Follower>(*this);
            }
        }

        return c_ctre_phoenix6_RequestControlFollower(network, deviceHash, UpdateFreqHz.to<double>(), cancelOtherRequests, MasterID, OpposeMasterDirection);
    }

public:
    /**
     * Device ID of the master to follow.
     */
    int MasterID;
    /**
     * Set to false for motor invert to match the master's configured Invert - which
     * is typical when master and follower are mechanically linked and spin in the
     * same direction.  Set to true for motor invert to oppose the master's
     * configured Invert - this is typical where the the master and follower
     * mechanically spin in opposite directions.
     */
    bool OpposeMasterDirection;

    /**
     * \brief The period at which this control will update at.
     * This is designated in Hertz, with a minimum of 20 Hz
     * (every 50 ms) and a maximum of 1000 Hz (every 1 ms).
     *
     * If this field is set to 0 Hz, the control request will
     * be sent immediately as a one-shot frame. This may be useful
     * for advanced applications that require outputs to be
     * synchronized with data acquisition. In this case, we
     * recommend not exceeding 50 ms between control calls.
     */
    units::frequency::hertz_t UpdateFreqHz{20_Hz}; // Default to 20_Hz

    /**
     * \brief Follow the motor output of another Talon.
     * 
     * \details If Talon is in torque control, the torque is copied - which will
     *          increase the total torque applied. If Talon is in percent supply
     *          output control, the duty cycle is matched.  Motor direction either
     *          matches master's configured direction or opposes it based on
     *          OpposeMasterDirection.
     * 
     * \param MasterID    Device ID of the master to follow.
     * \param OpposeMasterDirection    Set to false for motor invert to match the
     *                                 master's configured Invert - which is typical
     *                                 when master and follower are mechanically
     *                                 linked and spin in the same direction.  Set
     *                                 to true for motor invert to oppose the
     *                                 master's configured Invert - this is typical
     *                                 where the the master and follower
     *                                 mechanically spin in opposite directions.
     */
    Follower(int MasterID, bool OpposeMasterDirection) : ControlRequest{"Follower"},
        MasterID{std::move(MasterID)},
        OpposeMasterDirection{std::move(OpposeMasterDirection)}
    {}
    
    /**
     * \brief Modifies this Control Request's MasterID parameter and returns itself for
     *        method-chaining and easier to use request API.
     * \param newMasterID Parameter to modify
     * \returns Itself
     */
    Follower& WithMasterID(int newMasterID)
    {
        MasterID = std::move(newMasterID);
        return *this;
    }
    
    /**
     * \brief Modifies this Control Request's OpposeMasterDirection parameter and returns itself for
     *        method-chaining and easier to use request API.
     * \param newOpposeMasterDirection Parameter to modify
     * \returns Itself
     */
    Follower& WithOpposeMasterDirection(bool newOpposeMasterDirection)
    {
        OpposeMasterDirection = std::move(newOpposeMasterDirection);
        return *this;
    }
    /**
     * \brief Sets the period at which this control will update at.
     * This is designated in Hertz, with a minimum of 20 Hz
     * (every 50 ms) and a maximum of 1000 Hz (every 1 ms).
     *
     * If this field is set to 0 Hz, the control request will
     * be sent immediately as a one-shot frame. This may be useful
     * for advanced applications that require outputs to be
     * synchronized with data acquisition. In this case, we
     * recommend not exceeding 50 ms between control calls.
     *
     * \param newUpdateFreqHz Parameter to modify
     * \returns Itself
     */
    Follower &WithUpdateFreqHz(units::frequency::hertz_t newUpdateFreqHz)
    {
        UpdateFreqHz = newUpdateFreqHz;
        return *this;
    }
    /**
     * Returns a string representation of the object.
     *
     * \returns a string representation of the object.
     */
    std::string ToString() const override
    {
        std::stringstream ss;
        ss << "class: Follower" << std::endl;
        ss << "MasterID: " << MasterID << std::endl;
        ss << "OpposeMasterDirection: " << OpposeMasterDirection << std::endl;
        return ss.str();
    }

    /**
     * \brief Gets information about this control request.
     *
     * \returns Map of control parameter names and corresponding applied values
     */
    std::map<std::string, std::string> GetControlInfo() const override
    {
        std::map<std::string, std::string> controlInfo;
        std::stringstream ss;
        controlInfo["Name"] = GetName();
        ss << MasterID; controlInfo["MasterID"] = ss.str(); ss.str(std::string{});
        ss << OpposeMasterDirection; controlInfo["OpposeMasterDirection"] = ss.str(); ss.str(std::string{});
        return controlInfo;
    }
};

}
}
}

