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
#include "ctre/phoenix6/controls/PositionTorqueCurrentFOC.hpp"
#include "ctre/phoenix6/controls/VelocityTorqueCurrentFOC.hpp"
#include <units/frequency.h>
#include <units/time.h>


namespace ctre {
namespace phoenix6 {
namespace controls {
namespace compound {

/**
 * \private
 * Requires Phoenix Pro and CANivore;
 * Differential control with position average target and velocity difference
 * target using torque current control.
 */
class Diff_PositionTorqueCurrentFOC_Velocity : public ControlRequest
{
    ctre::phoenix::StatusCode SendRequest(const char *network, uint32_t deviceHash, bool cancelOtherRequests, std::shared_ptr<ControlRequest> &req) override
    {
        if (req.get() != this)
        {
            auto const reqCast = dynamic_cast<Diff_PositionTorqueCurrentFOC_Velocity *>(req.get());
            if (reqCast != nullptr)
            {
                *reqCast = *this;
            }
            else
            {
                req = std::make_shared<Diff_PositionTorqueCurrentFOC_Velocity>(*this);
            }
        }

        return c_ctre_phoenix6_RequestControlDiff_PositionTorqueCurrentFOC_Velocity(network, deviceHash, UpdateFreqHz.to<double>(), cancelOtherRequests, AverageRequest.Position.to<double>(), AverageRequest.Velocity.to<double>(), AverageRequest.FeedForward.to<double>(), AverageRequest.Slot, AverageRequest.OverrideCoastDurNeutral, AverageRequest.LimitForwardMotion, AverageRequest.LimitReverseMotion, DifferentialRequest.Velocity.to<double>(), DifferentialRequest.Acceleration.to<double>(), DifferentialRequest.FeedForward.to<double>(), DifferentialRequest.Slot, DifferentialRequest.OverrideCoastDurNeutral, DifferentialRequest.LimitForwardMotion, DifferentialRequest.LimitReverseMotion);
    }

public:
    /**
     * Average PositionTorqueCurrentFOC request of the mechanism.
     */
    PositionTorqueCurrentFOC AverageRequest;
    /**
     * Differential VelocityTorqueCurrentFOC request of the mechanism.
     */
    VelocityTorqueCurrentFOC DifferentialRequest;

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
    units::frequency::hertz_t UpdateFreqHz{100_Hz}; // Default to 100_Hz

    /**
     * \brief Requires Phoenix Pro and CANivore;
     *        Differential control with position
     *        average target and velocity difference target using torque current
     *        control.
     * 
     * \param AverageRequest    Average PositionTorqueCurrentFOC request of the
     *                          mechanism.
     * \param DifferentialRequest    Differential VelocityTorqueCurrentFOC request
     *                               of the mechanism.
     */
    Diff_PositionTorqueCurrentFOC_Velocity(PositionTorqueCurrentFOC AverageRequest, VelocityTorqueCurrentFOC DifferentialRequest) : ControlRequest{"Diff_PositionTorqueCurrentFOC_Velocity"},
        AverageRequest{std::move(AverageRequest)},
        DifferentialRequest{std::move(DifferentialRequest)}
    {}
    
    /**
     * \brief Modifies this Control Request's AverageRequest parameter and returns itself for
     *        method-chaining and easier to use request API.
     * \param newAverageRequest Parameter to modify
     * \returns Itself
     */
    Diff_PositionTorqueCurrentFOC_Velocity& WithAverageRequest(PositionTorqueCurrentFOC newAverageRequest)
    {
        AverageRequest = std::move(newAverageRequest);
        return *this;
    }
    
    /**
     * \brief Modifies this Control Request's DifferentialRequest parameter and returns itself for
     *        method-chaining and easier to use request API.
     * \param newDifferentialRequest Parameter to modify
     * \returns Itself
     */
    Diff_PositionTorqueCurrentFOC_Velocity& WithDifferentialRequest(VelocityTorqueCurrentFOC newDifferentialRequest)
    {
        DifferentialRequest = std::move(newDifferentialRequest);
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
    Diff_PositionTorqueCurrentFOC_Velocity &WithUpdateFreqHz(units::frequency::hertz_t newUpdateFreqHz)
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
        ss << "class: Diff_PositionTorqueCurrentFOC_Velocity" << std::endl;
        ss << "AverageRequest:" << std::endl;
        ss << "    Position: " << AverageRequest.Position.to<double>() << std::endl;
        ss << "    Velocity: " << AverageRequest.Velocity.to<double>() << std::endl;
        ss << "    FeedForward: " << AverageRequest.FeedForward.to<double>() << std::endl;
        ss << "    Slot: " << AverageRequest.Slot << std::endl;
        ss << "    OverrideCoastDurNeutral: " << AverageRequest.OverrideCoastDurNeutral << std::endl;
        ss << "    LimitForwardMotion: " << AverageRequest.LimitForwardMotion << std::endl;
        ss << "    LimitReverseMotion: " << AverageRequest.LimitReverseMotion << std::endl;
        ss << "DifferentialRequest:" << std::endl;
        ss << "    Velocity: " << DifferentialRequest.Velocity.to<double>() << std::endl;
        ss << "    Acceleration: " << DifferentialRequest.Acceleration.to<double>() << std::endl;
        ss << "    FeedForward: " << DifferentialRequest.FeedForward.to<double>() << std::endl;
        ss << "    Slot: " << DifferentialRequest.Slot << std::endl;
        ss << "    OverrideCoastDurNeutral: " << DifferentialRequest.OverrideCoastDurNeutral << std::endl;
        ss << "    LimitForwardMotion: " << DifferentialRequest.LimitForwardMotion << std::endl;
        ss << "    LimitReverseMotion: " << DifferentialRequest.LimitReverseMotion << std::endl;
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
        ss << AverageRequest.ToString(); controlInfo["AverageRequest"] = ss.str(); ss.str(std::string{});
        ss << DifferentialRequest.ToString(); controlInfo["DifferentialRequest"] = ss.str(); ss.str(std::string{});
        return controlInfo;
    }
};

}
}
}
}

