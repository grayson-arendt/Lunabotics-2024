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
#include <units/angle.h>
#include <units/current.h>
#include <units/frequency.h>
#include <units/time.h>


namespace ctre {
namespace phoenix6 {
namespace controls {

/**
 * Requires Phoenix Pro;
 * Requests Motion Magic® to target a final position using a motion profile. 
 * Users can optionally provide a torque current feedforward.
 * 
 * Motion Magic® produces a motion profile in real-time while attempting to honor the Cruise Velocity,
 * Acceleration, and Jerk value specified via the Motion Magic® configuration values.  This control mode does
 * not use the Expo_kV or Expo_kA configs.  Target position can be changed on-the-fly and Motion Magic® will
 * do its best to adjust the profile.  This control mode is based on torque current, so relevant closed-loop
 * gains will use Amperes for the numerator.
 */
class MotionMagicTorqueCurrentFOC : public ControlRequest
{
    ctre::phoenix::StatusCode SendRequest(const char *network, uint32_t deviceHash, bool cancelOtherRequests, std::shared_ptr<ControlRequest> &req) override
    {
        if (req.get() != this)
        {
            auto const reqCast = dynamic_cast<MotionMagicTorqueCurrentFOC *>(req.get());
            if (reqCast != nullptr)
            {
                *reqCast = *this;
            }
            else
            {
                req = std::make_shared<MotionMagicTorqueCurrentFOC>(*this);
            }
        }

        return c_ctre_phoenix6_RequestControlMotionMagicTorqueCurrentFOC(network, deviceHash, UpdateFreqHz.to<double>(), cancelOtherRequests, Position.to<double>(), FeedForward.to<double>(), Slot, OverrideCoastDurNeutral, LimitForwardMotion, LimitReverseMotion);
    }

public:
    /**
     * Position to drive toward in rotations.
     */
    units::angle::turn_t Position;
    /**
     * Feedforward to apply in torque current in Amperes.  User can use motor's kT
     * to scale Newton-meter to Amperes.
     */
    units::current::ampere_t FeedForward;
    /**
     * Select which gains are applied by selecting the slot.  Use the configuration
     * api to set the gain values for the selected slot before enabling this
     * feature. Slot must be within [0,2].
     */
    int Slot;
    /**
     * Set to true to coast the rotor when output is zero (or within deadband).  Set
     * to false to use the NeutralMode configuration setting (default). This flag
     * exists to provide the fundamental behavior of this control when output is
     * zero, which is to provide 0A (zero torque).
     */
    bool OverrideCoastDurNeutral;
    /**
     * Set to true to force forward limiting.  This allows users to use other limit
     * switch sensors connected to robot controller.  This also allows use of active
     * sensors that require external power.
     */
    bool LimitForwardMotion;
    /**
     * Set to true to force reverse limiting.  This allows users to use other limit
     * switch sensors connected to robot controller.  This also allows use of active
     * sensors that require external power.
     */
    bool LimitReverseMotion;

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
     * \brief Requires Phoenix Pro;
     *        Requests Motion Magic® to target a final
     *        position using a motion profile.  Users can optionally provide a
     *        torque current feedforward.
     * 
     * \details Motion Magic® produces a motion profile in real-time while
     *          attempting to honor the Cruise Velocity, Acceleration, and Jerk
     *          value specified via the Motion Magic® configuration values.  This
     *          control mode does not use the Expo_kV or Expo_kA configs.  Target
     *          position can be changed on-the-fly and Motion Magic® will do its
     *          best to adjust the profile.  This control mode is based on torque
     *          current, so relevant closed-loop gains will use Amperes for the
     *          numerator.
     * 
     * \param Position    Position to drive toward in rotations.
     * \param FeedForward    Feedforward to apply in torque current in Amperes. 
     *                       User can use motor's kT to scale Newton-meter to
     *                       Amperes.
     * \param Slot    Select which gains are applied by selecting the slot.  Use the
     *                configuration api to set the gain values for the selected slot
     *                before enabling this feature. Slot must be within [0,2].
     * \param OverrideCoastDurNeutral    Set to true to coast the rotor when output
     *                                   is zero (or within deadband).  Set to false
     *                                   to use the NeutralMode configuration
     *                                   setting (default). This flag exists to
     *                                   provide the fundamental behavior of this
     *                                   control when output is zero, which is to
     *                                   provide 0A (zero torque).
     * \param LimitForwardMotion    Set to true to force forward limiting.  This
     *                              allows users to use other limit switch sensors
     *                              connected to robot controller.  This also allows
     *                              use of active sensors that require external
     *                              power.
     * \param LimitReverseMotion    Set to true to force reverse limiting.  This
     *                              allows users to use other limit switch sensors
     *                              connected to robot controller.  This also allows
     *                              use of active sensors that require external
     *                              power.
     */
    MotionMagicTorqueCurrentFOC(units::angle::turn_t Position, units::current::ampere_t FeedForward = 0.0_A, int Slot = 0, bool OverrideCoastDurNeutral = false, bool LimitForwardMotion = false, bool LimitReverseMotion = false) : ControlRequest{"MotionMagicTorqueCurrentFOC"},
        Position{std::move(Position)},
        FeedForward{std::move(FeedForward)},
        Slot{std::move(Slot)},
        OverrideCoastDurNeutral{std::move(OverrideCoastDurNeutral)},
        LimitForwardMotion{std::move(LimitForwardMotion)},
        LimitReverseMotion{std::move(LimitReverseMotion)}
    {}
    
    /**
     * \brief Modifies this Control Request's Position parameter and returns itself for
     *        method-chaining and easier to use request API.
     * \param newPosition Parameter to modify
     * \returns Itself
     */
    MotionMagicTorqueCurrentFOC& WithPosition(units::angle::turn_t newPosition)
    {
        Position = std::move(newPosition);
        return *this;
    }
    
    /**
     * \brief Modifies this Control Request's FeedForward parameter and returns itself for
     *        method-chaining and easier to use request API.
     * \param newFeedForward Parameter to modify
     * \returns Itself
     */
    MotionMagicTorqueCurrentFOC& WithFeedForward(units::current::ampere_t newFeedForward)
    {
        FeedForward = std::move(newFeedForward);
        return *this;
    }
    
    /**
     * \brief Modifies this Control Request's Slot parameter and returns itself for
     *        method-chaining and easier to use request API.
     * \param newSlot Parameter to modify
     * \returns Itself
     */
    MotionMagicTorqueCurrentFOC& WithSlot(int newSlot)
    {
        Slot = std::move(newSlot);
        return *this;
    }
    
    /**
     * \brief Modifies this Control Request's OverrideCoastDurNeutral parameter and returns itself for
     *        method-chaining and easier to use request API.
     * \param newOverrideCoastDurNeutral Parameter to modify
     * \returns Itself
     */
    MotionMagicTorqueCurrentFOC& WithOverrideCoastDurNeutral(bool newOverrideCoastDurNeutral)
    {
        OverrideCoastDurNeutral = std::move(newOverrideCoastDurNeutral);
        return *this;
    }
    
    /**
     * \brief Modifies this Control Request's LimitForwardMotion parameter and returns itself for
     *        method-chaining and easier to use request API.
     * \param newLimitForwardMotion Parameter to modify
     * \returns Itself
     */
    MotionMagicTorqueCurrentFOC& WithLimitForwardMotion(bool newLimitForwardMotion)
    {
        LimitForwardMotion = std::move(newLimitForwardMotion);
        return *this;
    }
    
    /**
     * \brief Modifies this Control Request's LimitReverseMotion parameter and returns itself for
     *        method-chaining and easier to use request API.
     * \param newLimitReverseMotion Parameter to modify
     * \returns Itself
     */
    MotionMagicTorqueCurrentFOC& WithLimitReverseMotion(bool newLimitReverseMotion)
    {
        LimitReverseMotion = std::move(newLimitReverseMotion);
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
    MotionMagicTorqueCurrentFOC &WithUpdateFreqHz(units::frequency::hertz_t newUpdateFreqHz)
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
        ss << "class: MotionMagicTorqueCurrentFOC" << std::endl;
        ss << "Position: " << Position.to<double>() << std::endl;
        ss << "FeedForward: " << FeedForward.to<double>() << std::endl;
        ss << "Slot: " << Slot << std::endl;
        ss << "OverrideCoastDurNeutral: " << OverrideCoastDurNeutral << std::endl;
        ss << "LimitForwardMotion: " << LimitForwardMotion << std::endl;
        ss << "LimitReverseMotion: " << LimitReverseMotion << std::endl;
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
        ss << Position.to<double>(); controlInfo["Position"] = ss.str(); ss.str(std::string{});
        ss << FeedForward.to<double>(); controlInfo["FeedForward"] = ss.str(); ss.str(std::string{});
        ss << Slot; controlInfo["Slot"] = ss.str(); ss.str(std::string{});
        ss << OverrideCoastDurNeutral; controlInfo["OverrideCoastDurNeutral"] = ss.str(); ss.str(std::string{});
        ss << LimitForwardMotion; controlInfo["LimitForwardMotion"] = ss.str(); ss.str(std::string{});
        ss << LimitReverseMotion; controlInfo["LimitReverseMotion"] = ss.str(); ss.str(std::string{});
        return controlInfo;
    }
};

}
}
}

