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
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/voltage.h>
#include <units/frequency.h>
#include <units/time.h>


namespace ctre {
namespace phoenix6 {
namespace controls {

/**
 * Requests Motion Magic® to target a final velocity using a motion profile. 
 * This allows smooth transitions between velocity set points.  Users can
 * optionally provide a voltage feedforward.
 * 
 * Motion Magic® Velocity produces a motion profile in real-time while attempting to honor the specified
 * Acceleration and Jerk value. This control mode does not use the CruiseVelocity, Expo_kV, or Expo_kA
 * configs. If the specified acceleration is zero, the Acceleration under Motion Magic® configuration
 * parameter is used instead. This allows for runtime adjustment of acceleration for advanced users.  Jerk is
 * also specified in the Motion Magic® persistent configuration values.  If Jerk is set to zero, Motion Magic®
 * will produce a trapezoidal acceleration profile.  Target velocity can also be changed on-the-fly and Motion
 * Magic® will do its best to adjust the profile.  This control mode is voltage-based, so relevant closed-loop
 * gains will use Volts for the numerator.
 */
class MotionMagicVelocityVoltage : public ControlRequest
{
    ctre::phoenix::StatusCode SendRequest(const char *network, uint32_t deviceHash, bool cancelOtherRequests, std::shared_ptr<ControlRequest> &req) override
    {
        if (req.get() != this)
        {
            auto const reqCast = dynamic_cast<MotionMagicVelocityVoltage *>(req.get());
            if (reqCast != nullptr)
            {
                *reqCast = *this;
            }
            else
            {
                req = std::make_shared<MotionMagicVelocityVoltage>(*this);
            }
        }

        return c_ctre_phoenix6_RequestControlMotionMagicVelocityVoltage(network, deviceHash, UpdateFreqHz.to<double>(), cancelOtherRequests, Velocity.to<double>(), Acceleration.to<double>(), EnableFOC, FeedForward.to<double>(), Slot, OverrideBrakeDurNeutral, LimitForwardMotion, LimitReverseMotion);
    }

public:
    /**
     * Target velocity to drive toward in rotations per second.  This can be changed
     * on-the fly.
     */
    units::angular_velocity::turns_per_second_t Velocity;
    /**
     * This is the absolute Acceleration to use generating the profile.  If this
     * parameter is zero, the Acceleration persistent configuration parameter is
     * used instead. Acceleration is in rotations per second squared.  If nonzero,
     * the signage does not matter as the absolute value is used.
     */
    units::angular_acceleration::turns_per_second_squared_t Acceleration;
    /**
     * Set to true to use FOC commutation (requires Phoenix Pro), which increases
     * peak power by ~15%. Set to false to use trapezoidal commutation.  FOC
     * improves motor performance by leveraging torque (current) control.  However,
     * this may be inconvenient for applications that require specifying duty cycle
     * or voltage.  CTR-Electronics has developed a hybrid method that combines the
     * performances gains of FOC while still allowing applications to provide duty
     * cycle or voltage demand.  This not to be confused with simple sinusoidal
     * control or phase voltage control which lacks the performance gains.
     */
    bool EnableFOC;
    /**
     * Feedforward to apply in volts
     */
    units::voltage::volt_t FeedForward;
    /**
     * Select which gains are applied by selecting the slot.  Use the configuration
     * api to set the gain values for the selected slot before enabling this
     * feature. Slot must be within [0,2].
     */
    int Slot;
    /**
     * Set to true to static-brake the rotor when output is zero (or within
     * deadband).  Set to false to use the NeutralMode configuration setting
     * (default). This flag exists to provide the fundamental behavior of this
     * control when output is zero, which is to provide 0V to the motor.
     */
    bool OverrideBrakeDurNeutral;
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
     * \brief Requests Motion Magic® to target a final velocity using a motion
     *        profile.  This allows smooth transitions between velocity set points. 
     *        Users can optionally provide a voltage feedforward.
     * 
     * \details Motion Magic® Velocity produces a motion profile in real-time while
     *          attempting to honor the specified Acceleration and Jerk value. This
     *          control mode does not use the CruiseVelocity, Expo_kV, or Expo_kA
     *          configs. If the specified acceleration is zero, the Acceleration
     *          under Motion Magic® configuration parameter is used instead. This
     *          allows for runtime adjustment of acceleration for advanced users. 
     *          Jerk is also specified in the Motion Magic® persistent configuration
     *          values.  If Jerk is set to zero, Motion Magic® will produce a
     *          trapezoidal acceleration profile.  Target velocity can also be
     *          changed on-the-fly and Motion Magic® will do its best to adjust the
     *          profile.  This control mode is voltage-based, so relevant
     *          closed-loop gains will use Volts for the numerator.
     * 
     * \param Velocity    Target velocity to drive toward in rotations per second. 
     *                    This can be changed on-the fly.
     * \param Acceleration    This is the absolute Acceleration to use generating
     *                        the profile.  If this parameter is zero, the
     *                        Acceleration persistent configuration parameter is
     *                        used instead. Acceleration is in rotations per second
     *                        squared.  If nonzero, the signage does not matter as
     *                        the absolute value is used.
     * \param EnableFOC    Set to true to use FOC commutation (requires Phoenix
     *                     Pro), which increases peak power by ~15%. Set to false to
     *                     use trapezoidal commutation.  FOC improves motor
     *                     performance by leveraging torque (current) control. 
     *                     However, this may be inconvenient for applications that
     *                     require specifying duty cycle or voltage. 
     *                     CTR-Electronics has developed a hybrid method that
     *                     combines the performances gains of FOC while still
     *                     allowing applications to provide duty cycle or voltage
     *                     demand.  This not to be confused with simple sinusoidal
     *                     control or phase voltage control which lacks the
     *                     performance gains.
     * \param FeedForward    Feedforward to apply in volts
     * \param Slot    Select which gains are applied by selecting the slot.  Use the
     *                configuration api to set the gain values for the selected slot
     *                before enabling this feature. Slot must be within [0,2].
     * \param OverrideBrakeDurNeutral    Set to true to static-brake the rotor when
     *                                   output is zero (or within deadband).  Set
     *                                   to false to use the NeutralMode
     *                                   configuration setting (default). This flag
     *                                   exists to provide the fundamental behavior
     *                                   of this control when output is zero, which
     *                                   is to provide 0V to the motor.
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
    MotionMagicVelocityVoltage(units::angular_velocity::turns_per_second_t Velocity, units::angular_acceleration::turns_per_second_squared_t Acceleration = 0.0_tr_per_s_sq, bool EnableFOC = true, units::voltage::volt_t FeedForward = 0.0_V, int Slot = 0, bool OverrideBrakeDurNeutral = false, bool LimitForwardMotion = false, bool LimitReverseMotion = false) : ControlRequest{"MotionMagicVelocityVoltage"},
        Velocity{std::move(Velocity)},
        Acceleration{std::move(Acceleration)},
        EnableFOC{std::move(EnableFOC)},
        FeedForward{std::move(FeedForward)},
        Slot{std::move(Slot)},
        OverrideBrakeDurNeutral{std::move(OverrideBrakeDurNeutral)},
        LimitForwardMotion{std::move(LimitForwardMotion)},
        LimitReverseMotion{std::move(LimitReverseMotion)}
    {}
    
    /**
     * \brief Modifies this Control Request's Velocity parameter and returns itself for
     *        method-chaining and easier to use request API.
     * \param newVelocity Parameter to modify
     * \returns Itself
     */
    MotionMagicVelocityVoltage& WithVelocity(units::angular_velocity::turns_per_second_t newVelocity)
    {
        Velocity = std::move(newVelocity);
        return *this;
    }
    
    /**
     * \brief Modifies this Control Request's Acceleration parameter and returns itself for
     *        method-chaining and easier to use request API.
     * \param newAcceleration Parameter to modify
     * \returns Itself
     */
    MotionMagicVelocityVoltage& WithAcceleration(units::angular_acceleration::turns_per_second_squared_t newAcceleration)
    {
        Acceleration = std::move(newAcceleration);
        return *this;
    }
    
    /**
     * \brief Modifies this Control Request's EnableFOC parameter and returns itself for
     *        method-chaining and easier to use request API.
     * \param newEnableFOC Parameter to modify
     * \returns Itself
     */
    MotionMagicVelocityVoltage& WithEnableFOC(bool newEnableFOC)
    {
        EnableFOC = std::move(newEnableFOC);
        return *this;
    }
    
    /**
     * \brief Modifies this Control Request's FeedForward parameter and returns itself for
     *        method-chaining and easier to use request API.
     * \param newFeedForward Parameter to modify
     * \returns Itself
     */
    MotionMagicVelocityVoltage& WithFeedForward(units::voltage::volt_t newFeedForward)
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
    MotionMagicVelocityVoltage& WithSlot(int newSlot)
    {
        Slot = std::move(newSlot);
        return *this;
    }
    
    /**
     * \brief Modifies this Control Request's OverrideBrakeDurNeutral parameter and returns itself for
     *        method-chaining and easier to use request API.
     * \param newOverrideBrakeDurNeutral Parameter to modify
     * \returns Itself
     */
    MotionMagicVelocityVoltage& WithOverrideBrakeDurNeutral(bool newOverrideBrakeDurNeutral)
    {
        OverrideBrakeDurNeutral = std::move(newOverrideBrakeDurNeutral);
        return *this;
    }
    
    /**
     * \brief Modifies this Control Request's LimitForwardMotion parameter and returns itself for
     *        method-chaining and easier to use request API.
     * \param newLimitForwardMotion Parameter to modify
     * \returns Itself
     */
    MotionMagicVelocityVoltage& WithLimitForwardMotion(bool newLimitForwardMotion)
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
    MotionMagicVelocityVoltage& WithLimitReverseMotion(bool newLimitReverseMotion)
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
    MotionMagicVelocityVoltage &WithUpdateFreqHz(units::frequency::hertz_t newUpdateFreqHz)
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
        ss << "class: MotionMagicVelocityVoltage" << std::endl;
        ss << "Velocity: " << Velocity.to<double>() << std::endl;
        ss << "Acceleration: " << Acceleration.to<double>() << std::endl;
        ss << "EnableFOC: " << EnableFOC << std::endl;
        ss << "FeedForward: " << FeedForward.to<double>() << std::endl;
        ss << "Slot: " << Slot << std::endl;
        ss << "OverrideBrakeDurNeutral: " << OverrideBrakeDurNeutral << std::endl;
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
        ss << Velocity.to<double>(); controlInfo["Velocity"] = ss.str(); ss.str(std::string{});
        ss << Acceleration.to<double>(); controlInfo["Acceleration"] = ss.str(); ss.str(std::string{});
        ss << EnableFOC; controlInfo["EnableFOC"] = ss.str(); ss.str(std::string{});
        ss << FeedForward.to<double>(); controlInfo["FeedForward"] = ss.str(); ss.str(std::string{});
        ss << Slot; controlInfo["Slot"] = ss.str(); ss.str(std::string{});
        ss << OverrideBrakeDurNeutral; controlInfo["OverrideBrakeDurNeutral"] = ss.str(); ss.str(std::string{});
        ss << LimitForwardMotion; controlInfo["LimitForwardMotion"] = ss.str(); ss.str(std::string{});
        ss << LimitReverseMotion; controlInfo["LimitReverseMotion"] = ss.str(); ss.str(std::string{});
        return controlInfo;
    }
};

}
}
}

