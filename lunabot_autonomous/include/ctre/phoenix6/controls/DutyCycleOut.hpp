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
#include <units/dimensionless.h>
#include <units/frequency.h>
#include <units/time.h>


namespace ctre {
namespace phoenix6 {
namespace controls {

/**
 * Request a specified motor duty cycle.
 * 
 * This control mode will output a proportion of the supplied voltage which is supplied by the user.
 */
class DutyCycleOut : public ControlRequest
{
    ctre::phoenix::StatusCode SendRequest(const char *network, uint32_t deviceHash, bool cancelOtherRequests, std::shared_ptr<ControlRequest> &req) override
    {
        if (req.get() != this)
        {
            auto const reqCast = dynamic_cast<DutyCycleOut *>(req.get());
            if (reqCast != nullptr)
            {
                *reqCast = *this;
            }
            else
            {
                req = std::make_shared<DutyCycleOut>(*this);
            }
        }

        return c_ctre_phoenix6_RequestControlDutyCycleOut(network, deviceHash, UpdateFreqHz.to<double>(), cancelOtherRequests, Output.to<double>(), EnableFOC, OverrideBrakeDurNeutral, LimitForwardMotion, LimitReverseMotion);
    }

public:
    /**
     * Proportion of supply voltage to apply in fractional units between -1 and +1
     */
    units::dimensionless::scalar_t Output;
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
     * \brief Request a specified motor duty cycle.
     * 
     * \details This control mode will output a proportion of the supplied voltage
     *          which is supplied by the user.
     * 
     * \param Output    Proportion of supply voltage to apply in fractional units
     *                  between -1 and +1
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
    DutyCycleOut(units::dimensionless::scalar_t Output, bool EnableFOC = true, bool OverrideBrakeDurNeutral = false, bool LimitForwardMotion = false, bool LimitReverseMotion = false) : ControlRequest{"DutyCycleOut"},
        Output{std::move(Output)},
        EnableFOC{std::move(EnableFOC)},
        OverrideBrakeDurNeutral{std::move(OverrideBrakeDurNeutral)},
        LimitForwardMotion{std::move(LimitForwardMotion)},
        LimitReverseMotion{std::move(LimitReverseMotion)}
    {}
    
    /**
     * \brief Modifies this Control Request's Output parameter and returns itself for
     *        method-chaining and easier to use request API.
     * \param newOutput Parameter to modify
     * \returns Itself
     */
    DutyCycleOut& WithOutput(units::dimensionless::scalar_t newOutput)
    {
        Output = std::move(newOutput);
        return *this;
    }
    
    /**
     * \brief Modifies this Control Request's EnableFOC parameter and returns itself for
     *        method-chaining and easier to use request API.
     * \param newEnableFOC Parameter to modify
     * \returns Itself
     */
    DutyCycleOut& WithEnableFOC(bool newEnableFOC)
    {
        EnableFOC = std::move(newEnableFOC);
        return *this;
    }
    
    /**
     * \brief Modifies this Control Request's OverrideBrakeDurNeutral parameter and returns itself for
     *        method-chaining and easier to use request API.
     * \param newOverrideBrakeDurNeutral Parameter to modify
     * \returns Itself
     */
    DutyCycleOut& WithOverrideBrakeDurNeutral(bool newOverrideBrakeDurNeutral)
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
    DutyCycleOut& WithLimitForwardMotion(bool newLimitForwardMotion)
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
    DutyCycleOut& WithLimitReverseMotion(bool newLimitReverseMotion)
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
    DutyCycleOut &WithUpdateFreqHz(units::frequency::hertz_t newUpdateFreqHz)
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
        ss << "class: DutyCycleOut" << std::endl;
        ss << "Output: " << Output.to<double>() << std::endl;
        ss << "EnableFOC: " << EnableFOC << std::endl;
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
        ss << Output.to<double>(); controlInfo["Output"] = ss.str(); ss.str(std::string{});
        ss << EnableFOC; controlInfo["EnableFOC"] = ss.str(); ss.str(std::string{});
        ss << OverrideBrakeDurNeutral; controlInfo["OverrideBrakeDurNeutral"] = ss.str(); ss.str(std::string{});
        ss << LimitForwardMotion; controlInfo["LimitForwardMotion"] = ss.str(); ss.str(std::string{});
        ss << LimitReverseMotion; controlInfo["LimitReverseMotion"] = ss.str(); ss.str(std::string{});
        return controlInfo;
    }
};

}
}
}

