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
#include <units/frequency.h>
#include <units/time.h>


namespace ctre {
namespace phoenix6 {
namespace controls {

/**
 * Request PID to target position with a differential position setpoint.
 * 
 * This control mode will set the motor's position setpoint to the position specified by the user. It will
 * also set the motor's differential position setpoint to the specified position.
 */
class DifferentialPositionDutyCycle : public ControlRequest
{
    ctre::phoenix::StatusCode SendRequest(const char *network, uint32_t deviceHash, bool cancelOtherRequests, std::shared_ptr<ControlRequest> &req) override
    {
        if (req.get() != this)
        {
            auto const reqCast = dynamic_cast<DifferentialPositionDutyCycle *>(req.get());
            if (reqCast != nullptr)
            {
                *reqCast = *this;
            }
            else
            {
                req = std::make_shared<DifferentialPositionDutyCycle>(*this);
            }
        }

        return c_ctre_phoenix6_RequestControlDifferentialPositionDutyCycle(network, deviceHash, UpdateFreqHz.to<double>(), cancelOtherRequests, TargetPosition.to<double>(), DifferentialPosition.to<double>(), EnableFOC, TargetSlot, DifferentialSlot, OverrideBrakeDurNeutral, LimitForwardMotion, LimitReverseMotion);
    }

public:
    /**
     * Average position to drive toward in rotations.
     */
    units::angle::turn_t TargetPosition;
    /**
     * Differential position to drive toward in rotations.
     */
    units::angle::turn_t DifferentialPosition;
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
     * Select which gains are applied to the primary controller by selecting the
     * slot.  Use the configuration api to set the gain values for the selected slot
     * before enabling this feature. Slot must be within [0,2].
     */
    int TargetSlot;
    /**
     * Select which gains are applied to the differential controller by selecting
     * the slot.  Use the configuration api to set the gain values for the selected
     * slot before enabling this feature. Slot must be within [0,2].
     */
    int DifferentialSlot;
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
     * \brief Request PID to target position with a differential position setpoint.
     * 
     * \details This control mode will set the motor's position setpoint to the
     *          position specified by the user. It will also set the motor's
     *          differential position setpoint to the specified position.
     * 
     * \param TargetPosition    Average position to drive toward in rotations.
     * \param DifferentialPosition    Differential position to drive toward in
     *                                rotations.
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
     * \param TargetSlot    Select which gains are applied to the primary controller
     *                      by selecting the slot.  Use the configuration api to set
     *                      the gain values for the selected slot before enabling
     *                      this feature. Slot must be within [0,2].
     * \param DifferentialSlot    Select which gains are applied to the differential
     *                            controller by selecting the slot.  Use the
     *                            configuration api to set the gain values for the
     *                            selected slot before enabling this feature. Slot
     *                            must be within [0,2].
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
    DifferentialPositionDutyCycle(units::angle::turn_t TargetPosition, units::angle::turn_t DifferentialPosition, bool EnableFOC = true, int TargetSlot = 0, int DifferentialSlot = 1, bool OverrideBrakeDurNeutral = false, bool LimitForwardMotion = false, bool LimitReverseMotion = false) : ControlRequest{"DifferentialPositionDutyCycle"},
        TargetPosition{std::move(TargetPosition)},
        DifferentialPosition{std::move(DifferentialPosition)},
        EnableFOC{std::move(EnableFOC)},
        TargetSlot{std::move(TargetSlot)},
        DifferentialSlot{std::move(DifferentialSlot)},
        OverrideBrakeDurNeutral{std::move(OverrideBrakeDurNeutral)},
        LimitForwardMotion{std::move(LimitForwardMotion)},
        LimitReverseMotion{std::move(LimitReverseMotion)}
    {}
    
    /**
     * \brief Modifies this Control Request's TargetPosition parameter and returns itself for
     *        method-chaining and easier to use request API.
     * \param newTargetPosition Parameter to modify
     * \returns Itself
     */
    DifferentialPositionDutyCycle& WithTargetPosition(units::angle::turn_t newTargetPosition)
    {
        TargetPosition = std::move(newTargetPosition);
        return *this;
    }
    
    /**
     * \brief Modifies this Control Request's DifferentialPosition parameter and returns itself for
     *        method-chaining and easier to use request API.
     * \param newDifferentialPosition Parameter to modify
     * \returns Itself
     */
    DifferentialPositionDutyCycle& WithDifferentialPosition(units::angle::turn_t newDifferentialPosition)
    {
        DifferentialPosition = std::move(newDifferentialPosition);
        return *this;
    }
    
    /**
     * \brief Modifies this Control Request's EnableFOC parameter and returns itself for
     *        method-chaining and easier to use request API.
     * \param newEnableFOC Parameter to modify
     * \returns Itself
     */
    DifferentialPositionDutyCycle& WithEnableFOC(bool newEnableFOC)
    {
        EnableFOC = std::move(newEnableFOC);
        return *this;
    }
    
    /**
     * \brief Modifies this Control Request's TargetSlot parameter and returns itself for
     *        method-chaining and easier to use request API.
     * \param newTargetSlot Parameter to modify
     * \returns Itself
     */
    DifferentialPositionDutyCycle& WithTargetSlot(int newTargetSlot)
    {
        TargetSlot = std::move(newTargetSlot);
        return *this;
    }
    
    /**
     * \brief Modifies this Control Request's DifferentialSlot parameter and returns itself for
     *        method-chaining and easier to use request API.
     * \param newDifferentialSlot Parameter to modify
     * \returns Itself
     */
    DifferentialPositionDutyCycle& WithDifferentialSlot(int newDifferentialSlot)
    {
        DifferentialSlot = std::move(newDifferentialSlot);
        return *this;
    }
    
    /**
     * \brief Modifies this Control Request's OverrideBrakeDurNeutral parameter and returns itself for
     *        method-chaining and easier to use request API.
     * \param newOverrideBrakeDurNeutral Parameter to modify
     * \returns Itself
     */
    DifferentialPositionDutyCycle& WithOverrideBrakeDurNeutral(bool newOverrideBrakeDurNeutral)
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
    DifferentialPositionDutyCycle& WithLimitForwardMotion(bool newLimitForwardMotion)
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
    DifferentialPositionDutyCycle& WithLimitReverseMotion(bool newLimitReverseMotion)
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
    DifferentialPositionDutyCycle &WithUpdateFreqHz(units::frequency::hertz_t newUpdateFreqHz)
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
        ss << "class: DifferentialPositionDutyCycle" << std::endl;
        ss << "TargetPosition: " << TargetPosition.to<double>() << std::endl;
        ss << "DifferentialPosition: " << DifferentialPosition.to<double>() << std::endl;
        ss << "EnableFOC: " << EnableFOC << std::endl;
        ss << "TargetSlot: " << TargetSlot << std::endl;
        ss << "DifferentialSlot: " << DifferentialSlot << std::endl;
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
        ss << TargetPosition.to<double>(); controlInfo["TargetPosition"] = ss.str(); ss.str(std::string{});
        ss << DifferentialPosition.to<double>(); controlInfo["DifferentialPosition"] = ss.str(); ss.str(std::string{});
        ss << EnableFOC; controlInfo["EnableFOC"] = ss.str(); ss.str(std::string{});
        ss << TargetSlot; controlInfo["TargetSlot"] = ss.str(); ss.str(std::string{});
        ss << DifferentialSlot; controlInfo["DifferentialSlot"] = ss.str(); ss.str(std::string{});
        ss << OverrideBrakeDurNeutral; controlInfo["OverrideBrakeDurNeutral"] = ss.str(); ss.str(std::string{});
        ss << LimitForwardMotion; controlInfo["LimitForwardMotion"] = ss.str(); ss.str(std::string{});
        ss << LimitReverseMotion; controlInfo["LimitReverseMotion"] = ss.str(); ss.str(std::string{});
        return controlInfo;
    }
};

}
}
}

