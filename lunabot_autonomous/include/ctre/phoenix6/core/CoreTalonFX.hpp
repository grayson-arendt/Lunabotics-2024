/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
#pragma once

#include "ctre/phoenix6/hardware/ParentDevice.hpp"
#include "ctre/phoenix6/spns/SpnValue.hpp"
#include "ctre/phoenix6/Utils.hpp"
#include "ctre/phoenix6/controls/DutyCycleOut.hpp"
#include "ctre/phoenix6/controls/TorqueCurrentFOC.hpp"
#include "ctre/phoenix6/controls/VoltageOut.hpp"
#include "ctre/phoenix6/controls/PositionDutyCycle.hpp"
#include "ctre/phoenix6/controls/PositionVoltage.hpp"
#include "ctre/phoenix6/controls/PositionTorqueCurrentFOC.hpp"
#include "ctre/phoenix6/controls/VelocityDutyCycle.hpp"
#include "ctre/phoenix6/controls/VelocityVoltage.hpp"
#include "ctre/phoenix6/controls/VelocityTorqueCurrentFOC.hpp"
#include "ctre/phoenix6/controls/MotionMagicDutyCycle.hpp"
#include "ctre/phoenix6/controls/MotionMagicVoltage.hpp"
#include "ctre/phoenix6/controls/MotionMagicTorqueCurrentFOC.hpp"
#include "ctre/phoenix6/controls/DifferentialDutyCycle.hpp"
#include "ctre/phoenix6/controls/DifferentialVoltage.hpp"
#include "ctre/phoenix6/controls/DifferentialPositionDutyCycle.hpp"
#include "ctre/phoenix6/controls/DifferentialPositionVoltage.hpp"
#include "ctre/phoenix6/controls/DifferentialVelocityDutyCycle.hpp"
#include "ctre/phoenix6/controls/DifferentialVelocityVoltage.hpp"
#include "ctre/phoenix6/controls/DifferentialMotionMagicDutyCycle.hpp"
#include "ctre/phoenix6/controls/DifferentialMotionMagicVoltage.hpp"
#include "ctre/phoenix6/controls/Follower.hpp"
#include "ctre/phoenix6/controls/StrictFollower.hpp"
#include "ctre/phoenix6/controls/DifferentialFollower.hpp"
#include "ctre/phoenix6/controls/DifferentialStrictFollower.hpp"
#include "ctre/phoenix6/controls/NeutralOut.hpp"
#include "ctre/phoenix6/controls/CoastOut.hpp"
#include "ctre/phoenix6/controls/StaticBrake.hpp"
#include "ctre/phoenix6/controls/MusicTone.hpp"
#include "ctre/phoenix6/controls/MotionMagicVelocityDutyCycle.hpp"
#include "ctre/phoenix6/controls/MotionMagicVelocityTorqueCurrentFOC.hpp"
#include "ctre/phoenix6/controls/MotionMagicVelocityVoltage.hpp"
#include "ctre/phoenix6/controls/MotionMagicExpoDutyCycle.hpp"
#include "ctre/phoenix6/controls/MotionMagicExpoVoltage.hpp"
#include "ctre/phoenix6/controls/MotionMagicExpoTorqueCurrentFOC.hpp"
#include "ctre/phoenix6/controls/DynamicMotionMagicDutyCycle.hpp"
#include "ctre/phoenix6/controls/DynamicMotionMagicVoltage.hpp"
#include "ctre/phoenix6/controls/DynamicMotionMagicTorqueCurrentFOC.hpp"
#include "ctre/phoenix6/controls/compound/Diff_DutyCycleOut_Position.hpp"
#include "ctre/phoenix6/controls/compound/Diff_PositionDutyCycle_Position.hpp"
#include "ctre/phoenix6/controls/compound/Diff_VelocityDutyCycle_Position.hpp"
#include "ctre/phoenix6/controls/compound/Diff_MotionMagicDutyCycle_Position.hpp"
#include "ctre/phoenix6/controls/compound/Diff_DutyCycleOut_Velocity.hpp"
#include "ctre/phoenix6/controls/compound/Diff_PositionDutyCycle_Velocity.hpp"
#include "ctre/phoenix6/controls/compound/Diff_VelocityDutyCycle_Velocity.hpp"
#include "ctre/phoenix6/controls/compound/Diff_MotionMagicDutyCycle_Velocity.hpp"
#include "ctre/phoenix6/controls/compound/Diff_VoltageOut_Position.hpp"
#include "ctre/phoenix6/controls/compound/Diff_PositionVoltage_Position.hpp"
#include "ctre/phoenix6/controls/compound/Diff_VelocityVoltage_Position.hpp"
#include "ctre/phoenix6/controls/compound/Diff_MotionMagicVoltage_Position.hpp"
#include "ctre/phoenix6/controls/compound/Diff_VoltageOut_Velocity.hpp"
#include "ctre/phoenix6/controls/compound/Diff_PositionVoltage_Velocity.hpp"
#include "ctre/phoenix6/controls/compound/Diff_VelocityVoltage_Velocity.hpp"
#include "ctre/phoenix6/controls/compound/Diff_MotionMagicVoltage_Velocity.hpp"
#include "ctre/phoenix6/controls/compound/Diff_TorqueCurrentFOC_Position.hpp"
#include "ctre/phoenix6/controls/compound/Diff_PositionTorqueCurrentFOC_Position.hpp"
#include "ctre/phoenix6/controls/compound/Diff_VelocityTorqueCurrentFOC_Position.hpp"
#include "ctre/phoenix6/controls/compound/Diff_MotionMagicTorqueCurrentFOC_Position.hpp"
#include "ctre/phoenix6/controls/compound/Diff_TorqueCurrentFOC_Velocity.hpp"
#include "ctre/phoenix6/controls/compound/Diff_PositionTorqueCurrentFOC_Velocity.hpp"
#include "ctre/phoenix6/controls/compound/Diff_VelocityTorqueCurrentFOC_Velocity.hpp"
#include "ctre/phoenix6/controls/compound/Diff_MotionMagicTorqueCurrentFOC_Velocity.hpp"
#include "ctre/phoenix6/sim/TalonFXSimState.hpp"
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/dimensionless.h>
#include <units/temperature.h>
#include <units/voltage.h>

namespace ctre {
namespace phoenix6 {

namespace hardware {
namespace core {
    class CoreTalonFX;
}
}

namespace configs {

/**
 * Class description for the Talon FX integrated motor controller.
 *
 * This handles the configurations for the hardware#TalonFX
 */
class TalonFXConfiguration : public ParentConfiguration
{
public:
    /**
     * \brief True if we should factory default newer unsupported configs,
     *        false to leave newer unsupported configs alone.
     *
     * \details This flag addresses a corner case where the device may have
     *          firmware with newer configs that didn't exist when this
     *          version of the API was built. If this occurs and this
     *          flag is true, unsupported new configs will be factory
     *          defaulted to avoid unexpected behavior.
     *
     *          This is also the behavior in Phoenix 5, so this flag
     *          is defaulted to true to match.
     */
    bool FutureProofConfigs{true};

    
    /**
     * \brief Configs that directly affect motor-output.
     * 
     * \details Includes Motor Invert and various limit features.
     */
    MotorOutputConfigs MotorOutput;
    
    /**
     * \brief Configs that directly affect current limiting features.
     * 
     * \details Contains the supply/stator current limit thresholds and
     *          whether to enable them or not.
     */
    CurrentLimitsConfigs CurrentLimits;
    
    /**
     * \brief Voltage-specific configs
     * 
     * \details Voltage-specific configs
     */
    VoltageConfigs Voltage;
    
    /**
     * \brief Configs to control the maximum and minimum applied torque
     *        when using Torque Current control types.
     * 
     * \details Similar to peak output, but for the TorqueCurrentFOC
     *          control type requests.
     */
    TorqueCurrentConfigs TorqueCurrent;
    
    /**
     * \brief Configs that affect the feedback of this motor controller.
     * 
     * \details Includes feedback sensor source, any offsets for the
     *          feedback sensor, and various ratios to describe the
     *          relationship between the sensor and the mechanism for
     *          closed looping.
     */
    FeedbackConfigs Feedback;
    
    /**
     * \brief Configs related to sensors used for differential control of
     *        a mechanism.
     * 
     * \details Includes the differential sensor sources and IDs.
     */
    DifferentialSensorsConfigs DifferentialSensors;
    
    /**
     * \brief Configs related to constants used for differential control
     *        of a mechanism.
     * 
     * \details Includes the differential peak outputs.
     */
    DifferentialConstantsConfigs DifferentialConstants;
    
    /**
     * \brief Configs that affect the open-loop control of this motor
     *        controller.
     * 
     * \details Open-loop ramp rates for the various control types.
     */
    OpenLoopRampsConfigs OpenLoopRamps;
    
    /**
     * \brief Configs that affect the closed-loop control of this motor
     *        controller.
     * 
     * \details Closed-loop ramp rates for the various control types.
     */
    ClosedLoopRampsConfigs ClosedLoopRamps;
    
    /**
     * \brief Configs that change how the motor controller behaves under
     *        different limit switch statse.
     * 
     * \details Includes configs such as enabling limit switches,
     *          configuring the remote sensor ID, the source, and the
     *          position to set on limit.
     */
    HardwareLimitSwitchConfigs HardwareLimitSwitch;
    
    /**
     * \brief Configs that affect audible components of the device.
     * 
     * \details Includes configuration for the beep on boot.
     */
    AudioConfigs Audio;
    
    /**
     * \brief Configs that affect how software-limit switches behave.
     * 
     * \details Includes enabling software-limit switches and the
     *          threshold at which they're tripped.
     */
    SoftwareLimitSwitchConfigs SoftwareLimitSwitch;
    
    /**
     * \brief Configs for Motion Magic®.
     * 
     * \details Includes Velocity, Acceleration, Jerk, and Expo
     *          parameters.
     */
    MotionMagicConfigs MotionMagic;
    
    /**
     * \brief Custom Params.
     * 
     * \details Custom paramaters that have no real impact on controller.
     */
    CustomParamsConfigs CustomParams;
    
    /**
     * \brief Configs that affect general behavior during closed-looping.
     * 
     * \details Includes Continuous Wrap features.
     */
    ClosedLoopGeneralConfigs ClosedLoopGeneral;
    
    /**
     * \brief Gains for the specified slot.
     * 
     * \details If this slot is selected, these gains are used in closed
     *          loop control requests.
     */
    Slot0Configs Slot0;
    
    /**
     * \brief Gains for the specified slot.
     * 
     * \details If this slot is selected, these gains are used in closed
     *          loop control requests.
     */
    Slot1Configs Slot1;
    
    /**
     * \brief Gains for the specified slot.
     * 
     * \details If this slot is selected, these gains are used in closed
     *          loop control requests.
     */
    Slot2Configs Slot2;
    
    /**
     * \brief Modifies this configuration's MotorOutput parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newMotorOutput Parameter to modify
     * \returns Itself
     */
    TalonFXConfiguration& WithMotorOutput(MotorOutputConfigs newMotorOutput)
    {
        MotorOutput = std::move(newMotorOutput);
        return *this;
    }
    
    /**
     * \brief Modifies this configuration's CurrentLimits parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newCurrentLimits Parameter to modify
     * \returns Itself
     */
    TalonFXConfiguration& WithCurrentLimits(CurrentLimitsConfigs newCurrentLimits)
    {
        CurrentLimits = std::move(newCurrentLimits);
        return *this;
    }
    
    /**
     * \brief Modifies this configuration's Voltage parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newVoltage Parameter to modify
     * \returns Itself
     */
    TalonFXConfiguration& WithVoltage(VoltageConfigs newVoltage)
    {
        Voltage = std::move(newVoltage);
        return *this;
    }
    
    /**
     * \brief Modifies this configuration's TorqueCurrent parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newTorqueCurrent Parameter to modify
     * \returns Itself
     */
    TalonFXConfiguration& WithTorqueCurrent(TorqueCurrentConfigs newTorqueCurrent)
    {
        TorqueCurrent = std::move(newTorqueCurrent);
        return *this;
    }
    
    /**
     * \brief Modifies this configuration's Feedback parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newFeedback Parameter to modify
     * \returns Itself
     */
    TalonFXConfiguration& WithFeedback(FeedbackConfigs newFeedback)
    {
        Feedback = std::move(newFeedback);
        return *this;
    }
    
    /**
     * \brief Modifies this configuration's DifferentialSensors parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newDifferentialSensors Parameter to modify
     * \returns Itself
     */
    TalonFXConfiguration& WithDifferentialSensors(DifferentialSensorsConfigs newDifferentialSensors)
    {
        DifferentialSensors = std::move(newDifferentialSensors);
        return *this;
    }
    
    /**
     * \brief Modifies this configuration's DifferentialConstants parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newDifferentialConstants Parameter to modify
     * \returns Itself
     */
    TalonFXConfiguration& WithDifferentialConstants(DifferentialConstantsConfigs newDifferentialConstants)
    {
        DifferentialConstants = std::move(newDifferentialConstants);
        return *this;
    }
    
    /**
     * \brief Modifies this configuration's OpenLoopRamps parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newOpenLoopRamps Parameter to modify
     * \returns Itself
     */
    TalonFXConfiguration& WithOpenLoopRamps(OpenLoopRampsConfigs newOpenLoopRamps)
    {
        OpenLoopRamps = std::move(newOpenLoopRamps);
        return *this;
    }
    
    /**
     * \brief Modifies this configuration's ClosedLoopRamps parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newClosedLoopRamps Parameter to modify
     * \returns Itself
     */
    TalonFXConfiguration& WithClosedLoopRamps(ClosedLoopRampsConfigs newClosedLoopRamps)
    {
        ClosedLoopRamps = std::move(newClosedLoopRamps);
        return *this;
    }
    
    /**
     * \brief Modifies this configuration's HardwareLimitSwitch parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newHardwareLimitSwitch Parameter to modify
     * \returns Itself
     */
    TalonFXConfiguration& WithHardwareLimitSwitch(HardwareLimitSwitchConfigs newHardwareLimitSwitch)
    {
        HardwareLimitSwitch = std::move(newHardwareLimitSwitch);
        return *this;
    }
    
    /**
     * \brief Modifies this configuration's Audio parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newAudio Parameter to modify
     * \returns Itself
     */
    TalonFXConfiguration& WithAudio(AudioConfigs newAudio)
    {
        Audio = std::move(newAudio);
        return *this;
    }
    
    /**
     * \brief Modifies this configuration's SoftwareLimitSwitch parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newSoftwareLimitSwitch Parameter to modify
     * \returns Itself
     */
    TalonFXConfiguration& WithSoftwareLimitSwitch(SoftwareLimitSwitchConfigs newSoftwareLimitSwitch)
    {
        SoftwareLimitSwitch = std::move(newSoftwareLimitSwitch);
        return *this;
    }
    
    /**
     * \brief Modifies this configuration's MotionMagic parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newMotionMagic Parameter to modify
     * \returns Itself
     */
    TalonFXConfiguration& WithMotionMagic(MotionMagicConfigs newMotionMagic)
    {
        MotionMagic = std::move(newMotionMagic);
        return *this;
    }
    
    /**
     * \brief Modifies this configuration's CustomParams parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newCustomParams Parameter to modify
     * \returns Itself
     */
    TalonFXConfiguration& WithCustomParams(CustomParamsConfigs newCustomParams)
    {
        CustomParams = std::move(newCustomParams);
        return *this;
    }
    
    /**
     * \brief Modifies this configuration's ClosedLoopGeneral parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newClosedLoopGeneral Parameter to modify
     * \returns Itself
     */
    TalonFXConfiguration& WithClosedLoopGeneral(ClosedLoopGeneralConfigs newClosedLoopGeneral)
    {
        ClosedLoopGeneral = std::move(newClosedLoopGeneral);
        return *this;
    }
    
    /**
     * \brief Modifies this configuration's Slot0 parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newSlot0 Parameter to modify
     * \returns Itself
     */
    TalonFXConfiguration& WithSlot0(Slot0Configs newSlot0)
    {
        Slot0 = std::move(newSlot0);
        return *this;
    }
    
    /**
     * \brief Modifies this configuration's Slot1 parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newSlot1 Parameter to modify
     * \returns Itself
     */
    TalonFXConfiguration& WithSlot1(Slot1Configs newSlot1)
    {
        Slot1 = std::move(newSlot1);
        return *this;
    }
    
    /**
     * \brief Modifies this configuration's Slot2 parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newSlot2 Parameter to modify
     * \returns Itself
     */
    TalonFXConfiguration& WithSlot2(Slot2Configs newSlot2)
    {
        Slot2 = std::move(newSlot2);
        return *this;
    }

    /**
     * \brief Get the string representation of this configuration
     */
    std::string ToString() const
    {
        std::stringstream ss;
        ss << MotorOutput.ToString();
        ss << CurrentLimits.ToString();
        ss << Voltage.ToString();
        ss << TorqueCurrent.ToString();
        ss << Feedback.ToString();
        ss << DifferentialSensors.ToString();
        ss << DifferentialConstants.ToString();
        ss << OpenLoopRamps.ToString();
        ss << ClosedLoopRamps.ToString();
        ss << HardwareLimitSwitch.ToString();
        ss << Audio.ToString();
        ss << SoftwareLimitSwitch.ToString();
        ss << MotionMagic.ToString();
        ss << CustomParams.ToString();
        ss << ClosedLoopGeneral.ToString();
        ss << Slot0.ToString();
        ss << Slot1.ToString();
        ss << Slot2.ToString();
        return ss.str();
    }

    /**
     * \brief Get the serialized form of this configuration
     */
    std::string Serialize() const
    {
        std::stringstream ss;
        ss << MotorOutput.Serialize();
        ss << CurrentLimits.Serialize();
        ss << Voltage.Serialize();
        ss << TorqueCurrent.Serialize();
        ss << Feedback.Serialize();
        ss << DifferentialSensors.Serialize();
        ss << DifferentialConstants.Serialize();
        ss << OpenLoopRamps.Serialize();
        ss << ClosedLoopRamps.Serialize();
        ss << HardwareLimitSwitch.Serialize();
        ss << Audio.Serialize();
        ss << SoftwareLimitSwitch.Serialize();
        ss << MotionMagic.Serialize();
        ss << CustomParams.Serialize();
        ss << ClosedLoopGeneral.Serialize();
        ss << Slot0.Serialize();
        ss << Slot1.Serialize();
        ss << Slot2.Serialize();
        return ss.str();
    }

    /**
     * \brief Take a string and deserialize it to this configuration
     */
    ctre::phoenix::StatusCode Deserialize(const std::string& to_deserialize)
    {
        ctre::phoenix::StatusCode err = ctre::phoenix::StatusCode::OK;
        err = MotorOutput.Deserialize(to_deserialize);
        err = CurrentLimits.Deserialize(to_deserialize);
        err = Voltage.Deserialize(to_deserialize);
        err = TorqueCurrent.Deserialize(to_deserialize);
        err = Feedback.Deserialize(to_deserialize);
        err = DifferentialSensors.Deserialize(to_deserialize);
        err = DifferentialConstants.Deserialize(to_deserialize);
        err = OpenLoopRamps.Deserialize(to_deserialize);
        err = ClosedLoopRamps.Deserialize(to_deserialize);
        err = HardwareLimitSwitch.Deserialize(to_deserialize);
        err = Audio.Deserialize(to_deserialize);
        err = SoftwareLimitSwitch.Deserialize(to_deserialize);
        err = MotionMagic.Deserialize(to_deserialize);
        err = CustomParams.Deserialize(to_deserialize);
        err = ClosedLoopGeneral.Deserialize(to_deserialize);
        err = Slot0.Deserialize(to_deserialize);
        err = Slot1.Deserialize(to_deserialize);
        err = Slot2.Deserialize(to_deserialize);
        return err;
    }
};

/**
 * Class description for the Talon FX integrated motor controller.
 *
 * This handles the configurations for the hardware#TalonFX
 */
class TalonFXConfigurator : public ParentConfigurator
{
    TalonFXConfigurator (hardware::DeviceIdentifier id):
        ParentConfigurator{std::move(id)}
    {}

    friend hardware::core::CoreTalonFX;
public:

    /**
     * Delete the copy constructor, we can only pass by reference
     */
    TalonFXConfigurator(const TalonFXConfigurator&) = delete;

    /**
     * \brief Refreshes the values of the specified config group.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(TalonFXConfiguration& configs) const
    {
        return Refresh(configs, DefaultTimeoutSeconds);
    }

    /**
     * \brief Refreshes the values of the specified config group.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(TalonFXConfiguration& configs, units::time::second_t timeoutSeconds) const
    {
        std::string ref;
        ctre::phoenix::StatusCode ret = GetConfigsPrivate(ref, timeoutSeconds);
        configs.Deserialize(ref);
        return ret;
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const TalonFXConfiguration& configs)
    {
        return Apply(configs, DefaultTimeoutSeconds);
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const TalonFXConfiguration& configs, units::time::second_t timeoutSeconds)
    {
        return SetConfigsPrivate(configs.Serialize(), timeoutSeconds, configs.FutureProofConfigs, false);
    }


    /**
     * \brief Refreshes the values of the specified config group.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(MotorOutputConfigs& configs) const
    {
        return Refresh(configs, DefaultTimeoutSeconds);
    }
    /**
     * \brief Refreshes the values of the specified config group.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(MotorOutputConfigs& configs, units::time::second_t timeoutSeconds) const
    {
        std::string ref;
        ctre::phoenix::StatusCode ret = GetConfigsPrivate(ref, timeoutSeconds);
        configs.Deserialize(ref);
        return ret;
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const MotorOutputConfigs& configs)
    {
        return Apply(configs, DefaultTimeoutSeconds);
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const MotorOutputConfigs& configs, units::time::second_t timeoutSeconds)
    {
        return SetConfigsPrivate(configs.Serialize(), timeoutSeconds, false, false);
    }

    /**
     * \brief Refreshes the values of the specified config group.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(CurrentLimitsConfigs& configs) const
    {
        return Refresh(configs, DefaultTimeoutSeconds);
    }
    /**
     * \brief Refreshes the values of the specified config group.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(CurrentLimitsConfigs& configs, units::time::second_t timeoutSeconds) const
    {
        std::string ref;
        ctre::phoenix::StatusCode ret = GetConfigsPrivate(ref, timeoutSeconds);
        configs.Deserialize(ref);
        return ret;
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const CurrentLimitsConfigs& configs)
    {
        return Apply(configs, DefaultTimeoutSeconds);
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const CurrentLimitsConfigs& configs, units::time::second_t timeoutSeconds)
    {
        return SetConfigsPrivate(configs.Serialize(), timeoutSeconds, false, false);
    }

    /**
     * \brief Refreshes the values of the specified config group.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(VoltageConfigs& configs) const
    {
        return Refresh(configs, DefaultTimeoutSeconds);
    }
    /**
     * \brief Refreshes the values of the specified config group.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(VoltageConfigs& configs, units::time::second_t timeoutSeconds) const
    {
        std::string ref;
        ctre::phoenix::StatusCode ret = GetConfigsPrivate(ref, timeoutSeconds);
        configs.Deserialize(ref);
        return ret;
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const VoltageConfigs& configs)
    {
        return Apply(configs, DefaultTimeoutSeconds);
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const VoltageConfigs& configs, units::time::second_t timeoutSeconds)
    {
        return SetConfigsPrivate(configs.Serialize(), timeoutSeconds, false, false);
    }

    /**
     * \brief Refreshes the values of the specified config group.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(TorqueCurrentConfigs& configs) const
    {
        return Refresh(configs, DefaultTimeoutSeconds);
    }
    /**
     * \brief Refreshes the values of the specified config group.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(TorqueCurrentConfigs& configs, units::time::second_t timeoutSeconds) const
    {
        std::string ref;
        ctre::phoenix::StatusCode ret = GetConfigsPrivate(ref, timeoutSeconds);
        configs.Deserialize(ref);
        return ret;
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const TorqueCurrentConfigs& configs)
    {
        return Apply(configs, DefaultTimeoutSeconds);
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const TorqueCurrentConfigs& configs, units::time::second_t timeoutSeconds)
    {
        return SetConfigsPrivate(configs.Serialize(), timeoutSeconds, false, false);
    }

    /**
     * \brief Refreshes the values of the specified config group.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(FeedbackConfigs& configs) const
    {
        return Refresh(configs, DefaultTimeoutSeconds);
    }
    /**
     * \brief Refreshes the values of the specified config group.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(FeedbackConfigs& configs, units::time::second_t timeoutSeconds) const
    {
        std::string ref;
        ctre::phoenix::StatusCode ret = GetConfigsPrivate(ref, timeoutSeconds);
        configs.Deserialize(ref);
        return ret;
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const FeedbackConfigs& configs)
    {
        return Apply(configs, DefaultTimeoutSeconds);
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const FeedbackConfigs& configs, units::time::second_t timeoutSeconds)
    {
        return SetConfigsPrivate(configs.Serialize(), timeoutSeconds, false, false);
    }

    /**
     * \brief Refreshes the values of the specified config group.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(DifferentialSensorsConfigs& configs) const
    {
        return Refresh(configs, DefaultTimeoutSeconds);
    }
    /**
     * \brief Refreshes the values of the specified config group.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(DifferentialSensorsConfigs& configs, units::time::second_t timeoutSeconds) const
    {
        std::string ref;
        ctre::phoenix::StatusCode ret = GetConfigsPrivate(ref, timeoutSeconds);
        configs.Deserialize(ref);
        return ret;
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const DifferentialSensorsConfigs& configs)
    {
        return Apply(configs, DefaultTimeoutSeconds);
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const DifferentialSensorsConfigs& configs, units::time::second_t timeoutSeconds)
    {
        return SetConfigsPrivate(configs.Serialize(), timeoutSeconds, false, false);
    }

    /**
     * \brief Refreshes the values of the specified config group.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(DifferentialConstantsConfigs& configs) const
    {
        return Refresh(configs, DefaultTimeoutSeconds);
    }
    /**
     * \brief Refreshes the values of the specified config group.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(DifferentialConstantsConfigs& configs, units::time::second_t timeoutSeconds) const
    {
        std::string ref;
        ctre::phoenix::StatusCode ret = GetConfigsPrivate(ref, timeoutSeconds);
        configs.Deserialize(ref);
        return ret;
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const DifferentialConstantsConfigs& configs)
    {
        return Apply(configs, DefaultTimeoutSeconds);
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const DifferentialConstantsConfigs& configs, units::time::second_t timeoutSeconds)
    {
        return SetConfigsPrivate(configs.Serialize(), timeoutSeconds, false, false);
    }

    /**
     * \brief Refreshes the values of the specified config group.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(OpenLoopRampsConfigs& configs) const
    {
        return Refresh(configs, DefaultTimeoutSeconds);
    }
    /**
     * \brief Refreshes the values of the specified config group.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(OpenLoopRampsConfigs& configs, units::time::second_t timeoutSeconds) const
    {
        std::string ref;
        ctre::phoenix::StatusCode ret = GetConfigsPrivate(ref, timeoutSeconds);
        configs.Deserialize(ref);
        return ret;
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const OpenLoopRampsConfigs& configs)
    {
        return Apply(configs, DefaultTimeoutSeconds);
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const OpenLoopRampsConfigs& configs, units::time::second_t timeoutSeconds)
    {
        return SetConfigsPrivate(configs.Serialize(), timeoutSeconds, false, false);
    }

    /**
     * \brief Refreshes the values of the specified config group.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(ClosedLoopRampsConfigs& configs) const
    {
        return Refresh(configs, DefaultTimeoutSeconds);
    }
    /**
     * \brief Refreshes the values of the specified config group.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(ClosedLoopRampsConfigs& configs, units::time::second_t timeoutSeconds) const
    {
        std::string ref;
        ctre::phoenix::StatusCode ret = GetConfigsPrivate(ref, timeoutSeconds);
        configs.Deserialize(ref);
        return ret;
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const ClosedLoopRampsConfigs& configs)
    {
        return Apply(configs, DefaultTimeoutSeconds);
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const ClosedLoopRampsConfigs& configs, units::time::second_t timeoutSeconds)
    {
        return SetConfigsPrivate(configs.Serialize(), timeoutSeconds, false, false);
    }

    /**
     * \brief Refreshes the values of the specified config group.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(HardwareLimitSwitchConfigs& configs) const
    {
        return Refresh(configs, DefaultTimeoutSeconds);
    }
    /**
     * \brief Refreshes the values of the specified config group.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(HardwareLimitSwitchConfigs& configs, units::time::second_t timeoutSeconds) const
    {
        std::string ref;
        ctre::phoenix::StatusCode ret = GetConfigsPrivate(ref, timeoutSeconds);
        configs.Deserialize(ref);
        return ret;
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const HardwareLimitSwitchConfigs& configs)
    {
        return Apply(configs, DefaultTimeoutSeconds);
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const HardwareLimitSwitchConfigs& configs, units::time::second_t timeoutSeconds)
    {
        return SetConfigsPrivate(configs.Serialize(), timeoutSeconds, false, false);
    }

    /**
     * \brief Refreshes the values of the specified config group.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(AudioConfigs& configs) const
    {
        return Refresh(configs, DefaultTimeoutSeconds);
    }
    /**
     * \brief Refreshes the values of the specified config group.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(AudioConfigs& configs, units::time::second_t timeoutSeconds) const
    {
        std::string ref;
        ctre::phoenix::StatusCode ret = GetConfigsPrivate(ref, timeoutSeconds);
        configs.Deserialize(ref);
        return ret;
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const AudioConfigs& configs)
    {
        return Apply(configs, DefaultTimeoutSeconds);
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const AudioConfigs& configs, units::time::second_t timeoutSeconds)
    {
        return SetConfigsPrivate(configs.Serialize(), timeoutSeconds, false, false);
    }

    /**
     * \brief Refreshes the values of the specified config group.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(SoftwareLimitSwitchConfigs& configs) const
    {
        return Refresh(configs, DefaultTimeoutSeconds);
    }
    /**
     * \brief Refreshes the values of the specified config group.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(SoftwareLimitSwitchConfigs& configs, units::time::second_t timeoutSeconds) const
    {
        std::string ref;
        ctre::phoenix::StatusCode ret = GetConfigsPrivate(ref, timeoutSeconds);
        configs.Deserialize(ref);
        return ret;
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const SoftwareLimitSwitchConfigs& configs)
    {
        return Apply(configs, DefaultTimeoutSeconds);
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const SoftwareLimitSwitchConfigs& configs, units::time::second_t timeoutSeconds)
    {
        return SetConfigsPrivate(configs.Serialize(), timeoutSeconds, false, false);
    }

    /**
     * \brief Refreshes the values of the specified config group.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(MotionMagicConfigs& configs) const
    {
        return Refresh(configs, DefaultTimeoutSeconds);
    }
    /**
     * \brief Refreshes the values of the specified config group.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(MotionMagicConfigs& configs, units::time::second_t timeoutSeconds) const
    {
        std::string ref;
        ctre::phoenix::StatusCode ret = GetConfigsPrivate(ref, timeoutSeconds);
        configs.Deserialize(ref);
        return ret;
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const MotionMagicConfigs& configs)
    {
        return Apply(configs, DefaultTimeoutSeconds);
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const MotionMagicConfigs& configs, units::time::second_t timeoutSeconds)
    {
        return SetConfigsPrivate(configs.Serialize(), timeoutSeconds, false, false);
    }

    /**
     * \brief Refreshes the values of the specified config group.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(CustomParamsConfigs& configs) const
    {
        return Refresh(configs, DefaultTimeoutSeconds);
    }
    /**
     * \brief Refreshes the values of the specified config group.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(CustomParamsConfigs& configs, units::time::second_t timeoutSeconds) const
    {
        std::string ref;
        ctre::phoenix::StatusCode ret = GetConfigsPrivate(ref, timeoutSeconds);
        configs.Deserialize(ref);
        return ret;
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const CustomParamsConfigs& configs)
    {
        return Apply(configs, DefaultTimeoutSeconds);
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const CustomParamsConfigs& configs, units::time::second_t timeoutSeconds)
    {
        return SetConfigsPrivate(configs.Serialize(), timeoutSeconds, false, false);
    }

    /**
     * \brief Refreshes the values of the specified config group.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(ClosedLoopGeneralConfigs& configs) const
    {
        return Refresh(configs, DefaultTimeoutSeconds);
    }
    /**
     * \brief Refreshes the values of the specified config group.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(ClosedLoopGeneralConfigs& configs, units::time::second_t timeoutSeconds) const
    {
        std::string ref;
        ctre::phoenix::StatusCode ret = GetConfigsPrivate(ref, timeoutSeconds);
        configs.Deserialize(ref);
        return ret;
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const ClosedLoopGeneralConfigs& configs)
    {
        return Apply(configs, DefaultTimeoutSeconds);
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const ClosedLoopGeneralConfigs& configs, units::time::second_t timeoutSeconds)
    {
        return SetConfigsPrivate(configs.Serialize(), timeoutSeconds, false, false);
    }

    /**
     * \brief Refreshes the values of the specified config group.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(Slot0Configs& configs) const
    {
        return Refresh(configs, DefaultTimeoutSeconds);
    }
    /**
     * \brief Refreshes the values of the specified config group.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(Slot0Configs& configs, units::time::second_t timeoutSeconds) const
    {
        std::string ref;
        ctre::phoenix::StatusCode ret = GetConfigsPrivate(ref, timeoutSeconds);
        configs.Deserialize(ref);
        return ret;
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const Slot0Configs& configs)
    {
        return Apply(configs, DefaultTimeoutSeconds);
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const Slot0Configs& configs, units::time::second_t timeoutSeconds)
    {
        return SetConfigsPrivate(configs.Serialize(), timeoutSeconds, false, false);
    }

    /**
     * \brief Refreshes the values of the specified config group.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(Slot1Configs& configs) const
    {
        return Refresh(configs, DefaultTimeoutSeconds);
    }
    /**
     * \brief Refreshes the values of the specified config group.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(Slot1Configs& configs, units::time::second_t timeoutSeconds) const
    {
        std::string ref;
        ctre::phoenix::StatusCode ret = GetConfigsPrivate(ref, timeoutSeconds);
        configs.Deserialize(ref);
        return ret;
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const Slot1Configs& configs)
    {
        return Apply(configs, DefaultTimeoutSeconds);
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const Slot1Configs& configs, units::time::second_t timeoutSeconds)
    {
        return SetConfigsPrivate(configs.Serialize(), timeoutSeconds, false, false);
    }

    /**
     * \brief Refreshes the values of the specified config group.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(Slot2Configs& configs) const
    {
        return Refresh(configs, DefaultTimeoutSeconds);
    }
    /**
     * \brief Refreshes the values of the specified config group.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(Slot2Configs& configs, units::time::second_t timeoutSeconds) const
    {
        std::string ref;
        ctre::phoenix::StatusCode ret = GetConfigsPrivate(ref, timeoutSeconds);
        configs.Deserialize(ref);
        return ret;
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const Slot2Configs& configs)
    {
        return Apply(configs, DefaultTimeoutSeconds);
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const Slot2Configs& configs, units::time::second_t timeoutSeconds)
    {
        return SetConfigsPrivate(configs.Serialize(), timeoutSeconds, false, false);
    }

    /**
     * \brief Refreshes the values of the specified config group.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(SlotConfigs& configs) const
    {
        return Refresh(configs, DefaultTimeoutSeconds);
    }
    /**
     * \brief Refreshes the values of the specified config group.
     *
     * \details Call to refresh the selected configs from the device.
     *
     * \param configs The configs to refresh
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of refreshing the configs
     */
    ctre::phoenix::StatusCode Refresh(SlotConfigs& configs, units::time::second_t timeoutSeconds) const
    {
        std::string ref;
        ctre::phoenix::StatusCode ret = GetConfigsPrivate(ref, timeoutSeconds);
        configs.Deserialize(ref);
        return ret;
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * This will wait up to #DefaultTimeoutSeconds.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const SlotConfigs& configs)
    {
        return Apply(configs, DefaultTimeoutSeconds);
    }

    /**
     * \brief Applies the contents of the specified config to the device.
     *
     * \details Call to apply the selected configs.
     *
     * \param configs Configs to apply against.
     * \param timeoutSeconds Maximum amount of time to wait when performing configuration
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode Apply(const SlotConfigs& configs, units::time::second_t timeoutSeconds)
    {
        return SetConfigsPrivate(configs.Serialize(), timeoutSeconds, false, false);
    }

    
    /**
     * \brief Sets the mechanism position of the device in mechanism
     * rotations.
     * 
     * This will wait up to #DefaultTimeoutSeconds.
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \param newValue Value to set to. Units are in rotations.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode SetPosition(units::angle::turn_t newValue)
    {
        return SetPosition(newValue, DefaultTimeoutSeconds);
    }
    /**
     * \brief Sets the mechanism position of the device in mechanism
     * rotations.
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \param newValue Value to set to. Units are in rotations.
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode SetPosition(units::angle::turn_t newValue, units::time::second_t timeoutSeconds)
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::TalonFX_SetSensorPosition, newValue.to<double>(), &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return SetConfigsPrivate(ss.str(), timeoutSeconds, false, true);
    }
    
    /**
     * \brief Clear the sticky faults in the device.
     * 
     * \details This typically has no impact on the device functionality. 
     * Instead, it just clears telemetry faults that are accessible via
     * API and Tuner Self-Test.
     * 
     * This will wait up to #DefaultTimeoutSeconds.
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFaults()
    {
        return ClearStickyFaults(DefaultTimeoutSeconds);
    }
    /**
     * \brief Clear the sticky faults in the device.
     * 
     * \details This typically has no impact on the device functionality. 
     * Instead, it just clears telemetry faults that are accessible via
     * API and Tuner Self-Test.
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFaults(units::time::second_t timeoutSeconds)
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::SPN_ClearStickyFaults, 0, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return SetConfigsPrivate(ss.str(), timeoutSeconds, false, true);
    }
    
    /**
     * \brief Clear sticky fault: Hardware fault occurred
     * 
     * This will wait up to #DefaultTimeoutSeconds.
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_Hardware()
    {
        return ClearStickyFault_Hardware(DefaultTimeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: Hardware fault occurred
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_Hardware(units::time::second_t timeoutSeconds)
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::ClearStickyFault_Hardware, 0, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return SetConfigsPrivate(ss.str(), timeoutSeconds, false, true);
    }
    
    /**
     * \brief Clear sticky fault: Processor temperature exceeded limit
     * 
     * This will wait up to #DefaultTimeoutSeconds.
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_ProcTemp()
    {
        return ClearStickyFault_ProcTemp(DefaultTimeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: Processor temperature exceeded limit
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_ProcTemp(units::time::second_t timeoutSeconds)
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::ClearStickyFault_ProcTemp, 0, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return SetConfigsPrivate(ss.str(), timeoutSeconds, false, true);
    }
    
    /**
     * \brief Clear sticky fault: Device temperature exceeded limit
     * 
     * This will wait up to #DefaultTimeoutSeconds.
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_DeviceTemp()
    {
        return ClearStickyFault_DeviceTemp(DefaultTimeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: Device temperature exceeded limit
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_DeviceTemp(units::time::second_t timeoutSeconds)
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::ClearStickyFault_DeviceTemp, 0, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return SetConfigsPrivate(ss.str(), timeoutSeconds, false, true);
    }
    
    /**
     * \brief Clear sticky fault: Device supply voltage dropped to near
     * brownout levels
     * 
     * This will wait up to #DefaultTimeoutSeconds.
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_Undervoltage()
    {
        return ClearStickyFault_Undervoltage(DefaultTimeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: Device supply voltage dropped to near
     * brownout levels
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_Undervoltage(units::time::second_t timeoutSeconds)
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::ClearStickyFault_Undervoltage, 0, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return SetConfigsPrivate(ss.str(), timeoutSeconds, false, true);
    }
    
    /**
     * \brief Clear sticky fault: Device boot while detecting the enable
     * signal
     * 
     * This will wait up to #DefaultTimeoutSeconds.
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_BootDuringEnable()
    {
        return ClearStickyFault_BootDuringEnable(DefaultTimeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: Device boot while detecting the enable
     * signal
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_BootDuringEnable(units::time::second_t timeoutSeconds)
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::ClearStickyFault_BootDuringEnable, 0, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return SetConfigsPrivate(ss.str(), timeoutSeconds, false, true);
    }
    
    /**
     * \brief Clear sticky fault: Bridge was disabled most likely due to
     * supply voltage dropping too low.
     * 
     * This will wait up to #DefaultTimeoutSeconds.
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_BridgeBrownout()
    {
        return ClearStickyFault_BridgeBrownout(DefaultTimeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: Bridge was disabled most likely due to
     * supply voltage dropping too low.
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_BridgeBrownout(units::time::second_t timeoutSeconds)
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::ClearStickyFault_TALONFX_BridgeBrownout, 0, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return SetConfigsPrivate(ss.str(), timeoutSeconds, false, true);
    }
    
    /**
     * \brief Clear sticky fault: The remote sensor has reset.
     * 
     * This will wait up to #DefaultTimeoutSeconds.
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_RemoteSensorReset()
    {
        return ClearStickyFault_RemoteSensorReset(DefaultTimeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: The remote sensor has reset.
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_RemoteSensorReset(units::time::second_t timeoutSeconds)
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::ClearStickyFault_TALONFX_RemoteSensorReset, 0, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return SetConfigsPrivate(ss.str(), timeoutSeconds, false, true);
    }
    
    /**
     * \brief Clear sticky fault: The remote Talon FX used for
     * differential control is not present on CAN Bus.
     * 
     * This will wait up to #DefaultTimeoutSeconds.
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_MissingDifferentialFX()
    {
        return ClearStickyFault_MissingDifferentialFX(DefaultTimeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: The remote Talon FX used for
     * differential control is not present on CAN Bus.
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_MissingDifferentialFX(units::time::second_t timeoutSeconds)
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::ClearStickyFault_TALONFX_MissingDifferentialFX, 0, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return SetConfigsPrivate(ss.str(), timeoutSeconds, false, true);
    }
    
    /**
     * \brief Clear sticky fault: The remote sensor position has
     * overflowed. Because of the nature of remote sensors, it is possible
     * for the remote sensor position to overflow beyond what is supported
     * by the status signal frame. However, this is rare and cannot occur
     * over the course of an FRC match under normal use.
     * 
     * This will wait up to #DefaultTimeoutSeconds.
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_RemoteSensorPosOverflow()
    {
        return ClearStickyFault_RemoteSensorPosOverflow(DefaultTimeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: The remote sensor position has
     * overflowed. Because of the nature of remote sensors, it is possible
     * for the remote sensor position to overflow beyond what is supported
     * by the status signal frame. However, this is rare and cannot occur
     * over the course of an FRC match under normal use.
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_RemoteSensorPosOverflow(units::time::second_t timeoutSeconds)
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::ClearStickyFault_TALONFX_RemoteSensorPosOverflow, 0, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return SetConfigsPrivate(ss.str(), timeoutSeconds, false, true);
    }
    
    /**
     * \brief Clear sticky fault: Supply Voltage has exceeded the maximum
     * voltage rating of device.
     * 
     * This will wait up to #DefaultTimeoutSeconds.
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_OverSupplyV()
    {
        return ClearStickyFault_OverSupplyV(DefaultTimeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: Supply Voltage has exceeded the maximum
     * voltage rating of device.
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_OverSupplyV(units::time::second_t timeoutSeconds)
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::ClearStickyFault_TALONFX_OverSupplyV, 0, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return SetConfigsPrivate(ss.str(), timeoutSeconds, false, true);
    }
    
    /**
     * \brief Clear sticky fault: Supply Voltage is unstable.  Ensure you
     * are using a battery and current limited power supply.
     * 
     * This will wait up to #DefaultTimeoutSeconds.
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_UnstableSupplyV()
    {
        return ClearStickyFault_UnstableSupplyV(DefaultTimeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: Supply Voltage is unstable.  Ensure you
     * are using a battery and current limited power supply.
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_UnstableSupplyV(units::time::second_t timeoutSeconds)
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::ClearStickyFault_TALONFX_UnstableSupplyV, 0, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return SetConfigsPrivate(ss.str(), timeoutSeconds, false, true);
    }
    
    /**
     * \brief Clear sticky fault: Reverse limit switch has been asserted. 
     * Output is set to neutral.
     * 
     * This will wait up to #DefaultTimeoutSeconds.
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_ReverseHardLimit()
    {
        return ClearStickyFault_ReverseHardLimit(DefaultTimeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: Reverse limit switch has been asserted. 
     * Output is set to neutral.
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_ReverseHardLimit(units::time::second_t timeoutSeconds)
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::ClearStickyFault_TALONFX_ReverseHardLimit, 0, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return SetConfigsPrivate(ss.str(), timeoutSeconds, false, true);
    }
    
    /**
     * \brief Clear sticky fault: Forward limit switch has been asserted. 
     * Output is set to neutral.
     * 
     * This will wait up to #DefaultTimeoutSeconds.
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_ForwardHardLimit()
    {
        return ClearStickyFault_ForwardHardLimit(DefaultTimeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: Forward limit switch has been asserted. 
     * Output is set to neutral.
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_ForwardHardLimit(units::time::second_t timeoutSeconds)
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::ClearStickyFault_TALONFX_ForwardHardLimit, 0, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return SetConfigsPrivate(ss.str(), timeoutSeconds, false, true);
    }
    
    /**
     * \brief Clear sticky fault: Reverse soft limit has been asserted. 
     * Output is set to neutral.
     * 
     * This will wait up to #DefaultTimeoutSeconds.
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_ReverseSoftLimit()
    {
        return ClearStickyFault_ReverseSoftLimit(DefaultTimeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: Reverse soft limit has been asserted. 
     * Output is set to neutral.
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_ReverseSoftLimit(units::time::second_t timeoutSeconds)
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::ClearStickyFault_TALONFX_ReverseSoftLimit, 0, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return SetConfigsPrivate(ss.str(), timeoutSeconds, false, true);
    }
    
    /**
     * \brief Clear sticky fault: Forward soft limit has been asserted. 
     * Output is set to neutral.
     * 
     * This will wait up to #DefaultTimeoutSeconds.
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_ForwardSoftLimit()
    {
        return ClearStickyFault_ForwardSoftLimit(DefaultTimeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: Forward soft limit has been asserted. 
     * Output is set to neutral.
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_ForwardSoftLimit(units::time::second_t timeoutSeconds)
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::ClearStickyFault_TALONFX_ForwardSoftLimit, 0, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return SetConfigsPrivate(ss.str(), timeoutSeconds, false, true);
    }
    
    /**
     * \brief Clear sticky fault: The remote sensor's data is no longer
     * trusted. This can happen if the remote sensor disappears from the
     * CAN bus or if the remote sensor indicates its data is no longer
     * valid, such as when a CANcoder's magnet strength falls into the
     * "red" range.
     * 
     * This will wait up to #DefaultTimeoutSeconds.
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_RemoteSensorDataInvalid()
    {
        return ClearStickyFault_RemoteSensorDataInvalid(DefaultTimeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: The remote sensor's data is no longer
     * trusted. This can happen if the remote sensor disappears from the
     * CAN bus or if the remote sensor indicates its data is no longer
     * valid, such as when a CANcoder's magnet strength falls into the
     * "red" range.
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_RemoteSensorDataInvalid(units::time::second_t timeoutSeconds)
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::ClearStickyFault_TALONFX_MissingRemoteSensor, 0, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return SetConfigsPrivate(ss.str(), timeoutSeconds, false, true);
    }
    
    /**
     * \brief Clear sticky fault: The remote sensor used for fusion has
     * fallen out of sync to the local sensor. A re-synchronization has
     * occurred, which may cause a discontinuity. This typically happens
     * if there is significant slop in the mechanism, or if the
     * RotorToSensorRatio configuration parameter is incorrect.
     * 
     * This will wait up to #DefaultTimeoutSeconds.
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_FusedSensorOutOfSync()
    {
        return ClearStickyFault_FusedSensorOutOfSync(DefaultTimeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: The remote sensor used for fusion has
     * fallen out of sync to the local sensor. A re-synchronization has
     * occurred, which may cause a discontinuity. This typically happens
     * if there is significant slop in the mechanism, or if the
     * RotorToSensorRatio configuration parameter is incorrect.
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_FusedSensorOutOfSync(units::time::second_t timeoutSeconds)
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::ClearStickyFault_TALONFX_FusedSensorOutOfSync, 0, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return SetConfigsPrivate(ss.str(), timeoutSeconds, false, true);
    }
    
    /**
     * \brief Clear sticky fault: Stator current limit occured.
     * 
     * This will wait up to #DefaultTimeoutSeconds.
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_StatorCurrLimit()
    {
        return ClearStickyFault_StatorCurrLimit(DefaultTimeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: Stator current limit occured.
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_StatorCurrLimit(units::time::second_t timeoutSeconds)
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::ClearStickyFault_TALONFX_StatorCurrLimit, 0, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return SetConfigsPrivate(ss.str(), timeoutSeconds, false, true);
    }
    
    /**
     * \brief Clear sticky fault: Supply current limit occured.
     * 
     * This will wait up to #DefaultTimeoutSeconds.
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_SupplyCurrLimit()
    {
        return ClearStickyFault_SupplyCurrLimit(DefaultTimeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: Supply current limit occured.
     * 
     * This is available in the configurator in case the user wants
     * to initialize their device entirely without passing a device
     * reference down to the code that performs the initialization.
     * In this case, the user passes down the configurator object
     * and performs all the initialization code on the object.
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_SupplyCurrLimit(units::time::second_t timeoutSeconds)
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::ClearStickyFault_TALONFX_SupplyCurrLimit, 0, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return SetConfigsPrivate(ss.str(), timeoutSeconds, false, true);
    }
};

}

namespace hardware {
namespace core {

/**
 * Class description for the Talon FX integrated motor controller.
 */
class CoreTalonFX : public ParentDevice
{
private:
    configs::TalonFXConfigurator _configs;

    
    /**
     * \brief Proportional output of PID controller when PID'ing under a
     * DutyCycle Request
     * 
     *   Minimum Value: -128.0
     *   Maximum Value: 127.9990234375
     *   Default Value: 0
     *   Units: fractional
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns PIDDutyCycle_ProportionalOutput Status Signal Object
     */
    StatusSignal<units::dimensionless::scalar_t> &GetPIDDutyCycle_ProportionalOutput();
    
    /**
     * \brief Proportional output of PID controller when PID'ing under a
     * Voltage Request
     * 
     *   Minimum Value: -1310.72
     *   Maximum Value: 1310.71
     *   Default Value: 0
     *   Units: V
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns PIDMotorVoltage_ProportionalOutput Status Signal Object
     */
    StatusSignal<units::voltage::volt_t> &GetPIDMotorVoltage_ProportionalOutput();
    
    /**
     * \brief Proportional output of PID controller when PID'ing under a
     * TorqueCurrent Request
     * 
     *   Minimum Value: -13107.2
     *   Maximum Value: 13107.1
     *   Default Value: 0
     *   Units: A
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns PIDTorqueCurrent_ProportionalOutput Status Signal Object
     */
    StatusSignal<units::current::ampere_t> &GetPIDTorqueCurrent_ProportionalOutput();
    
    /**
     * \brief Integrated Accumulator of PID controller when PID'ing under
     * a DutyCycle Request
     * 
     *   Minimum Value: -128.0
     *   Maximum Value: 127.9990234375
     *   Default Value: 0
     *   Units: fractional
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns PIDDutyCycle_IntegratedAccum Status Signal Object
     */
    StatusSignal<units::dimensionless::scalar_t> &GetPIDDutyCycle_IntegratedAccum();
    
    /**
     * \brief Integrated Accumulator of PID controller when PID'ing under
     * a Voltage Request
     * 
     *   Minimum Value: -1310.72
     *   Maximum Value: 1310.71
     *   Default Value: 0
     *   Units: V
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns PIDMotorVoltage_IntegratedAccum Status Signal Object
     */
    StatusSignal<units::voltage::volt_t> &GetPIDMotorVoltage_IntegratedAccum();
    
    /**
     * \brief Integrated Accumulator of PID controller when PID'ing under
     * a TorqueCurrent Request
     * 
     *   Minimum Value: -13107.2
     *   Maximum Value: 13107.1
     *   Default Value: 0
     *   Units: A
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns PIDTorqueCurrent_IntegratedAccum Status Signal Object
     */
    StatusSignal<units::current::ampere_t> &GetPIDTorqueCurrent_IntegratedAccum();
    
    /**
     * \brief Feedforward passed to PID controller
     * 
     *   Minimum Value: -2.0
     *   Maximum Value: 1.9990234375
     *   Default Value: 0
     *   Units: fractional
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns PIDDutyCycle_FeedForward Status Signal Object
     */
    StatusSignal<units::dimensionless::scalar_t> &GetPIDDutyCycle_FeedForward();
    
    /**
     * \brief Feedforward passed to PID controller
     * 
     *   Minimum Value: -20.48
     *   Maximum Value: 20.47
     *   Default Value: 0
     *   Units: V
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns PIDMotorVoltage_FeedForward Status Signal Object
     */
    StatusSignal<units::voltage::volt_t> &GetPIDMotorVoltage_FeedForward();
    
    /**
     * \brief Feedforward passed to PID controller
     * 
     *   Minimum Value: -409.6
     *   Maximum Value: 409.40000000000003
     *   Default Value: 0
     *   Units: A
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns PIDTorqueCurrent_FeedForward Status Signal Object
     */
    StatusSignal<units::current::ampere_t> &GetPIDTorqueCurrent_FeedForward();
    
    /**
     * \brief Derivative Output of PID controller when PID'ing under a
     * DutyCycle Request
     * 
     *   Minimum Value: -128.0
     *   Maximum Value: 127.9990234375
     *   Default Value: 0
     *   Units: fractional
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns PIDDutyCycle_DerivativeOutput Status Signal Object
     */
    StatusSignal<units::dimensionless::scalar_t> &GetPIDDutyCycle_DerivativeOutput();
    
    /**
     * \brief Derivative Output of PID controller when PID'ing under a
     * Voltage Request
     * 
     *   Minimum Value: -1310.72
     *   Maximum Value: 1310.71
     *   Default Value: 0
     *   Units: V
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns PIDMotorVoltage_DerivativeOutput Status Signal Object
     */
    StatusSignal<units::voltage::volt_t> &GetPIDMotorVoltage_DerivativeOutput();
    
    /**
     * \brief Derivative Output of PID controller when PID'ing under a
     * TorqueCurrent Request
     * 
     *   Minimum Value: -13107.2
     *   Maximum Value: 13107.1
     *   Default Value: 0
     *   Units: A
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns PIDTorqueCurrent_DerivativeOutput Status Signal Object
     */
    StatusSignal<units::current::ampere_t> &GetPIDTorqueCurrent_DerivativeOutput();
    
    /**
     * \brief Output of PID controller when PID'ing under a DutyCycle
     * Request
     * 
     *   Minimum Value: -128.0
     *   Maximum Value: 127.9990234375
     *   Default Value: 0
     *   Units: fractional
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns PIDDutyCycle_Output Status Signal Object
     */
    StatusSignal<units::dimensionless::scalar_t> &GetPIDDutyCycle_Output();
    
    /**
     * \brief Output of PID controller when PID'ing under a Voltage
     * Request
     * 
     *   Minimum Value: -1310.72
     *   Maximum Value: 1310.71
     *   Default Value: 0
     *   Units: V
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns PIDMotorVoltage_Output Status Signal Object
     */
    StatusSignal<units::voltage::volt_t> &GetPIDMotorVoltage_Output();
    
    /**
     * \brief Output of PID controller when PID'ing under a TorqueCurrent
     * Request
     * 
     *   Minimum Value: -13107.2
     *   Maximum Value: 13107.1
     *   Default Value: 0
     *   Units: A
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns PIDTorqueCurrent_Output Status Signal Object
     */
    StatusSignal<units::current::ampere_t> &GetPIDTorqueCurrent_Output();
    
    /**
     * \brief Input position of PID controller when PID'ing to a position
     * 
     *   Minimum Value: -10000
     *   Maximum Value: 10000
     *   Default Value: 0
     *   Units: rotations
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns PIDPosition_Reference Status Signal Object
     */
    StatusSignal<units::angle::turn_t> &GetPIDPosition_Reference();
    
    /**
     * \brief Input velocity of PID controller when PID'ing to a velocity
     * 
     *   Minimum Value: -10000
     *   Maximum Value: 10000
     *   Default Value: 0
     *   Units: rotations per second
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns PIDVelocity_Reference Status Signal Object
     */
    StatusSignal<units::angular_velocity::turns_per_second_t> &GetPIDVelocity_Reference();
    
    /**
     * \brief Change in input (velocity) of PID controller when PID'ing to
     * a position
     * 
     *   Minimum Value: -512.0
     *   Maximum Value: 511.984375
     *   Default Value: 0
     *   Units: rotations per second
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns PIDPosition_ReferenceSlope Status Signal Object
     */
    StatusSignal<units::angular_velocity::turns_per_second_t> &GetPIDPosition_ReferenceSlope();
    
    /**
     * \brief Change in input (acceleration) of PID controller when
     * PID'ing to a velocity
     * 
     *   Minimum Value: -512.0
     *   Maximum Value: 511.984375
     *   Default Value: 0
     *   Units: rotations per second²
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns PIDVelocity_ReferenceSlope Status Signal Object
     */
    StatusSignal<units::angular_acceleration::turns_per_second_squared_t> &GetPIDVelocity_ReferenceSlope();
    
    /**
     * \brief The difference between target position and current position
     * 
     *   Minimum Value: -10000
     *   Maximum Value: 10000
     *   Default Value: 0
     *   Units: rotations
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns PIDPosition_ClosedLoopError Status Signal Object
     */
    StatusSignal<units::angle::turn_t> &GetPIDPosition_ClosedLoopError();
    
    /**
     * \brief The difference between target velocity and current velocity
     * 
     *   Minimum Value: -10000
     *   Maximum Value: 10000
     *   Default Value: 0
     *   Units: rotations per second
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns PIDVelocity_ClosedLoopError Status Signal Object
     */
    StatusSignal<units::angular_velocity::turns_per_second_t> &GetPIDVelocity_ClosedLoopError();
    
    /**
     * \brief The calculated motor duty cycle for differential followers.
     * 
     *   Minimum Value: -32.0
     *   Maximum Value: 31.9990234375
     *   Default Value: 0
     *   Units: fractional
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns DifferentialDutyCycle Status Signal Object
     */
    StatusSignal<units::dimensionless::scalar_t> &GetDifferentialDutyCycle();
    
    /**
     * \brief The calculated motor torque current for differential
     * followers.
     * 
     *   Minimum Value: -327.68
     *   Maximum Value: 327.67
     *   Default Value: 0
     *   Units: A
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns DifferentialTorqueCurrent Status Signal Object
     */
    StatusSignal<units::current::ampere_t> &GetDifferentialTorqueCurrent();
    
    /**
     * \brief Proportional output of differential PID controller when
     * PID'ing under a DutyCycle Request
     * 
     *   Minimum Value: -128.0
     *   Maximum Value: 127.9990234375
     *   Default Value: 0
     *   Units: fractional
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns DifferentialPIDDutyCycle_ProportionalOutput Status Signal Object
     */
    StatusSignal<units::dimensionless::scalar_t> &GetDifferentialPIDDutyCycle_ProportionalOutput();
    
    /**
     * \brief Proportional output of differential PID controller when
     * PID'ing under a Voltage Request
     * 
     *   Minimum Value: -1310.72
     *   Maximum Value: 1310.71
     *   Default Value: 0
     *   Units: V
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns DifferentialPIDMotorVoltage_ProportionalOutput Status Signal Object
     */
    StatusSignal<units::voltage::volt_t> &GetDifferentialPIDMotorVoltage_ProportionalOutput();
    
    /**
     * \brief Proportional output of differential PID controller when
     * PID'ing under a TorqueCurrent Request
     * 
     *   Minimum Value: -13107.2
     *   Maximum Value: 13107.1
     *   Default Value: 0
     *   Units: A
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns DifferentialPIDTorqueCurrent_ProportionalOutput Status Signal Object
     */
    StatusSignal<units::current::ampere_t> &GetDifferentialPIDTorqueCurrent_ProportionalOutput();
    
    /**
     * \brief Integrated Accumulator of differential PID controller when
     * PID'ing under a DutyCycle Request
     * 
     *   Minimum Value: -128.0
     *   Maximum Value: 127.9990234375
     *   Default Value: 0
     *   Units: fractional
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns DifferentialPIDDutyCycle_IntegratedAccum Status Signal Object
     */
    StatusSignal<units::dimensionless::scalar_t> &GetDifferentialPIDDutyCycle_IntegratedAccum();
    
    /**
     * \brief Integrated Accumulator of differential PID controller when
     * PID'ing under a Voltage Request
     * 
     *   Minimum Value: -1310.72
     *   Maximum Value: 1310.71
     *   Default Value: 0
     *   Units: V
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns DifferentialPIDMotorVoltage_IntegratedAccum Status Signal Object
     */
    StatusSignal<units::voltage::volt_t> &GetDifferentialPIDMotorVoltage_IntegratedAccum();
    
    /**
     * \brief Integrated Accumulator of differential PID controller when
     * PID'ing under a TorqueCurrent Request
     * 
     *   Minimum Value: -13107.2
     *   Maximum Value: 13107.1
     *   Default Value: 0
     *   Units: A
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns DifferentialPIDTorqueCurrent_IntegratedAccum Status Signal Object
     */
    StatusSignal<units::current::ampere_t> &GetDifferentialPIDTorqueCurrent_IntegratedAccum();
    
    /**
     * \brief Feedforward passed to differential PID controller
     * 
     *   Minimum Value: -2.0
     *   Maximum Value: 1.9990234375
     *   Default Value: 0
     *   Units: fractional
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns DifferentialPIDDutyCycle_FeedForward Status Signal Object
     */
    StatusSignal<units::dimensionless::scalar_t> &GetDifferentialPIDDutyCycle_FeedForward();
    
    /**
     * \brief Feedforward passed to differential PID controller
     * 
     *   Minimum Value: -20.48
     *   Maximum Value: 20.47
     *   Default Value: 0
     *   Units: V
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns DifferentialPIDMotorVoltage_FeedForward Status Signal Object
     */
    StatusSignal<units::voltage::volt_t> &GetDifferentialPIDMotorVoltage_FeedForward();
    
    /**
     * \brief Feedforward passed to differential PID controller
     * 
     *   Minimum Value: -409.6
     *   Maximum Value: 409.40000000000003
     *   Default Value: 0
     *   Units: A
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns DifferentialPIDTorqueCurrent_FeedForward Status Signal Object
     */
    StatusSignal<units::current::ampere_t> &GetDifferentialPIDTorqueCurrent_FeedForward();
    
    /**
     * \brief Derivative Output of differential PID controller when
     * PID'ing under a DutyCycle Request
     * 
     *   Minimum Value: -128.0
     *   Maximum Value: 127.9990234375
     *   Default Value: 0
     *   Units: fractional
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns DifferentialPIDDutyCycle_DerivativeOutput Status Signal Object
     */
    StatusSignal<units::dimensionless::scalar_t> &GetDifferentialPIDDutyCycle_DerivativeOutput();
    
    /**
     * \brief Derivative Output of differential PID controller when
     * PID'ing under a Voltage Request
     * 
     *   Minimum Value: -1310.72
     *   Maximum Value: 1310.71
     *   Default Value: 0
     *   Units: V
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns DiffPIDMotorVoltage_DerivativeOutput Status Signal Object
     */
    StatusSignal<units::voltage::volt_t> &GetDiffPIDMotorVoltage_DerivativeOutput();
    
    /**
     * \brief Derivative Output of differential PID controller when
     * PID'ing under a TorqueCurrent Request
     * 
     *   Minimum Value: -13107.2
     *   Maximum Value: 13107.1
     *   Default Value: 0
     *   Units: A
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns DifferentialPIDTorqueCurrent_DerivativeOutput Status Signal Object
     */
    StatusSignal<units::current::ampere_t> &GetDifferentialPIDTorqueCurrent_DerivativeOutput();
    
    /**
     * \brief Output of differential PID controller when PID'ing under a
     * DutyCycle Request
     * 
     *   Minimum Value: -128.0
     *   Maximum Value: 127.9990234375
     *   Default Value: 0
     *   Units: fractional
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns DifferentialPIDDutyCycle_Output Status Signal Object
     */
    StatusSignal<units::dimensionless::scalar_t> &GetDifferentialPIDDutyCycle_Output();
    
    /**
     * \brief Output of differential PID controller when PID'ing under a
     * Voltage Request
     * 
     *   Minimum Value: -1310.72
     *   Maximum Value: 1310.71
     *   Default Value: 0
     *   Units: V
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns DifferentialPIDMotorVoltage_Output Status Signal Object
     */
    StatusSignal<units::voltage::volt_t> &GetDifferentialPIDMotorVoltage_Output();
    
    /**
     * \brief Output of differential PID controller when PID'ing under a
     * TorqueCurrent Request
     * 
     *   Minimum Value: -13107.2
     *   Maximum Value: 13107.1
     *   Default Value: 0
     *   Units: A
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns DifferentialPIDTorqueCurrent_Output Status Signal Object
     */
    StatusSignal<units::current::ampere_t> &GetDifferentialPIDTorqueCurrent_Output();
    
    /**
     * \brief Input position of differential PID controller when PID'ing
     * to a differential position
     * 
     *   Minimum Value: -10000
     *   Maximum Value: 10000
     *   Default Value: 0
     *   Units: rotations
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns DifferentialPIDPosition_Reference Status Signal Object
     */
    StatusSignal<units::angle::turn_t> &GetDifferentialPIDPosition_Reference();
    
    /**
     * \brief Input velocity of differential PID controller when PID'ing
     * to a differential velocity
     * 
     *   Minimum Value: -10000
     *   Maximum Value: 10000
     *   Default Value: 0
     *   Units: rotations per second
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns DifferentialPIDVelocity_Reference Status Signal Object
     */
    StatusSignal<units::angular_velocity::turns_per_second_t> &GetDifferentialPIDVelocity_Reference();
    
    /**
     * \brief Change in input (velocity) of differential PID controller
     * when PID'ing to a differential position
     * 
     *   Minimum Value: -512.0
     *   Maximum Value: 511.984375
     *   Default Value: 0
     *   Units: rotations per second
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns DifferentialPIDPosition_ReferenceSlope Status Signal Object
     */
    StatusSignal<units::angular_velocity::turns_per_second_t> &GetDifferentialPIDPosition_ReferenceSlope();
    
    /**
     * \brief Change in input (acceleration) of differential PID
     * controller when PID'ing to a differential velocity
     * 
     *   Minimum Value: -512.0
     *   Maximum Value: 511.984375
     *   Default Value: 0
     *   Units: rotations per second²
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns DifferentialPIDVelocity_ReferenceSlope Status Signal Object
     */
    StatusSignal<units::angular_acceleration::turns_per_second_squared_t> &GetDifferentialPIDVelocity_ReferenceSlope();
    
    /**
     * \brief The difference between target differential position and
     * current differential position
     * 
     *   Minimum Value: -10000
     *   Maximum Value: 10000
     *   Default Value: 0
     *   Units: rotations
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns DifferentialPIDPosition_ClosedLoopError Status Signal Object
     */
    StatusSignal<units::angle::turn_t> &GetDifferentialPIDPosition_ClosedLoopError();
    
    /**
     * \brief The difference between target differential velocity and
     * current differential velocity
     * 
     *   Minimum Value: -10000
     *   Maximum Value: 10000
     *   Default Value: 0
     *   Units: rotations per second
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns DifferentialPIDVelocity_ClosedLoopError Status Signal Object
     */
    StatusSignal<units::angular_velocity::turns_per_second_t> &GetDifferentialPIDVelocity_ClosedLoopError();
public:
    /**
     * Constructs a new Talon FX motor controller object.
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
    CoreTalonFX(int deviceId, std::string canbus = "");

    CoreTalonFX(CoreTalonFX const &) = delete;
    CoreTalonFX &operator=(CoreTalonFX const &) = delete;

    /**
     * \brief Gets the configurator for this TalonFX
     *
     * \details Gets the configurator for this TalonFX
     *
     * \returns Configurator for this TalonFX
     */
    configs::TalonFXConfigurator &GetConfigurator()
    {
        return _configs;
    }

    /**
     * \brief Gets the configurator for this TalonFX
     *
     * \details Gets the configurator for this TalonFX
     *
     * \returns Configurator for this TalonFX
     */
    configs::TalonFXConfigurator const &GetConfigurator() const
    {
        return _configs;
    }


private:
    std::unique_ptr<sim::TalonFXSimState> _simState{};
public:
    /**
     * \brief Get the simulation state for this device.
     *
     * \details This function reuses an allocated simulation
     * state object, so it is safe to call this function multiple
     * times in a robot loop.
     *
     * \returns Simulation state
     */
    sim::TalonFXSimState &GetSimState()
    {
        if (_simState == nullptr)
            _simState = std::make_unique<sim::TalonFXSimState>(*this);
        return *_simState;
    }


        
    /**
     * \brief App Major Version number.
     * 
     *   Minimum Value: 0
     *   Maximum Value: 255
     *   Default Value: 0
     *   Units: 
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns VersionMajor Status Signal Object
     */
    StatusSignal<int> &GetVersionMajor();
        
    /**
     * \brief App Minor Version number.
     * 
     *   Minimum Value: 0
     *   Maximum Value: 255
     *   Default Value: 0
     *   Units: 
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns VersionMinor Status Signal Object
     */
    StatusSignal<int> &GetVersionMinor();
        
    /**
     * \brief App Bugfix Version number.
     * 
     *   Minimum Value: 0
     *   Maximum Value: 255
     *   Default Value: 0
     *   Units: 
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns VersionBugfix Status Signal Object
     */
    StatusSignal<int> &GetVersionBugfix();
        
    /**
     * \brief App Build Version number.
     * 
     *   Minimum Value: 0
     *   Maximum Value: 255
     *   Default Value: 0
     *   Units: 
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns VersionBuild Status Signal Object
     */
    StatusSignal<int> &GetVersionBuild();
        
    /**
     * \brief Full Version.  The format is a four byte value.
     * 
     * \details Full Version of firmware in device. The format is a four
     * byte value.
     * 
     *   Minimum Value: 0
     *   Maximum Value: 4294967295
     *   Default Value: 0
     *   Units: 
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns Version Status Signal Object
     */
    StatusSignal<int> &GetVersion();
        
    /**
     * \brief Integer representing all faults
     * 
     * \details This returns the fault flags reported by the device. These
     * are device specific and are not used directly in typical
     * applications. Use the signal specific GetFault_*() methods instead.
     *  
     * 
     *   Minimum Value: 0
     *   Maximum Value: 16777215
     *   Default Value: 0
     *   Units: 
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns FaultField Status Signal Object
     */
    StatusSignal<int> &GetFaultField();
        
    /**
     * \brief Integer representing all sticky faults
     * 
     * \details This returns the persistent "sticky" fault flags reported
     * by the device. These are device specific and are not used directly
     * in typical applications. Use the signal specific GetStickyFault_*()
     * methods instead.  
     * 
     *   Minimum Value: 0
     *   Maximum Value: 16777215
     *   Default Value: 0
     *   Units: 
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns StickyFaultField Status Signal Object
     */
    StatusSignal<int> &GetStickyFaultField();
        
    /**
     * \brief The applied (output) motor voltage.
     * 
     *   Minimum Value: -40.96
     *   Maximum Value: 40.95
     *   Default Value: 0
     *   Units: V
     * 
     * Default Rates:
     *   CAN 2.0: 100.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns MotorVoltage Status Signal Object
     */
    StatusSignal<units::voltage::volt_t> &GetMotorVoltage();
        
    /**
     * \brief Forward Limit Pin.
     * 
     * 
     * Default Rates:
     *   CAN 2.0: 100.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns ForwardLimit Status Signal Object
     */
    StatusSignal<signals::ForwardLimitValue> &GetForwardLimit();
        
    /**
     * \brief Reverse Limit Pin.
     * 
     * 
     * Default Rates:
     *   CAN 2.0: 100.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns ReverseLimit Status Signal Object
     */
    StatusSignal<signals::ReverseLimitValue> &GetReverseLimit();
        
    /**
     * \brief The applied rotor polarity.  This typically is determined by
     * the Inverted config, but can be overridden if using Follower
     * features.
     * 
     * 
     * Default Rates:
     *   CAN 2.0: 100.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns AppliedRotorPolarity Status Signal Object
     */
    StatusSignal<signals::AppliedRotorPolarityValue> &GetAppliedRotorPolarity();
        
    /**
     * \brief The applied motor duty cycle.
     * 
     *   Minimum Value: -2.0
     *   Maximum Value: 1.9990234375
     *   Default Value: 0
     *   Units: fractional
     * 
     * Default Rates:
     *   CAN 2.0: 100.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns DutyCycle Status Signal Object
     */
    StatusSignal<units::dimensionless::scalar_t> &GetDutyCycle();
        
    /**
     * \brief Current corresponding to the torque output by the motor.
     * Similar to StatorCurrent. Users will likely prefer this current to
     * calculate the applied torque to the rotor.
     * 
     * \details Stator current where positive current means torque is
     * applied in the forward direction as determined by the Inverted
     * setting
     * 
     *   Minimum Value: -327.68
     *   Maximum Value: 327.67
     *   Default Value: 0
     *   Units: A
     * 
     * Default Rates:
     *   CAN 2.0: 100.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns TorqueCurrent Status Signal Object
     */
    StatusSignal<units::current::ampere_t> &GetTorqueCurrent();
        
    /**
     * \brief Current corresponding to the stator windings. Similar to
     * TorqueCurrent. Users will likely prefer TorqueCurrent over
     * StatorCurrent.
     * 
     * \details Stator current where Positive current indicates motoring
     * regardless of direction. Negative current indicates regenerative
     * braking regardless of direction.
     * 
     *   Minimum Value: -327.68
     *   Maximum Value: 327.66
     *   Default Value: 0
     *   Units: A
     * 
     * Default Rates:
     *   CAN 2.0: 4.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns StatorCurrent Status Signal Object
     */
    StatusSignal<units::current::ampere_t> &GetStatorCurrent();
        
    /**
     * \brief Measured supply side current
     * 
     *   Minimum Value: -327.68
     *   Maximum Value: 327.66
     *   Default Value: 0
     *   Units: A
     * 
     * Default Rates:
     *   CAN 2.0: 4.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns SupplyCurrent Status Signal Object
     */
    StatusSignal<units::current::ampere_t> &GetSupplyCurrent();
        
    /**
     * \brief Measured supply voltage to the TalonFX.
     * 
     *   Minimum Value: 4
     *   Maximum Value: 29.575
     *   Default Value: 4
     *   Units: V
     * 
     * Default Rates:
     *   CAN 2.0: 4.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns SupplyVoltage Status Signal Object
     */
    StatusSignal<units::voltage::volt_t> &GetSupplyVoltage();
        
    /**
     * \brief Temperature of device
     * 
     * \details This is the temperature that the device measures itself to
     * be at. Similar to Processor Temperature.
     * 
     *   Minimum Value: 0.0
     *   Maximum Value: 255.0
     *   Default Value: 0
     *   Units: ℃
     * 
     * Default Rates:
     *   CAN 2.0: 4.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns DeviceTemp Status Signal Object
     */
    StatusSignal<units::temperature::celsius_t> &GetDeviceTemp();
        
    /**
     * \brief Temperature of the processor
     * 
     * \details This is the temperature that the processor measures itself
     * to be at. Similar to Device Temperature.
     * 
     *   Minimum Value: 0.0
     *   Maximum Value: 255.0
     *   Default Value: 0
     *   Units: ℃
     * 
     * Default Rates:
     *   CAN 2.0: 4.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns ProcessorTemp Status Signal Object
     */
    StatusSignal<units::temperature::celsius_t> &GetProcessorTemp();
        
    /**
     * \brief Velocity of the motor rotor. This velocity is not affected
     * by any feedback configs.
     * 
     *   Minimum Value: -512.0
     *   Maximum Value: 511.998046875
     *   Default Value: 0
     *   Units: rotations per second
     * 
     * Default Rates:
     *   CAN 2.0: 4.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns RotorVelocity Status Signal Object
     */
    StatusSignal<units::angular_velocity::turns_per_second_t> &GetRotorVelocity();
        
    /**
     * \brief Position of the motor rotor. This position is only affected
     * by the RotorOffset config.
     * 
     *   Minimum Value: -16384.0
     *   Maximum Value: 16383.999755859375
     *   Default Value: 0
     *   Units: rotations
     * 
     * Default Rates:
     *   CAN 2.0: 4.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns RotorPosition Status Signal Object
     */
    StatusSignal<units::angle::turn_t> &GetRotorPosition();
        
    /**
     * \brief Velocity of the device in mechanism rotations per second.
     * This can be the velocity of a remote sensor and is affected by the
     * RotorToSensorRatio and SensorToMechanismRatio configs.
     * 
     *   Minimum Value: -512.0
     *   Maximum Value: 511.998046875
     *   Default Value: 0
     *   Units: rotations per second
     * 
     * Default Rates:
     *   CAN 2.0: 50.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns Velocity Status Signal Object
     */
    StatusSignal<units::angular_velocity::turns_per_second_t> &GetVelocity();
        
    /**
     * \brief Position of the device in mechanism rotations. This can be
     * the position of a remote sensor and is affected by the
     * RotorToSensorRatio and SensorToMechanismRatio configs.
     * 
     *   Minimum Value: -16384.0
     *   Maximum Value: 16383.999755859375
     *   Default Value: 0
     *   Units: rotations
     * 
     * Default Rates:
     *   CAN 2.0: 50.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns Position Status Signal Object
     */
    StatusSignal<units::angle::turn_t> &GetPosition();
        
    /**
     * \brief Acceleration of the device in mechanism rotations per
     * second². This can be the acceleration of a remote sensor and is
     * affected by the RotorToSensorRatio and SensorToMechanismRatio
     * configs.
     * 
     *   Minimum Value: -2048.0
     *   Maximum Value: 2047.75
     *   Default Value: 0
     *   Units: rotations per second²
     * 
     * Default Rates:
     *   CAN 2.0: 50.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns Acceleration Status Signal Object
     */
    StatusSignal<units::angular_acceleration::turns_per_second_squared_t> &GetAcceleration();
        
    /**
     * \brief The active control mode of the motor controller
     * 
     * 
     * Default Rates:
     *   CAN 2.0: 4.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns ControlMode Status Signal Object
     */
    StatusSignal<signals::ControlModeValue> &GetControlMode();
        
    /**
     * \brief Check if Motion Magic® is running.  This is equivalent to
     * checking that the reported control mode is a Motion Magic® based
     * mode.
     * 
     * 
     * Default Rates:
     *   CAN 2.0: 4.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns MotionMagicIsRunning Status Signal Object
     */
    StatusSignal<signals::MotionMagicIsRunningValue> &GetMotionMagicIsRunning();
        
    /**
     * \brief Indicates if device is actuator enabled.
     * 
     * 
     * Default Rates:
     *   CAN 2.0: 4.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns DeviceEnable Status Signal Object
     */
    StatusSignal<signals::DeviceEnableValue> &GetDeviceEnable();
        
    /**
     * \brief Closed loop slot in use
     * 
     * \details This is the slot that the closed loop PID is using.
     * 
     *   Minimum Value: 0
     *   Maximum Value: 2
     *   Default Value: 0
     *   Units: 
     * 
     * Default Rates:
     *   CAN 2.0: 4.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns ClosedLoopSlot Status Signal Object
     */
    StatusSignal<int> &GetClosedLoopSlot();
        
    /**
     * \brief The active control mode of the differential controller
     * 
     * 
     * Default Rates:
     *   CAN 2.0: 100.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns DifferentialControlMode Status Signal Object
     */
    StatusSignal<signals::DifferentialControlModeValue> &GetDifferentialControlMode();
        
    /**
     * \brief Average component of the differential velocity of device.
     * 
     *   Minimum Value: -512.0
     *   Maximum Value: 511.998046875
     *   Default Value: 0
     *   Units: rotations per second
     * 
     * Default Rates:
     *   CAN 2.0: 4.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns DifferentialAverageVelocity Status Signal Object
     */
    StatusSignal<units::angular_velocity::turns_per_second_t> &GetDifferentialAverageVelocity();
        
    /**
     * \brief Average component of the differential position of device.
     * 
     *   Minimum Value: -16384.0
     *   Maximum Value: 16383.999755859375
     *   Default Value: 0
     *   Units: rotations
     * 
     * Default Rates:
     *   CAN 2.0: 4.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns DifferentialAveragePosition Status Signal Object
     */
    StatusSignal<units::angle::turn_t> &GetDifferentialAveragePosition();
        
    /**
     * \brief Difference component of the differential velocity of device.
     * 
     *   Minimum Value: -512.0
     *   Maximum Value: 511.998046875
     *   Default Value: 0
     *   Units: rotations per second
     * 
     * Default Rates:
     *   CAN 2.0: 4.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns DifferentialDifferenceVelocity Status Signal Object
     */
    StatusSignal<units::angular_velocity::turns_per_second_t> &GetDifferentialDifferenceVelocity();
        
    /**
     * \brief Difference component of the differential position of device.
     * 
     *   Minimum Value: -16384.0
     *   Maximum Value: 16383.999755859375
     *   Default Value: 0
     *   Units: rotations
     * 
     * Default Rates:
     *   CAN 2.0: 4.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns DifferentialDifferencePosition Status Signal Object
     */
    StatusSignal<units::angle::turn_t> &GetDifferentialDifferencePosition();
        
    /**
     * \brief Differential Closed loop slot in use
     * 
     * \details This is the slot that the closed loop differential PID is
     * using.
     * 
     *   Minimum Value: 0
     *   Maximum Value: 2
     *   Default Value: 0
     *   Units: 
     * 
     * Default Rates:
     *   CAN 2.0: 4.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns DifferentialClosedLoopSlot Status Signal Object
     */
    StatusSignal<int> &GetDifferentialClosedLoopSlot();
        
    /**
     * \brief The applied output of the bridge.
     * 
     * 
     * Default Rates:
     *   CAN 2.0: 100.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns BridgeOutput Status Signal Object
     */
    StatusSignal<signals::BridgeOutputValue> &GetBridgeOutput();
        
    /**
     * \brief Whether the device is Phoenix Pro licensed.
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns IsProLicensed Status Signal Object
     */
    StatusSignal<bool> &GetIsProLicensed();
        
    /**
     * \brief Temperature of device from second sensor
     * 
     * \details Newer versions of Talon FX have multiple temperature
     * measurement methods.
     * 
     *   Minimum Value: 0.0
     *   Maximum Value: 255.0
     *   Default Value: 0
     *   Units: ℃
     * 
     * Default Rates:
     *   CAN 2.0: 4.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns AncillaryDeviceTemp Status Signal Object
     */
    StatusSignal<units::temperature::celsius_t> &GetAncillaryDeviceTemp();
        
    /**
     * \brief The type of motor attached to the Talon FX
     * 
     * \details This can be used to determine what motor is attached to
     * the Talon FX.  Return will be "Unknown" if firmware is too old or
     * device is not present.
     * 
     * 
     * Default Rates:
     *   CAN 2.0: 4.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns MotorType Status Signal Object
     */
    StatusSignal<signals::MotorTypeValue> &GetMotorType();
        
    /**
     * \brief Hardware fault occurred
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns Fault_Hardware Status Signal Object
     */
    StatusSignal<bool> &GetFault_Hardware();
        
    /**
     * \brief Hardware fault occurred
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns StickyFault_Hardware Status Signal Object
     */
    StatusSignal<bool> &GetStickyFault_Hardware();
        
    /**
     * \brief Processor temperature exceeded limit
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns Fault_ProcTemp Status Signal Object
     */
    StatusSignal<bool> &GetFault_ProcTemp();
        
    /**
     * \brief Processor temperature exceeded limit
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns StickyFault_ProcTemp Status Signal Object
     */
    StatusSignal<bool> &GetStickyFault_ProcTemp();
        
    /**
     * \brief Device temperature exceeded limit
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns Fault_DeviceTemp Status Signal Object
     */
    StatusSignal<bool> &GetFault_DeviceTemp();
        
    /**
     * \brief Device temperature exceeded limit
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns StickyFault_DeviceTemp Status Signal Object
     */
    StatusSignal<bool> &GetStickyFault_DeviceTemp();
        
    /**
     * \brief Device supply voltage dropped to near brownout levels
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns Fault_Undervoltage Status Signal Object
     */
    StatusSignal<bool> &GetFault_Undervoltage();
        
    /**
     * \brief Device supply voltage dropped to near brownout levels
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns StickyFault_Undervoltage Status Signal Object
     */
    StatusSignal<bool> &GetStickyFault_Undervoltage();
        
    /**
     * \brief Device boot while detecting the enable signal
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns Fault_BootDuringEnable Status Signal Object
     */
    StatusSignal<bool> &GetFault_BootDuringEnable();
        
    /**
     * \brief Device boot while detecting the enable signal
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns StickyFault_BootDuringEnable Status Signal Object
     */
    StatusSignal<bool> &GetStickyFault_BootDuringEnable();
        
    /**
     * \brief An unlicensed feature is in use, device may not behave as
     * expected.
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns Fault_UnlicensedFeatureInUse Status Signal Object
     */
    StatusSignal<bool> &GetFault_UnlicensedFeatureInUse();
        
    /**
     * \brief An unlicensed feature is in use, device may not behave as
     * expected.
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns StickyFault_UnlicensedFeatureInUse Status Signal Object
     */
    StatusSignal<bool> &GetStickyFault_UnlicensedFeatureInUse();
        
    /**
     * \brief Bridge was disabled most likely due to supply voltage
     * dropping too low.
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns Fault_BridgeBrownout Status Signal Object
     */
    StatusSignal<bool> &GetFault_BridgeBrownout();
        
    /**
     * \brief Bridge was disabled most likely due to supply voltage
     * dropping too low.
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns StickyFault_BridgeBrownout Status Signal Object
     */
    StatusSignal<bool> &GetStickyFault_BridgeBrownout();
        
    /**
     * \brief The remote sensor has reset.
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns Fault_RemoteSensorReset Status Signal Object
     */
    StatusSignal<bool> &GetFault_RemoteSensorReset();
        
    /**
     * \brief The remote sensor has reset.
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns StickyFault_RemoteSensorReset Status Signal Object
     */
    StatusSignal<bool> &GetStickyFault_RemoteSensorReset();
        
    /**
     * \brief The remote Talon FX used for differential control is not
     * present on CAN Bus.
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns Fault_MissingDifferentialFX Status Signal Object
     */
    StatusSignal<bool> &GetFault_MissingDifferentialFX();
        
    /**
     * \brief The remote Talon FX used for differential control is not
     * present on CAN Bus.
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns StickyFault_MissingDifferentialFX Status Signal Object
     */
    StatusSignal<bool> &GetStickyFault_MissingDifferentialFX();
        
    /**
     * \brief The remote sensor position has overflowed. Because of the
     * nature of remote sensors, it is possible for the remote sensor
     * position to overflow beyond what is supported by the status signal
     * frame. However, this is rare and cannot occur over the course of an
     * FRC match under normal use.
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns Fault_RemoteSensorPosOverflow Status Signal Object
     */
    StatusSignal<bool> &GetFault_RemoteSensorPosOverflow();
        
    /**
     * \brief The remote sensor position has overflowed. Because of the
     * nature of remote sensors, it is possible for the remote sensor
     * position to overflow beyond what is supported by the status signal
     * frame. However, this is rare and cannot occur over the course of an
     * FRC match under normal use.
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns StickyFault_RemoteSensorPosOverflow Status Signal Object
     */
    StatusSignal<bool> &GetStickyFault_RemoteSensorPosOverflow();
        
    /**
     * \brief Supply Voltage has exceeded the maximum voltage rating of
     * device.
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns Fault_OverSupplyV Status Signal Object
     */
    StatusSignal<bool> &GetFault_OverSupplyV();
        
    /**
     * \brief Supply Voltage has exceeded the maximum voltage rating of
     * device.
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns StickyFault_OverSupplyV Status Signal Object
     */
    StatusSignal<bool> &GetStickyFault_OverSupplyV();
        
    /**
     * \brief Supply Voltage is unstable.  Ensure you are using a battery
     * and current limited power supply.
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns Fault_UnstableSupplyV Status Signal Object
     */
    StatusSignal<bool> &GetFault_UnstableSupplyV();
        
    /**
     * \brief Supply Voltage is unstable.  Ensure you are using a battery
     * and current limited power supply.
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns StickyFault_UnstableSupplyV Status Signal Object
     */
    StatusSignal<bool> &GetStickyFault_UnstableSupplyV();
        
    /**
     * \brief Reverse limit switch has been asserted.  Output is set to
     * neutral.
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns Fault_ReverseHardLimit Status Signal Object
     */
    StatusSignal<bool> &GetFault_ReverseHardLimit();
        
    /**
     * \brief Reverse limit switch has been asserted.  Output is set to
     * neutral.
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns StickyFault_ReverseHardLimit Status Signal Object
     */
    StatusSignal<bool> &GetStickyFault_ReverseHardLimit();
        
    /**
     * \brief Forward limit switch has been asserted.  Output is set to
     * neutral.
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns Fault_ForwardHardLimit Status Signal Object
     */
    StatusSignal<bool> &GetFault_ForwardHardLimit();
        
    /**
     * \brief Forward limit switch has been asserted.  Output is set to
     * neutral.
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns StickyFault_ForwardHardLimit Status Signal Object
     */
    StatusSignal<bool> &GetStickyFault_ForwardHardLimit();
        
    /**
     * \brief Reverse soft limit has been asserted.  Output is set to
     * neutral.
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns Fault_ReverseSoftLimit Status Signal Object
     */
    StatusSignal<bool> &GetFault_ReverseSoftLimit();
        
    /**
     * \brief Reverse soft limit has been asserted.  Output is set to
     * neutral.
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns StickyFault_ReverseSoftLimit Status Signal Object
     */
    StatusSignal<bool> &GetStickyFault_ReverseSoftLimit();
        
    /**
     * \brief Forward soft limit has been asserted.  Output is set to
     * neutral.
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns Fault_ForwardSoftLimit Status Signal Object
     */
    StatusSignal<bool> &GetFault_ForwardSoftLimit();
        
    /**
     * \brief Forward soft limit has been asserted.  Output is set to
     * neutral.
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns StickyFault_ForwardSoftLimit Status Signal Object
     */
    StatusSignal<bool> &GetStickyFault_ForwardSoftLimit();
        
    /**
     * \brief The remote sensor's data is no longer trusted. This can
     * happen if the remote sensor disappears from the CAN bus or if the
     * remote sensor indicates its data is no longer valid, such as when a
     * CANcoder's magnet strength falls into the "red" range.
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns Fault_RemoteSensorDataInvalid Status Signal Object
     */
    StatusSignal<bool> &GetFault_RemoteSensorDataInvalid();
        
    /**
     * \brief The remote sensor's data is no longer trusted. This can
     * happen if the remote sensor disappears from the CAN bus or if the
     * remote sensor indicates its data is no longer valid, such as when a
     * CANcoder's magnet strength falls into the "red" range.
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns StickyFault_RemoteSensorDataInvalid Status Signal Object
     */
    StatusSignal<bool> &GetStickyFault_RemoteSensorDataInvalid();
        
    /**
     * \brief The remote sensor used for fusion has fallen out of sync to
     * the local sensor. A re-synchronization has occurred, which may
     * cause a discontinuity. This typically happens if there is
     * significant slop in the mechanism, or if the RotorToSensorRatio
     * configuration parameter is incorrect.
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns Fault_FusedSensorOutOfSync Status Signal Object
     */
    StatusSignal<bool> &GetFault_FusedSensorOutOfSync();
        
    /**
     * \brief The remote sensor used for fusion has fallen out of sync to
     * the local sensor. A re-synchronization has occurred, which may
     * cause a discontinuity. This typically happens if there is
     * significant slop in the mechanism, or if the RotorToSensorRatio
     * configuration parameter is incorrect.
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns StickyFault_FusedSensorOutOfSync Status Signal Object
     */
    StatusSignal<bool> &GetStickyFault_FusedSensorOutOfSync();
        
    /**
     * \brief Stator current limit occured.
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns Fault_StatorCurrLimit Status Signal Object
     */
    StatusSignal<bool> &GetFault_StatorCurrLimit();
        
    /**
     * \brief Stator current limit occured.
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns StickyFault_StatorCurrLimit Status Signal Object
     */
    StatusSignal<bool> &GetStickyFault_StatorCurrLimit();
        
    /**
     * \brief Supply current limit occured.
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns Fault_SupplyCurrLimit Status Signal Object
     */
    StatusSignal<bool> &GetFault_SupplyCurrLimit();
        
    /**
     * \brief Supply current limit occured.
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns StickyFault_SupplyCurrLimit Status Signal Object
     */
    StatusSignal<bool> &GetStickyFault_SupplyCurrLimit();
        
    /**
     * \brief Using Fused CANcoder feature while unlicensed. Device has
     * fallen back to remote CANcoder.
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns Fault_UsingFusedCANcoderWhileUnlicensed Status Signal Object
     */
    StatusSignal<bool> &GetFault_UsingFusedCANcoderWhileUnlicensed();
        
    /**
     * \brief Using Fused CANcoder feature while unlicensed. Device has
     * fallen back to remote CANcoder.
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns StickyFault_UsingFusedCANcoderWhileUnlicensed Status Signal Object
     */
    StatusSignal<bool> &GetStickyFault_UsingFusedCANcoderWhileUnlicensed();
    
    /**
     * \brief Closed loop proportional component
     * 
     * \details The portion of the closed loop output that is the
     * proportional to the error. Alternatively, the p-Contribution of the
     * closed loop output.
     * 
     * Default Rates:
     *   CAN 2.0: 4.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns  ClosedLoopProportionalOutput Status Signal object
     */
    StatusSignal<double> &GetClosedLoopProportionalOutput();
    
    /**
     * \brief Closed loop integrated component
     * 
     * \details The portion of the closed loop output that is proportional
     * to the integrated error. Alternatively, the i-Contribution of the
     * closed loop output.
     * 
     * Default Rates:
     *   CAN 2.0: 4.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns  ClosedLoopIntegratedOutput Status Signal object
     */
    StatusSignal<double> &GetClosedLoopIntegratedOutput();
    
    /**
     * \brief Feedforward passed by the user
     * 
     * \details This is the general feedforward that the user provides for
     * the closed loop.
     * 
     * Default Rates:
     *   CAN 2.0: 4.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns  ClosedLoopFeedForward Status Signal object
     */
    StatusSignal<double> &GetClosedLoopFeedForward();
    
    /**
     * \brief Closed loop derivative component
     * 
     * \details The portion of the closed loop output that is the
     * proportional to the deriviative the error. Alternatively, the
     * d-Contribution of the closed loop output.
     * 
     * Default Rates:
     *   CAN 2.0: 4.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns  ClosedLoopDerivativeOutput Status Signal object
     */
    StatusSignal<double> &GetClosedLoopDerivativeOutput();
    
    /**
     * \brief Closed loop total output
     * 
     * \details The total output of the closed loop output.
     * 
     * Default Rates:
     *   CAN 2.0: 4.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns  ClosedLoopOutput Status Signal object
     */
    StatusSignal<double> &GetClosedLoopOutput();
    
    /**
     * \brief Value that the closed loop is targeting
     * 
     * \details This is the value that the closed loop PID controller
     * targets.
     * 
     * Default Rates:
     *   CAN 2.0: 4.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns  ClosedLoopReference Status Signal object
     */
    StatusSignal<double> &GetClosedLoopReference();
    
    /**
     * \brief Derivative of the target that the closed loop is targeting
     * 
     * \details This is the change in the closed loop reference. This may
     * be used in the feed-forward calculation, the derivative-error, or
     * in application of the signage for kS. Typically, this represents
     * the target velocity during Motion Magic®.
     * 
     * Default Rates:
     *   CAN 2.0: 4.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns  ClosedLoopReferenceSlope Status Signal object
     */
    StatusSignal<double> &GetClosedLoopReferenceSlope();
    
    /**
     * \brief The difference between target reference and current
     * measurement
     * 
     * \details This is the value that is treated as the error in the PID
     * loop.
     * 
     * Default Rates:
     *   CAN 2.0: 4.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns  ClosedLoopError Status Signal object
     */
    StatusSignal<double> &GetClosedLoopError();
    
    /**
     * \brief The calculated motor output for differential followers.
     * 
     * \details This is a torque request when using the TorqueCurrentFOC
     * control output type, and a duty cycle in all other control types.
     * 
     * Default Rates:
     *   CAN 2.0: 100.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns  DifferentialOutput Status Signal object
     */
    StatusSignal<double> &GetDifferentialOutput();
    
    /**
     * \brief Differential closed loop proportional component
     * 
     * \details The portion of the differential closed loop output that is
     * the proportional to the error. Alternatively, the p-Contribution of
     * the closed loop output.
     * 
     * Default Rates:
     *   CAN 2.0: 4.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns  DifferentialClosedLoopProportionalOutput Status Signal object
     */
    StatusSignal<double> &GetDifferentialClosedLoopProportionalOutput();
    
    /**
     * \brief Differential closed loop integrated component
     * 
     * \details The portion of the differential closed loop output that is
     * proportional to the integrated error. Alternatively, the
     * i-Contribution of the closed loop output.
     * 
     * Default Rates:
     *   CAN 2.0: 100.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns  DifferentialClosedLoopIntegratedOutput Status Signal object
     */
    StatusSignal<double> &GetDifferentialClosedLoopIntegratedOutput();
    
    /**
     * \brief Differential Feedforward passed by the user
     * 
     * \details This is the general feedforward that the user provides for
     * the differential closed loop.
     * 
     * Default Rates:
     *   CAN 2.0: 100.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns  DifferentialClosedLoopFeedForward Status Signal object
     */
    StatusSignal<double> &GetDifferentialClosedLoopFeedForward();
    
    /**
     * \brief Differential closed loop derivative component
     * 
     * \details The portion of the differential closed loop output that is
     * the proportional to the deriviative the error. Alternatively, the
     * d-Contribution of the closed loop output.
     * 
     * Default Rates:
     *   CAN 2.0: 4.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns  DifferentialClosedLoopDerivativeOutput Status Signal object
     */
    StatusSignal<double> &GetDifferentialClosedLoopDerivativeOutput();
    
    /**
     * \brief Differential closed loop total output
     * 
     * \details The total output of the differential closed loop output.
     * 
     * Default Rates:
     *   CAN 2.0: 4.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns  DifferentialClosedLoopOutput Status Signal object
     */
    StatusSignal<double> &GetDifferentialClosedLoopOutput();
    
    /**
     * \brief Value that the differential closed loop is targeting
     * 
     * \details This is the value that the differential closed loop PID
     * controller targets.
     * 
     * Default Rates:
     *   CAN 2.0: 4.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns  DifferentialClosedLoopReference Status Signal object
     */
    StatusSignal<double> &GetDifferentialClosedLoopReference();
    
    /**
     * \brief Derivative of the target that the differential closed loop
     * is targeting
     * 
     * \details This is the change in the closed loop reference. This may
     * be used in the feed-forward calculation, the derivative-error, or
     * in application of the signage for kS. Typically, this represents
     * the target velocity during Motion Magic®.
     * 
     * Default Rates:
     *   CAN 2.0: 4.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns  DifferentialClosedLoopReferenceSlope Status Signal object
     */
    StatusSignal<double> &GetDifferentialClosedLoopReferenceSlope();
    
    /**
     * \brief The difference between target differential reference and
     * current measurement
     * 
     * \details This is the value that is treated as the error in the
     * differential PID loop.
     * 
     * Default Rates:
     *   CAN 2.0: 4.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns  DifferentialClosedLoopError Status Signal object
     */
    StatusSignal<double> &GetDifferentialClosedLoopError();

    
    /**
     * \brief Request a specified motor duty cycle.
     * 
     * \details This control mode will output a proportion of the supplied
     * voltage which is supplied by the user.
     * 
     *   DutyCycleOut Parameters: 
     *    Output: Proportion of supply voltage to apply in fractional units between -1
     *         and +1
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::DutyCycleOut& request);
    /**
     * \brief Request a specified motor duty cycle.
     * 
     * \details This control mode will output a proportion of the supplied
     * voltage which is supplied by the user.
     * 
     *   DutyCycleOut Parameters: 
     *    Output: Proportion of supply voltage to apply in fractional units between -1
     *         and +1
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::DutyCycleOut&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Request a specified motor current (field oriented control).
     * 
     * \details This control request will drive the motor to the requested
     * motor (stator) current value.  This leverages field oriented
     * control (FOC), which means greater peak power than what is
     * documented.  This scales to torque based on Motor's kT constant.
     * 
     *   TorqueCurrentFOC Parameters: 
     *    Output: Amount of motor current in Amperes
     *    MaxAbsDutyCycle: The maximum absolute motor output that can be applied, which
     *                  effectively limits the velocity. For example, 0.50 means no
     *                  more than 50% output in either direction.  This is useful
     *                  for preventing the motor from spinning to its terminal
     *                  velocity when there is no external torque applied unto the
     *                  rotor.  Note this is absolute maximum, so the value should
     *                  be between zero and one.
     *    Deadband: Deadband in Amperes.  If torque request is within deadband, the
     *           bridge output is neutral. If deadband is set to zero then there is
     *           effectively no deadband. Note if deadband is zero, a free spinning
     *           motor will spin for quite a while as the firmware attempts to hold
     *           the motor's bemf. If user expects motor to cease spinning quickly
     *           with a demand of zero, we recommend a deadband of one Ampere. This
     *           value will be converted to an integral value of amps.
     *    OverrideCoastDurNeutral: Set to true to coast the rotor when output is zero
     *                          (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0A (zero torque).
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::TorqueCurrentFOC& request);
    /**
     * \brief Request a specified motor current (field oriented control).
     * 
     * \details This control request will drive the motor to the requested
     * motor (stator) current value.  This leverages field oriented
     * control (FOC), which means greater peak power than what is
     * documented.  This scales to torque based on Motor's kT constant.
     * 
     *   TorqueCurrentFOC Parameters: 
     *    Output: Amount of motor current in Amperes
     *    MaxAbsDutyCycle: The maximum absolute motor output that can be applied, which
     *                  effectively limits the velocity. For example, 0.50 means no
     *                  more than 50% output in either direction.  This is useful
     *                  for preventing the motor from spinning to its terminal
     *                  velocity when there is no external torque applied unto the
     *                  rotor.  Note this is absolute maximum, so the value should
     *                  be between zero and one.
     *    Deadband: Deadband in Amperes.  If torque request is within deadband, the
     *           bridge output is neutral. If deadband is set to zero then there is
     *           effectively no deadband. Note if deadband is zero, a free spinning
     *           motor will spin for quite a while as the firmware attempts to hold
     *           the motor's bemf. If user expects motor to cease spinning quickly
     *           with a demand of zero, we recommend a deadband of one Ampere. This
     *           value will be converted to an integral value of amps.
     *    OverrideCoastDurNeutral: Set to true to coast the rotor when output is zero
     *                          (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0A (zero torque).
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::TorqueCurrentFOC&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Request a specified voltage.
     * 
     * \details This control mode will attempt to apply the specified
     * voltage to the motor. If the supply voltage is below the requested
     * voltage, the motor controller will output the supply voltage.
     * 
     *   VoltageOut Parameters: 
     *    Output: Voltage to attempt to drive at
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::VoltageOut& request);
    /**
     * \brief Request a specified voltage.
     * 
     * \details This control mode will attempt to apply the specified
     * voltage to the motor. If the supply voltage is below the requested
     * voltage, the motor controller will output the supply voltage.
     * 
     *   VoltageOut Parameters: 
     *    Output: Voltage to attempt to drive at
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::VoltageOut&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Request PID to target position with duty cycle feedforward.
     * 
     * \details This control mode will set the motor's position setpoint
     * to the position specified by the user. In addition, it will apply
     * an additional duty cycle as an arbitrary feedforward value.
     * 
     *   PositionDutyCycle Parameters: 
     *    Position: Position to drive toward in rotations.
     *    Velocity: Velocity to drive toward in rotations per second. This is typically
     *           used for motion profiles generated by the robot program.
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    FeedForward: Feedforward to apply in fractional units between -1 and +1.
     *    Slot: Select which gains are applied by selecting the slot.  Use the
     *       configuration api to set the gain values for the selected slot before
     *       enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::PositionDutyCycle& request);
    /**
     * \brief Request PID to target position with duty cycle feedforward.
     * 
     * \details This control mode will set the motor's position setpoint
     * to the position specified by the user. In addition, it will apply
     * an additional duty cycle as an arbitrary feedforward value.
     * 
     *   PositionDutyCycle Parameters: 
     *    Position: Position to drive toward in rotations.
     *    Velocity: Velocity to drive toward in rotations per second. This is typically
     *           used for motion profiles generated by the robot program.
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    FeedForward: Feedforward to apply in fractional units between -1 and +1.
     *    Slot: Select which gains are applied by selecting the slot.  Use the
     *       configuration api to set the gain values for the selected slot before
     *       enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::PositionDutyCycle&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Request PID to target position with voltage feedforward
     * 
     * \details This control mode will set the motor's position setpoint
     * to the position specified by the user. In addition, it will apply
     * an additional voltage as an arbitrary feedforward value.
     * 
     *   PositionVoltage Parameters: 
     *    Position: Position to drive toward in rotations.
     *    Velocity: Velocity to drive toward in rotations per second. This is typically
     *           used for motion profiles generated by the robot program.
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    FeedForward: Feedforward to apply in volts
     *    Slot: Select which gains are applied by selecting the slot.  Use the
     *       configuration api to set the gain values for the selected slot before
     *       enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::PositionVoltage& request);
    /**
     * \brief Request PID to target position with voltage feedforward
     * 
     * \details This control mode will set the motor's position setpoint
     * to the position specified by the user. In addition, it will apply
     * an additional voltage as an arbitrary feedforward value.
     * 
     *   PositionVoltage Parameters: 
     *    Position: Position to drive toward in rotations.
     *    Velocity: Velocity to drive toward in rotations per second. This is typically
     *           used for motion profiles generated by the robot program.
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    FeedForward: Feedforward to apply in volts
     *    Slot: Select which gains are applied by selecting the slot.  Use the
     *       configuration api to set the gain values for the selected slot before
     *       enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::PositionVoltage&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Request PID to target position with torque current
     * feedforward.
     * 
     * \details This control mode will set the motor's position setpoint
     * to the position specified by the user. In addition, it will apply
     * an additional torque current as an arbitrary feedforward value.
     * 
     *   PositionTorqueCurrentFOC Parameters: 
     *    Position: Position to drive toward in rotations.
     *    Velocity: Velocity to drive toward in rotations per second. This is typically
     *           used for motion profiles generated by the robot program.
     *    FeedForward: Feedforward to apply in torque current in Amperes.  User can use
     *              motor's kT to scale Newton-meter to Amperes.
     *    Slot: Select which gains are applied by selecting the slot.  Use the
     *       configuration api to set the gain values for the selected slot before
     *       enabling this feature. Slot must be within [0,2].
     *    OverrideCoastDurNeutral: Set to true to coast the rotor when output is zero
     *                          (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0A (zero torque).
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::PositionTorqueCurrentFOC& request);
    /**
     * \brief Request PID to target position with torque current
     * feedforward.
     * 
     * \details This control mode will set the motor's position setpoint
     * to the position specified by the user. In addition, it will apply
     * an additional torque current as an arbitrary feedforward value.
     * 
     *   PositionTorqueCurrentFOC Parameters: 
     *    Position: Position to drive toward in rotations.
     *    Velocity: Velocity to drive toward in rotations per second. This is typically
     *           used for motion profiles generated by the robot program.
     *    FeedForward: Feedforward to apply in torque current in Amperes.  User can use
     *              motor's kT to scale Newton-meter to Amperes.
     *    Slot: Select which gains are applied by selecting the slot.  Use the
     *       configuration api to set the gain values for the selected slot before
     *       enabling this feature. Slot must be within [0,2].
     *    OverrideCoastDurNeutral: Set to true to coast the rotor when output is zero
     *                          (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0A (zero torque).
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::PositionTorqueCurrentFOC&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Request PID to target velocity with duty cycle feedforward.
     * 
     * \details This control mode will set the motor's velocity setpoint
     * to the velocity specified by the user. In addition, it will apply
     * an additional voltage as an arbitrary feedforward value.
     * 
     *   VelocityDutyCycle Parameters: 
     *    Velocity: Velocity to drive toward in rotations per second.
     *    Acceleration: Acceleration to drive toward in rotations per second squared.
     *               This is typically used for motion profiles generated by the
     *               robot program.
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    FeedForward: Feedforward to apply in fractional units between -1 and +1.
     *    Slot: Select which gains are applied by selecting the slot.  Use the
     *       configuration api to set the gain values for the selected slot before
     *       enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::VelocityDutyCycle& request);
    /**
     * \brief Request PID to target velocity with duty cycle feedforward.
     * 
     * \details This control mode will set the motor's velocity setpoint
     * to the velocity specified by the user. In addition, it will apply
     * an additional voltage as an arbitrary feedforward value.
     * 
     *   VelocityDutyCycle Parameters: 
     *    Velocity: Velocity to drive toward in rotations per second.
     *    Acceleration: Acceleration to drive toward in rotations per second squared.
     *               This is typically used for motion profiles generated by the
     *               robot program.
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    FeedForward: Feedforward to apply in fractional units between -1 and +1.
     *    Slot: Select which gains are applied by selecting the slot.  Use the
     *       configuration api to set the gain values for the selected slot before
     *       enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::VelocityDutyCycle&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Request PID to target velocity with voltage feedforward.
     * 
     * \details This control mode will set the motor's velocity setpoint
     * to the velocity specified by the user. In addition, it will apply
     * an additional voltage as an arbitrary feedforward value.
     * 
     *   VelocityVoltage Parameters: 
     *    Velocity: Velocity to drive toward in rotations per second.
     *    Acceleration: Acceleration to drive toward in rotations per second squared.
     *               This is typically used for motion profiles generated by the
     *               robot program.
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    FeedForward: Feedforward to apply in volts
     *    Slot: Select which gains are applied by selecting the slot.  Use the
     *       configuration api to set the gain values for the selected slot before
     *       enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::VelocityVoltage& request);
    /**
     * \brief Request PID to target velocity with voltage feedforward.
     * 
     * \details This control mode will set the motor's velocity setpoint
     * to the velocity specified by the user. In addition, it will apply
     * an additional voltage as an arbitrary feedforward value.
     * 
     *   VelocityVoltage Parameters: 
     *    Velocity: Velocity to drive toward in rotations per second.
     *    Acceleration: Acceleration to drive toward in rotations per second squared.
     *               This is typically used for motion profiles generated by the
     *               robot program.
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    FeedForward: Feedforward to apply in volts
     *    Slot: Select which gains are applied by selecting the slot.  Use the
     *       configuration api to set the gain values for the selected slot before
     *       enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::VelocityVoltage&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Request PID to target velocity with torque current
     * feedforward.
     * 
     * \details This control mode will set the motor's velocity setpoint
     * to the velocity specified by the user. In addition, it will apply
     * an additional torque current as an arbitrary feedforward value.
     * 
     *   VelocityTorqueCurrentFOC Parameters: 
     *    Velocity: Velocity to drive toward in rotations per second.
     *    Acceleration: Acceleration to drive toward in rotations per second squared.
     *               This is typically used for motion profiles generated by the
     *               robot program.
     *    FeedForward: Feedforward to apply in torque current in Amperes.  User can use
     *              motor's kT to scale Newton-meter to Amperes.
     *    Slot: Select which gains are applied by selecting the slot.  Use the
     *       configuration api to set the gain values for the selected slot before
     *       enabling this feature. Slot must be within [0,2].
     *    OverrideCoastDurNeutral: Set to true to coast the rotor when output is zero
     *                          (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0A (zero torque).
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::VelocityTorqueCurrentFOC& request);
    /**
     * \brief Request PID to target velocity with torque current
     * feedforward.
     * 
     * \details This control mode will set the motor's velocity setpoint
     * to the velocity specified by the user. In addition, it will apply
     * an additional torque current as an arbitrary feedforward value.
     * 
     *   VelocityTorqueCurrentFOC Parameters: 
     *    Velocity: Velocity to drive toward in rotations per second.
     *    Acceleration: Acceleration to drive toward in rotations per second squared.
     *               This is typically used for motion profiles generated by the
     *               robot program.
     *    FeedForward: Feedforward to apply in torque current in Amperes.  User can use
     *              motor's kT to scale Newton-meter to Amperes.
     *    Slot: Select which gains are applied by selecting the slot.  Use the
     *       configuration api to set the gain values for the selected slot before
     *       enabling this feature. Slot must be within [0,2].
     *    OverrideCoastDurNeutral: Set to true to coast the rotor when output is zero
     *                          (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0A (zero torque).
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::VelocityTorqueCurrentFOC&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Requests Motion Magic® to target a final position using a
     * motion profile.  Users can optionally provide a duty cycle
     * feedforward.
     * 
     * \details Motion Magic® produces a motion profile in real-time while
     * attempting to honor the Cruise Velocity, Acceleration, and Jerk
     * value specified via the Motion Magic® configuration values.  This
     * control mode does not use the Expo_kV or Expo_kA configs.  Target
     * position can be changed on-the-fly and Motion Magic® will do its
     * best to adjust the profile.  This control mode is duty cycle based,
     * so relevant closed-loop gains will use fractional duty cycle for
     * the numerator:  +1.0 represents full forward output.
     * 
     *   MotionMagicDutyCycle Parameters: 
     *    Position: Position to drive toward in rotations.
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    FeedForward: Feedforward to apply in fractional units between -1 and +1.
     *    Slot: Select which gains are applied by selecting the slot.  Use the
     *       configuration api to set the gain values for the selected slot before
     *       enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::MotionMagicDutyCycle& request);
    /**
     * \brief Requests Motion Magic® to target a final position using a
     * motion profile.  Users can optionally provide a duty cycle
     * feedforward.
     * 
     * \details Motion Magic® produces a motion profile in real-time while
     * attempting to honor the Cruise Velocity, Acceleration, and Jerk
     * value specified via the Motion Magic® configuration values.  This
     * control mode does not use the Expo_kV or Expo_kA configs.  Target
     * position can be changed on-the-fly and Motion Magic® will do its
     * best to adjust the profile.  This control mode is duty cycle based,
     * so relevant closed-loop gains will use fractional duty cycle for
     * the numerator:  +1.0 represents full forward output.
     * 
     *   MotionMagicDutyCycle Parameters: 
     *    Position: Position to drive toward in rotations.
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    FeedForward: Feedforward to apply in fractional units between -1 and +1.
     *    Slot: Select which gains are applied by selecting the slot.  Use the
     *       configuration api to set the gain values for the selected slot before
     *       enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::MotionMagicDutyCycle&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Requests Motion Magic® to target a final position using a
     * motion profile.  Users can optionally provide a voltage
     * feedforward.
     * 
     * \details Motion Magic® produces a motion profile in real-time while
     * attempting to honor the Cruise Velocity, Acceleration, and Jerk
     * value specified via the Motion Magic® configuration values.  This
     * control mode does not use the Expo_kV or Expo_kA configs.  Target
     * position can be changed on-the-fly and Motion Magic® will do its
     * best to adjust the profile.  This control mode is voltage-based, so
     * relevant closed-loop gains will use Volts for the numerator.
     * 
     *   MotionMagicVoltage Parameters: 
     *    Position: Position to drive toward in rotations.
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    FeedForward: Feedforward to apply in volts
     *    Slot: Select which gains are applied by selecting the slot.  Use the
     *       configuration api to set the gain values for the selected slot before
     *       enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::MotionMagicVoltage& request);
    /**
     * \brief Requests Motion Magic® to target a final position using a
     * motion profile.  Users can optionally provide a voltage
     * feedforward.
     * 
     * \details Motion Magic® produces a motion profile in real-time while
     * attempting to honor the Cruise Velocity, Acceleration, and Jerk
     * value specified via the Motion Magic® configuration values.  This
     * control mode does not use the Expo_kV or Expo_kA configs.  Target
     * position can be changed on-the-fly and Motion Magic® will do its
     * best to adjust the profile.  This control mode is voltage-based, so
     * relevant closed-loop gains will use Volts for the numerator.
     * 
     *   MotionMagicVoltage Parameters: 
     *    Position: Position to drive toward in rotations.
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    FeedForward: Feedforward to apply in volts
     *    Slot: Select which gains are applied by selecting the slot.  Use the
     *       configuration api to set the gain values for the selected slot before
     *       enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::MotionMagicVoltage&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Requests Motion Magic® to target a final position using a
     * motion profile.  Users can optionally provide a torque current
     * feedforward.
     * 
     * \details Motion Magic® produces a motion profile in real-time while
     * attempting to honor the Cruise Velocity, Acceleration, and Jerk
     * value specified via the Motion Magic® configuration values.  This
     * control mode does not use the Expo_kV or Expo_kA configs.  Target
     * position can be changed on-the-fly and Motion Magic® will do its
     * best to adjust the profile.  This control mode is based on torque
     * current, so relevant closed-loop gains will use Amperes for the
     * numerator.
     * 
     *   MotionMagicTorqueCurrentFOC Parameters: 
     *    Position: Position to drive toward in rotations.
     *    FeedForward: Feedforward to apply in torque current in Amperes.  User can use
     *              motor's kT to scale Newton-meter to Amperes.
     *    Slot: Select which gains are applied by selecting the slot.  Use the
     *       configuration api to set the gain values for the selected slot before
     *       enabling this feature. Slot must be within [0,2].
     *    OverrideCoastDurNeutral: Set to true to coast the rotor when output is zero
     *                          (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0A (zero torque).
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::MotionMagicTorqueCurrentFOC& request);
    /**
     * \brief Requests Motion Magic® to target a final position using a
     * motion profile.  Users can optionally provide a torque current
     * feedforward.
     * 
     * \details Motion Magic® produces a motion profile in real-time while
     * attempting to honor the Cruise Velocity, Acceleration, and Jerk
     * value specified via the Motion Magic® configuration values.  This
     * control mode does not use the Expo_kV or Expo_kA configs.  Target
     * position can be changed on-the-fly and Motion Magic® will do its
     * best to adjust the profile.  This control mode is based on torque
     * current, so relevant closed-loop gains will use Amperes for the
     * numerator.
     * 
     *   MotionMagicTorqueCurrentFOC Parameters: 
     *    Position: Position to drive toward in rotations.
     *    FeedForward: Feedforward to apply in torque current in Amperes.  User can use
     *              motor's kT to scale Newton-meter to Amperes.
     *    Slot: Select which gains are applied by selecting the slot.  Use the
     *       configuration api to set the gain values for the selected slot before
     *       enabling this feature. Slot must be within [0,2].
     *    OverrideCoastDurNeutral: Set to true to coast the rotor when output is zero
     *                          (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0A (zero torque).
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::MotionMagicTorqueCurrentFOC&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Request a specified motor duty cycle with a differential
     * position closed-loop.
     * 
     * \details This control mode will output a proportion of the supplied
     * voltage which is supplied by the user. It will also set the motor's
     * differential position setpoint to the specified position.
     * 
     *   DifferentialDutyCycle Parameters: 
     *    TargetOutput: Proportion of supply voltage to apply in fractional units
     *               between -1 and +1
     *    DifferentialPosition: Differential position to drive towards in rotations
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    DifferentialSlot: Select which gains are applied to the differential
     *                   controller by selecting the slot.  Use the configuration
     *                   api to set the gain values for the selected slot before
     *                   enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::DifferentialDutyCycle& request);
    /**
     * \brief Request a specified motor duty cycle with a differential
     * position closed-loop.
     * 
     * \details This control mode will output a proportion of the supplied
     * voltage which is supplied by the user. It will also set the motor's
     * differential position setpoint to the specified position.
     * 
     *   DifferentialDutyCycle Parameters: 
     *    TargetOutput: Proportion of supply voltage to apply in fractional units
     *               between -1 and +1
     *    DifferentialPosition: Differential position to drive towards in rotations
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    DifferentialSlot: Select which gains are applied to the differential
     *                   controller by selecting the slot.  Use the configuration
     *                   api to set the gain values for the selected slot before
     *                   enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::DifferentialDutyCycle&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Request a specified voltage with a differential position
     * closed-loop.
     * 
     * \details This control mode will attempt to apply the specified
     * voltage to the motor. If the supply voltage is below the requested
     * voltage, the motor controller will output the supply voltage. It
     * will also set the motor's differential position setpoint to the
     * specified position.
     * 
     *   DifferentialVoltage Parameters: 
     *    TargetOutput: Voltage to attempt to drive at
     *    DifferentialPosition: Differential position to drive towards in rotations
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    DifferentialSlot: Select which gains are applied to the differential
     *                   controller by selecting the slot.  Use the configuration
     *                   api to set the gain values for the selected slot before
     *                   enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::DifferentialVoltage& request);
    /**
     * \brief Request a specified voltage with a differential position
     * closed-loop.
     * 
     * \details This control mode will attempt to apply the specified
     * voltage to the motor. If the supply voltage is below the requested
     * voltage, the motor controller will output the supply voltage. It
     * will also set the motor's differential position setpoint to the
     * specified position.
     * 
     *   DifferentialVoltage Parameters: 
     *    TargetOutput: Voltage to attempt to drive at
     *    DifferentialPosition: Differential position to drive towards in rotations
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    DifferentialSlot: Select which gains are applied to the differential
     *                   controller by selecting the slot.  Use the configuration
     *                   api to set the gain values for the selected slot before
     *                   enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::DifferentialVoltage&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Request PID to target position with a differential position
     * setpoint.
     * 
     * \details This control mode will set the motor's position setpoint
     * to the position specified by the user. It will also set the motor's
     * differential position setpoint to the specified position.
     * 
     *   DifferentialPositionDutyCycle Parameters: 
     *    TargetPosition: Average position to drive toward in rotations.
     *    DifferentialPosition: Differential position to drive toward in rotations.
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    TargetSlot: Select which gains are applied to the primary controller by
     *             selecting the slot.  Use the configuration api to set the gain
     *             values for the selected slot before enabling this feature. Slot
     *             must be within [0,2].
     *    DifferentialSlot: Select which gains are applied to the differential
     *                   controller by selecting the slot.  Use the configuration
     *                   api to set the gain values for the selected slot before
     *                   enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::DifferentialPositionDutyCycle& request);
    /**
     * \brief Request PID to target position with a differential position
     * setpoint.
     * 
     * \details This control mode will set the motor's position setpoint
     * to the position specified by the user. It will also set the motor's
     * differential position setpoint to the specified position.
     * 
     *   DifferentialPositionDutyCycle Parameters: 
     *    TargetPosition: Average position to drive toward in rotations.
     *    DifferentialPosition: Differential position to drive toward in rotations.
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    TargetSlot: Select which gains are applied to the primary controller by
     *             selecting the slot.  Use the configuration api to set the gain
     *             values for the selected slot before enabling this feature. Slot
     *             must be within [0,2].
     *    DifferentialSlot: Select which gains are applied to the differential
     *                   controller by selecting the slot.  Use the configuration
     *                   api to set the gain values for the selected slot before
     *                   enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::DifferentialPositionDutyCycle&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Request PID to target position with a differential position
     * setpoint
     * 
     * \details This control mode will set the motor's position setpoint
     * to the position specified by the user. It will also set the motor's
     * differential position setpoint to the specified position.
     * 
     *   DifferentialPositionVoltage Parameters: 
     *    TargetPosition: Average position to drive toward in rotations.
     *    DifferentialPosition: Differential position to drive toward in rotations.
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    TargetSlot: Select which gains are applied to the primary controller by
     *             selecting the slot.  Use the configuration api to set the gain
     *             values for the selected slot before enabling this feature. Slot
     *             must be within [0,2].
     *    DifferentialSlot: Select which gains are applied to the differential
     *                   controller by selecting the slot.  Use the configuration
     *                   api to set the gain values for the selected slot before
     *                   enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::DifferentialPositionVoltage& request);
    /**
     * \brief Request PID to target position with a differential position
     * setpoint
     * 
     * \details This control mode will set the motor's position setpoint
     * to the position specified by the user. It will also set the motor's
     * differential position setpoint to the specified position.
     * 
     *   DifferentialPositionVoltage Parameters: 
     *    TargetPosition: Average position to drive toward in rotations.
     *    DifferentialPosition: Differential position to drive toward in rotations.
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    TargetSlot: Select which gains are applied to the primary controller by
     *             selecting the slot.  Use the configuration api to set the gain
     *             values for the selected slot before enabling this feature. Slot
     *             must be within [0,2].
     *    DifferentialSlot: Select which gains are applied to the differential
     *                   controller by selecting the slot.  Use the configuration
     *                   api to set the gain values for the selected slot before
     *                   enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::DifferentialPositionVoltage&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Request PID to target velocity with a differential position
     * setpoint.
     * 
     * \details This control mode will set the motor's velocity setpoint
     * to the velocity specified by the user. It will also set the motor's
     * differential position setpoint to the specified position.
     * 
     *   DifferentialVelocityDutyCycle Parameters: 
     *    TargetVelocity: Average velocity to drive toward in rotations per second.
     *    DifferentialPosition: Differential position to drive toward in rotations.
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    TargetSlot: Select which gains are applied to the primary controller by
     *             selecting the slot.  Use the configuration api to set the gain
     *             values for the selected slot before enabling this feature. Slot
     *             must be within [0,2].
     *    DifferentialSlot: Select which gains are applied to the differential
     *                   controller by selecting the slot.  Use the configuration
     *                   api to set the gain values for the selected slot before
     *                   enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::DifferentialVelocityDutyCycle& request);
    /**
     * \brief Request PID to target velocity with a differential position
     * setpoint.
     * 
     * \details This control mode will set the motor's velocity setpoint
     * to the velocity specified by the user. It will also set the motor's
     * differential position setpoint to the specified position.
     * 
     *   DifferentialVelocityDutyCycle Parameters: 
     *    TargetVelocity: Average velocity to drive toward in rotations per second.
     *    DifferentialPosition: Differential position to drive toward in rotations.
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    TargetSlot: Select which gains are applied to the primary controller by
     *             selecting the slot.  Use the configuration api to set the gain
     *             values for the selected slot before enabling this feature. Slot
     *             must be within [0,2].
     *    DifferentialSlot: Select which gains are applied to the differential
     *                   controller by selecting the slot.  Use the configuration
     *                   api to set the gain values for the selected slot before
     *                   enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::DifferentialVelocityDutyCycle&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Request PID to target velocity with a differential position
     * setpoint.
     * 
     * \details This control mode will set the motor's velocity setpoint
     * to the velocity specified by the user. It will also set the motor's
     * differential position setpoint to the specified position.
     * 
     *   DifferentialVelocityVoltage Parameters: 
     *    TargetVelocity: Average velocity to drive toward in rotations per second.
     *    DifferentialPosition: Differential position to drive toward in rotations.
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    TargetSlot: Select which gains are applied to the primary controller by
     *             selecting the slot.  Use the configuration api to set the gain
     *             values for the selected slot before enabling this feature. Slot
     *             must be within [0,2].
     *    DifferentialSlot: Select which gains are applied to the differential
     *                   controller by selecting the slot.  Use the configuration
     *                   api to set the gain values for the selected slot before
     *                   enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::DifferentialVelocityVoltage& request);
    /**
     * \brief Request PID to target velocity with a differential position
     * setpoint.
     * 
     * \details This control mode will set the motor's velocity setpoint
     * to the velocity specified by the user. It will also set the motor's
     * differential position setpoint to the specified position.
     * 
     *   DifferentialVelocityVoltage Parameters: 
     *    TargetVelocity: Average velocity to drive toward in rotations per second.
     *    DifferentialPosition: Differential position to drive toward in rotations.
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    TargetSlot: Select which gains are applied to the primary controller by
     *             selecting the slot.  Use the configuration api to set the gain
     *             values for the selected slot before enabling this feature. Slot
     *             must be within [0,2].
     *    DifferentialSlot: Select which gains are applied to the differential
     *                   controller by selecting the slot.  Use the configuration
     *                   api to set the gain values for the selected slot before
     *                   enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::DifferentialVelocityVoltage&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Requests Motion Magic® to target a final position using a
     * motion profile, and PID to a differential position setpoint.
     * 
     * \details Motion Magic® produces a motion profile in real-time while
     * attempting to honor the Cruise Velocity, Acceleration, and Jerk
     * value specified via the Motion Magic® configuration values.  This
     * control mode does not use the Expo_kV or Expo_kA configs.  Target
     * position can be changed on-the-fly and Motion Magic® will do its
     * best to adjust the profile.  This control mode is duty cycle based,
     * so relevant closed-loop gains will use fractional duty cycle for
     * the numerator:  +1.0 represents full forward output.
     * 
     *   DifferentialMotionMagicDutyCycle Parameters: 
     *    TargetPosition: Average position to drive toward in rotations.
     *    DifferentialPosition: Differential position to drive toward in rotations.
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    TargetSlot: Select which gains are applied to the primary controller by
     *             selecting the slot.  Use the configuration api to set the gain
     *             values for the selected slot before enabling this feature. Slot
     *             must be within [0,2].
     *    DifferentialSlot: Select which gains are applied to the differential
     *                   controller by selecting the slot.  Use the configuration
     *                   api to set the gain values for the selected slot before
     *                   enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::DifferentialMotionMagicDutyCycle& request);
    /**
     * \brief Requests Motion Magic® to target a final position using a
     * motion profile, and PID to a differential position setpoint.
     * 
     * \details Motion Magic® produces a motion profile in real-time while
     * attempting to honor the Cruise Velocity, Acceleration, and Jerk
     * value specified via the Motion Magic® configuration values.  This
     * control mode does not use the Expo_kV or Expo_kA configs.  Target
     * position can be changed on-the-fly and Motion Magic® will do its
     * best to adjust the profile.  This control mode is duty cycle based,
     * so relevant closed-loop gains will use fractional duty cycle for
     * the numerator:  +1.0 represents full forward output.
     * 
     *   DifferentialMotionMagicDutyCycle Parameters: 
     *    TargetPosition: Average position to drive toward in rotations.
     *    DifferentialPosition: Differential position to drive toward in rotations.
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    TargetSlot: Select which gains are applied to the primary controller by
     *             selecting the slot.  Use the configuration api to set the gain
     *             values for the selected slot before enabling this feature. Slot
     *             must be within [0,2].
     *    DifferentialSlot: Select which gains are applied to the differential
     *                   controller by selecting the slot.  Use the configuration
     *                   api to set the gain values for the selected slot before
     *                   enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::DifferentialMotionMagicDutyCycle&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Requests Motion Magic® to target a final position using a
     * motion profile, and PID to a differential position setpoint.
     * 
     * \details Motion Magic® produces a motion profile in real-time while
     * attempting to honor the Cruise Velocity, Acceleration, and Jerk
     * value specified via the Motion Magic® configuration values.  This
     * control mode does not use the Expo_kV or Expo_kA configs.  Target
     * position can be changed on-the-fly and Motion Magic® will do its
     * best to adjust the profile.  This control mode is voltage-based, so
     * relevant closed-loop gains will use Volts for the numerator.
     * 
     *   DifferentialMotionMagicVoltage Parameters: 
     *    TargetPosition: Average position to drive toward in rotations.
     *    DifferentialPosition: Differential position to drive toward in rotations.
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    TargetSlot: Select which gains are applied to the primary controller by
     *             selecting the slot.  Use the configuration api to set the gain
     *             values for the selected slot before enabling this feature. Slot
     *             must be within [0,2].
     *    DifferentialSlot: Select which gains are applied to the differential
     *                   controller by selecting the slot.  Use the configuration
     *                   api to set the gain values for the selected slot before
     *                   enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::DifferentialMotionMagicVoltage& request);
    /**
     * \brief Requests Motion Magic® to target a final position using a
     * motion profile, and PID to a differential position setpoint.
     * 
     * \details Motion Magic® produces a motion profile in real-time while
     * attempting to honor the Cruise Velocity, Acceleration, and Jerk
     * value specified via the Motion Magic® configuration values.  This
     * control mode does not use the Expo_kV or Expo_kA configs.  Target
     * position can be changed on-the-fly and Motion Magic® will do its
     * best to adjust the profile.  This control mode is voltage-based, so
     * relevant closed-loop gains will use Volts for the numerator.
     * 
     *   DifferentialMotionMagicVoltage Parameters: 
     *    TargetPosition: Average position to drive toward in rotations.
     *    DifferentialPosition: Differential position to drive toward in rotations.
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    TargetSlot: Select which gains are applied to the primary controller by
     *             selecting the slot.  Use the configuration api to set the gain
     *             values for the selected slot before enabling this feature. Slot
     *             must be within [0,2].
     *    DifferentialSlot: Select which gains are applied to the differential
     *                   controller by selecting the slot.  Use the configuration
     *                   api to set the gain values for the selected slot before
     *                   enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::DifferentialMotionMagicVoltage&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Follow the motor output of another Talon.
     * 
     * \details If Talon is in torque control, the torque is copied -
     * which will increase the total torque applied. If Talon is in
     * percent supply output control, the duty cycle is matched.  Motor
     * direction either matches master's configured direction or opposes
     * it based on OpposeMasterDirection.
     * 
     *   Follower Parameters: 
     *    MasterID: Device ID of the master to follow.
     *    OpposeMasterDirection: Set to false for motor invert to match the master's
     *                        configured Invert - which is typical when master and
     *                        follower are mechanically linked and spin in the same
     *                        direction.  Set to true for motor invert to oppose the
     *                        master's configured Invert - this is typical where the
     *                        the master and follower mechanically spin in opposite
     *                        directions.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::Follower& request);
    /**
     * \brief Follow the motor output of another Talon.
     * 
     * \details If Talon is in torque control, the torque is copied -
     * which will increase the total torque applied. If Talon is in
     * percent supply output control, the duty cycle is matched.  Motor
     * direction either matches master's configured direction or opposes
     * it based on OpposeMasterDirection.
     * 
     *   Follower Parameters: 
     *    MasterID: Device ID of the master to follow.
     *    OpposeMasterDirection: Set to false for motor invert to match the master's
     *                        configured Invert - which is typical when master and
     *                        follower are mechanically linked and spin in the same
     *                        direction.  Set to true for motor invert to oppose the
     *                        master's configured Invert - this is typical where the
     *                        the master and follower mechanically spin in opposite
     *                        directions.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::Follower&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Follow the motor output of another Talon while ignoring the
     * master's invert setting.
     * 
     * \details If Talon is in torque control, the torque is copied -
     * which will increase the total torque applied. If Talon is in
     * percent supply output control, the duty cycle is matched.  Motor
     * direction is strictly determined by the configured invert and not
     * the master.  If you want motor direction to match or oppose the
     * master, use FollowerRequest instead.
     * 
     *   StrictFollower Parameters: 
     *    MasterID: Device ID of the master to follow.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::StrictFollower& request);
    /**
     * \brief Follow the motor output of another Talon while ignoring the
     * master's invert setting.
     * 
     * \details If Talon is in torque control, the torque is copied -
     * which will increase the total torque applied. If Talon is in
     * percent supply output control, the duty cycle is matched.  Motor
     * direction is strictly determined by the configured invert and not
     * the master.  If you want motor direction to match or oppose the
     * master, use FollowerRequest instead.
     * 
     *   StrictFollower Parameters: 
     *    MasterID: Device ID of the master to follow.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::StrictFollower&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Follow the differential motor output of another Talon.
     * 
     * \details If Talon is in torque control, the torque is copied -
     * which will increase the total torque applied. If Talon is in
     * percent supply output control, the duty cycle is matched.  Motor
     * direction either matches master's configured direction or opposes
     * it based on OpposeMasterDirection.
     * 
     *   DifferentialFollower Parameters: 
     *    MasterID: Device ID of the differential master to follow.
     *    OpposeMasterDirection: Set to false for motor invert to match the master's
     *                        configured Invert - which is typical when master and
     *                        follower are mechanically linked and spin in the same
     *                        direction.  Set to true for motor invert to oppose the
     *                        master's configured Invert - this is typical where the
     *                        the master and follower mechanically spin in opposite
     *                        directions.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::DifferentialFollower& request);
    /**
     * \brief Follow the differential motor output of another Talon.
     * 
     * \details If Talon is in torque control, the torque is copied -
     * which will increase the total torque applied. If Talon is in
     * percent supply output control, the duty cycle is matched.  Motor
     * direction either matches master's configured direction or opposes
     * it based on OpposeMasterDirection.
     * 
     *   DifferentialFollower Parameters: 
     *    MasterID: Device ID of the differential master to follow.
     *    OpposeMasterDirection: Set to false for motor invert to match the master's
     *                        configured Invert - which is typical when master and
     *                        follower are mechanically linked and spin in the same
     *                        direction.  Set to true for motor invert to oppose the
     *                        master's configured Invert - this is typical where the
     *                        the master and follower mechanically spin in opposite
     *                        directions.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::DifferentialFollower&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Follow the differential motor output of another Talon while
     * ignoring the master's invert setting.
     * 
     * \details If Talon is in torque control, the torque is copied -
     * which will increase the total torque applied. If Talon is in
     * percent supply output control, the duty cycle is matched.  Motor
     * direction is strictly determined by the configured invert and not
     * the master.  If you want motor direction to match or oppose the
     * master, use FollowerRequest instead.
     * 
     *   DifferentialStrictFollower Parameters: 
     *    MasterID: Device ID of the differential master to follow.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::DifferentialStrictFollower& request);
    /**
     * \brief Follow the differential motor output of another Talon while
     * ignoring the master's invert setting.
     * 
     * \details If Talon is in torque control, the torque is copied -
     * which will increase the total torque applied. If Talon is in
     * percent supply output control, the duty cycle is matched.  Motor
     * direction is strictly determined by the configured invert and not
     * the master.  If you want motor direction to match or oppose the
     * master, use FollowerRequest instead.
     * 
     *   DifferentialStrictFollower Parameters: 
     *    MasterID: Device ID of the differential master to follow.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::DifferentialStrictFollower&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Request neutral output of actuator. The applied brake type
     * is determined by the NeutralMode configuration.
     * 
     *   NeutralOut Parameters: 
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::NeutralOut& request);
    /**
     * \brief Request neutral output of actuator. The applied brake type
     * is determined by the NeutralMode configuration.
     * 
     *   NeutralOut Parameters: 
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::NeutralOut&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Request coast neutral output of actuator.  The bridge is
     * disabled and the rotor is allowed to coast.
     * 
     *   CoastOut Parameters: 
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::CoastOut& request);
    /**
     * \brief Request coast neutral output of actuator.  The bridge is
     * disabled and the rotor is allowed to coast.
     * 
     *   CoastOut Parameters: 
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::CoastOut&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Applies full neutral-brake by shorting motor leads together.
     * 
     *   StaticBrake Parameters: 
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::StaticBrake& request);
    /**
     * \brief Applies full neutral-brake by shorting motor leads together.
     * 
     *   StaticBrake Parameters: 
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::StaticBrake&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Plays a single tone at the user specified frequency.
     * 
     *   MusicTone Parameters: 
     *    AudioFrequency: Sound frequency to play.  A value of zero will silence the
     *                 device. The effective frequency range is 10-20000Hz.  Any
     *                 nonzero frequency less than 10 Hz will be capped to 10Hz. 
     *                 Any frequency above 20Khz will be capped to 20KHz.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::MusicTone& request);
    /**
     * \brief Plays a single tone at the user specified frequency.
     * 
     *   MusicTone Parameters: 
     *    AudioFrequency: Sound frequency to play.  A value of zero will silence the
     *                 device. The effective frequency range is 10-20000Hz.  Any
     *                 nonzero frequency less than 10 Hz will be capped to 10Hz. 
     *                 Any frequency above 20Khz will be capped to 20KHz.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::MusicTone&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Requests Motion Magic® to target a final velocity using a
     * motion profile.  This allows smooth transitions between velocity
     * set points.  Users can optionally provide a duty cycle feedforward.
     * 
     * \details Motion Magic® Velocity produces a motion profile in
     * real-time while attempting to honor the specified Acceleration and
     * Jerk value. This control mode does not use the CruiseVelocity,
     * Expo_kV, or Expo_kA configs. If the specified acceleration is zero,
     * the Acceleration under Motion Magic® configuration parameter is
     * used instead. This allows for runtime adjustment of acceleration
     * for advanced users.  Jerk is also specified in the Motion Magic®
     * persistent configuration values.  If Jerk is set to zero, Motion
     * Magic® will produce a trapezoidal acceleration profile.  Target
     * velocity can also be changed on-the-fly and Motion Magic® will do
     * its best to adjust the profile.  This control mode is duty cycle
     * based, so relevant closed-loop gains will use fractional duty cycle
     * for the numerator:  +1.0 represents full forward output.
     * 
     *   MotionMagicVelocityDutyCycle Parameters: 
     *    Velocity: Target velocity to drive toward in rotations per second.  This can
     *           be changed on-the fly.
     *    Acceleration: This is the absolute Acceleration to use generating the
     *               profile.  If this parameter is zero, the Acceleration
     *               persistent configuration parameter is used instead.
     *               Acceleration is in rotations per second squared.  If nonzero,
     *               the signage does not matter as the absolute value is used.
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    FeedForward: Feedforward to apply in fractional units between -1 and +1.
     *    Slot: Select which gains are applied by selecting the slot.  Use the
     *       configuration api to set the gain values for the selected slot before
     *       enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::MotionMagicVelocityDutyCycle& request);
    /**
     * \brief Requests Motion Magic® to target a final velocity using a
     * motion profile.  This allows smooth transitions between velocity
     * set points.  Users can optionally provide a duty cycle feedforward.
     * 
     * \details Motion Magic® Velocity produces a motion profile in
     * real-time while attempting to honor the specified Acceleration and
     * Jerk value. This control mode does not use the CruiseVelocity,
     * Expo_kV, or Expo_kA configs. If the specified acceleration is zero,
     * the Acceleration under Motion Magic® configuration parameter is
     * used instead. This allows for runtime adjustment of acceleration
     * for advanced users.  Jerk is also specified in the Motion Magic®
     * persistent configuration values.  If Jerk is set to zero, Motion
     * Magic® will produce a trapezoidal acceleration profile.  Target
     * velocity can also be changed on-the-fly and Motion Magic® will do
     * its best to adjust the profile.  This control mode is duty cycle
     * based, so relevant closed-loop gains will use fractional duty cycle
     * for the numerator:  +1.0 represents full forward output.
     * 
     *   MotionMagicVelocityDutyCycle Parameters: 
     *    Velocity: Target velocity to drive toward in rotations per second.  This can
     *           be changed on-the fly.
     *    Acceleration: This is the absolute Acceleration to use generating the
     *               profile.  If this parameter is zero, the Acceleration
     *               persistent configuration parameter is used instead.
     *               Acceleration is in rotations per second squared.  If nonzero,
     *               the signage does not matter as the absolute value is used.
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    FeedForward: Feedforward to apply in fractional units between -1 and +1.
     *    Slot: Select which gains are applied by selecting the slot.  Use the
     *       configuration api to set the gain values for the selected slot before
     *       enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::MotionMagicVelocityDutyCycle&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Requests Motion Magic® to target a final velocity using a
     * motion profile.  This allows smooth transitions between velocity
     * set points.  Users can optionally provide a torque feedforward.
     * 
     * \details Motion Magic® Velocity produces a motion profile in
     * real-time while attempting to honor the specified Acceleration and
     * Jerk value. This control mode does not use the CruiseVelocity,
     * Expo_kV, or Expo_kA configs. If the specified acceleration is zero,
     * the Acceleration under Motion Magic® configuration parameter is
     * used instead. This allows for runtime adjustment of acceleration
     * for advanced users.  Jerk is also specified in the Motion Magic®
     * persistent configuration values.  If Jerk is set to zero, Motion
     * Magic® will produce a trapezoidal acceleration profile.  Target
     * velocity can also be changed on-the-fly and Motion Magic® will do
     * its best to adjust the profile.  This control mode is based on
     * torque current, so relevant closed-loop gains will use Amperes for
     * the numerator.
     * 
     *   MotionMagicVelocityTorqueCurrentFOC Parameters: 
     *    Velocity: Target velocity to drive toward in rotations per second.  This can
     *           be changed on-the fly.
     *    Acceleration: This is the absolute Acceleration to use generating the
     *               profile.  If this parameter is zero, the Acceleration
     *               persistent configuration parameter is used instead.
     *               Acceleration is in rotations per second squared.  If nonzero,
     *               the signage does not matter as the absolute value is used.
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    FeedForward: Feedforward to apply in torque current in Amperes.  User can use
     *              motor's kT to scale Newton-meter to Amperes.
     *    Slot: Select which gains are applied by selecting the slot.  Use the
     *       configuration api to set the gain values for the selected slot before
     *       enabling this feature. Slot must be within [0,2].
     *    OverrideCoastDurNeutral: Set to true to coast the rotor when output is zero
     *                          (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0A (zero torque).
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::MotionMagicVelocityTorqueCurrentFOC& request);
    /**
     * \brief Requests Motion Magic® to target a final velocity using a
     * motion profile.  This allows smooth transitions between velocity
     * set points.  Users can optionally provide a torque feedforward.
     * 
     * \details Motion Magic® Velocity produces a motion profile in
     * real-time while attempting to honor the specified Acceleration and
     * Jerk value. This control mode does not use the CruiseVelocity,
     * Expo_kV, or Expo_kA configs. If the specified acceleration is zero,
     * the Acceleration under Motion Magic® configuration parameter is
     * used instead. This allows for runtime adjustment of acceleration
     * for advanced users.  Jerk is also specified in the Motion Magic®
     * persistent configuration values.  If Jerk is set to zero, Motion
     * Magic® will produce a trapezoidal acceleration profile.  Target
     * velocity can also be changed on-the-fly and Motion Magic® will do
     * its best to adjust the profile.  This control mode is based on
     * torque current, so relevant closed-loop gains will use Amperes for
     * the numerator.
     * 
     *   MotionMagicVelocityTorqueCurrentFOC Parameters: 
     *    Velocity: Target velocity to drive toward in rotations per second.  This can
     *           be changed on-the fly.
     *    Acceleration: This is the absolute Acceleration to use generating the
     *               profile.  If this parameter is zero, the Acceleration
     *               persistent configuration parameter is used instead.
     *               Acceleration is in rotations per second squared.  If nonzero,
     *               the signage does not matter as the absolute value is used.
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    FeedForward: Feedforward to apply in torque current in Amperes.  User can use
     *              motor's kT to scale Newton-meter to Amperes.
     *    Slot: Select which gains are applied by selecting the slot.  Use the
     *       configuration api to set the gain values for the selected slot before
     *       enabling this feature. Slot must be within [0,2].
     *    OverrideCoastDurNeutral: Set to true to coast the rotor when output is zero
     *                          (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0A (zero torque).
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::MotionMagicVelocityTorqueCurrentFOC&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Requests Motion Magic® to target a final velocity using a
     * motion profile.  This allows smooth transitions between velocity
     * set points.  Users can optionally provide a voltage feedforward.
     * 
     * \details Motion Magic® Velocity produces a motion profile in
     * real-time while attempting to honor the specified Acceleration and
     * Jerk value. This control mode does not use the CruiseVelocity,
     * Expo_kV, or Expo_kA configs. If the specified acceleration is zero,
     * the Acceleration under Motion Magic® configuration parameter is
     * used instead. This allows for runtime adjustment of acceleration
     * for advanced users.  Jerk is also specified in the Motion Magic®
     * persistent configuration values.  If Jerk is set to zero, Motion
     * Magic® will produce a trapezoidal acceleration profile.  Target
     * velocity can also be changed on-the-fly and Motion Magic® will do
     * its best to adjust the profile.  This control mode is
     * voltage-based, so relevant closed-loop gains will use Volts for the
     * numerator.
     * 
     *   MotionMagicVelocityVoltage Parameters: 
     *    Velocity: Target velocity to drive toward in rotations per second.  This can
     *           be changed on-the fly.
     *    Acceleration: This is the absolute Acceleration to use generating the
     *               profile.  If this parameter is zero, the Acceleration
     *               persistent configuration parameter is used instead.
     *               Acceleration is in rotations per second squared.  If nonzero,
     *               the signage does not matter as the absolute value is used.
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    FeedForward: Feedforward to apply in volts
     *    Slot: Select which gains are applied by selecting the slot.  Use the
     *       configuration api to set the gain values for the selected slot before
     *       enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::MotionMagicVelocityVoltage& request);
    /**
     * \brief Requests Motion Magic® to target a final velocity using a
     * motion profile.  This allows smooth transitions between velocity
     * set points.  Users can optionally provide a voltage feedforward.
     * 
     * \details Motion Magic® Velocity produces a motion profile in
     * real-time while attempting to honor the specified Acceleration and
     * Jerk value. This control mode does not use the CruiseVelocity,
     * Expo_kV, or Expo_kA configs. If the specified acceleration is zero,
     * the Acceleration under Motion Magic® configuration parameter is
     * used instead. This allows for runtime adjustment of acceleration
     * for advanced users.  Jerk is also specified in the Motion Magic®
     * persistent configuration values.  If Jerk is set to zero, Motion
     * Magic® will produce a trapezoidal acceleration profile.  Target
     * velocity can also be changed on-the-fly and Motion Magic® will do
     * its best to adjust the profile.  This control mode is
     * voltage-based, so relevant closed-loop gains will use Volts for the
     * numerator.
     * 
     *   MotionMagicVelocityVoltage Parameters: 
     *    Velocity: Target velocity to drive toward in rotations per second.  This can
     *           be changed on-the fly.
     *    Acceleration: This is the absolute Acceleration to use generating the
     *               profile.  If this parameter is zero, the Acceleration
     *               persistent configuration parameter is used instead.
     *               Acceleration is in rotations per second squared.  If nonzero,
     *               the signage does not matter as the absolute value is used.
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    FeedForward: Feedforward to apply in volts
     *    Slot: Select which gains are applied by selecting the slot.  Use the
     *       configuration api to set the gain values for the selected slot before
     *       enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::MotionMagicVelocityVoltage&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Requests Motion Magic® to target a final position using an
     * exponential motion profile.  Users can optionally provide a duty
     * cycle feedforward.
     * 
     * \details Motion Magic® Expo produces a motion profile in real-time
     * while attempting to honor the Cruise Velocity (optional) and the
     * mechanism kV and kA, specified via the Motion Magic® configuration
     * values.  Setting Cruise Velocity to 0 will allow the profile to run
     * to the max possible velocity based on Expo_kV.  This control mode
     * does not use the Acceleration or Jerk configs.  Target position can
     * be changed on-the-fly and Motion Magic® will do its best to adjust
     * the profile.  This control mode is duty cycle based, so relevant
     * closed-loop gains will use fractional duty cycle for the numerator:
     *  +1.0 represents full forward output.
     * 
     *   MotionMagicExpoDutyCycle Parameters: 
     *    Position: Position to drive toward in rotations.
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    FeedForward: Feedforward to apply in fractional units between -1 and +1.
     *    Slot: Select which gains are applied by selecting the slot.  Use the
     *       configuration api to set the gain values for the selected slot before
     *       enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::MotionMagicExpoDutyCycle& request);
    /**
     * \brief Requests Motion Magic® to target a final position using an
     * exponential motion profile.  Users can optionally provide a duty
     * cycle feedforward.
     * 
     * \details Motion Magic® Expo produces a motion profile in real-time
     * while attempting to honor the Cruise Velocity (optional) and the
     * mechanism kV and kA, specified via the Motion Magic® configuration
     * values.  Setting Cruise Velocity to 0 will allow the profile to run
     * to the max possible velocity based on Expo_kV.  This control mode
     * does not use the Acceleration or Jerk configs.  Target position can
     * be changed on-the-fly and Motion Magic® will do its best to adjust
     * the profile.  This control mode is duty cycle based, so relevant
     * closed-loop gains will use fractional duty cycle for the numerator:
     *  +1.0 represents full forward output.
     * 
     *   MotionMagicExpoDutyCycle Parameters: 
     *    Position: Position to drive toward in rotations.
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    FeedForward: Feedforward to apply in fractional units between -1 and +1.
     *    Slot: Select which gains are applied by selecting the slot.  Use the
     *       configuration api to set the gain values for the selected slot before
     *       enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::MotionMagicExpoDutyCycle&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Requests Motion Magic® to target a final position using an
     * exponential motion profile.  Users can optionally provide a voltage
     * feedforward.
     * 
     * \details Motion Magic® Expo produces a motion profile in real-time
     * while attempting to honor the Cruise Velocity (optional) and the
     * mechanism kV and kA, specified via the Motion Magic® configuration
     * values.  Setting Cruise Velocity to 0 will allow the profile to run
     * to the max possible velocity based on Expo_kV.  This control mode
     * does not use the Acceleration or Jerk configs.  Target position can
     * be changed on-the-fly and Motion Magic® will do its best to adjust
     * the profile.  This control mode is voltage-based, so relevant
     * closed-loop gains will use Volts for the numerator.
     * 
     *   MotionMagicExpoVoltage Parameters: 
     *    Position: Position to drive toward in rotations.
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    FeedForward: Feedforward to apply in volts
     *    Slot: Select which gains are applied by selecting the slot.  Use the
     *       configuration api to set the gain values for the selected slot before
     *       enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::MotionMagicExpoVoltage& request);
    /**
     * \brief Requests Motion Magic® to target a final position using an
     * exponential motion profile.  Users can optionally provide a voltage
     * feedforward.
     * 
     * \details Motion Magic® Expo produces a motion profile in real-time
     * while attempting to honor the Cruise Velocity (optional) and the
     * mechanism kV and kA, specified via the Motion Magic® configuration
     * values.  Setting Cruise Velocity to 0 will allow the profile to run
     * to the max possible velocity based on Expo_kV.  This control mode
     * does not use the Acceleration or Jerk configs.  Target position can
     * be changed on-the-fly and Motion Magic® will do its best to adjust
     * the profile.  This control mode is voltage-based, so relevant
     * closed-loop gains will use Volts for the numerator.
     * 
     *   MotionMagicExpoVoltage Parameters: 
     *    Position: Position to drive toward in rotations.
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    FeedForward: Feedforward to apply in volts
     *    Slot: Select which gains are applied by selecting the slot.  Use the
     *       configuration api to set the gain values for the selected slot before
     *       enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::MotionMagicExpoVoltage&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Requests Motion Magic® to target a final position using an
     * exponential motion profile.  Users can optionally provide a torque
     * current feedforward.
     * 
     * \details Motion Magic® Expo produces a motion profile in real-time
     * while attempting to honor the Cruise Velocity (optional) and the
     * mechanism kV and kA, specified via the Motion Magic® configuration
     * values.  Setting Cruise Velocity to 0 will allow the profile to run
     * to the max possible velocity based on Expo_kV.  This control mode
     * does not use the Acceleration or Jerk configs.  Target position can
     * be changed on-the-fly and Motion Magic® will do its best to adjust
     * the profile.  This control mode is based on torque current, so
     * relevant closed-loop gains will use Amperes for the numerator.
     * 
     *   MotionMagicExpoTorqueCurrentFOC Parameters: 
     *    Position: Position to drive toward in rotations.
     *    FeedForward: Feedforward to apply in torque current in Amperes.  User can use
     *              motor's kT to scale Newton-meter to Amperes.
     *    Slot: Select which gains are applied by selecting the slot.  Use the
     *       configuration api to set the gain values for the selected slot before
     *       enabling this feature. Slot must be within [0,2].
     *    OverrideCoastDurNeutral: Set to true to coast the rotor when output is zero
     *                          (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0A (zero torque).
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::MotionMagicExpoTorqueCurrentFOC& request);
    /**
     * \brief Requests Motion Magic® to target a final position using an
     * exponential motion profile.  Users can optionally provide a torque
     * current feedforward.
     * 
     * \details Motion Magic® Expo produces a motion profile in real-time
     * while attempting to honor the Cruise Velocity (optional) and the
     * mechanism kV and kA, specified via the Motion Magic® configuration
     * values.  Setting Cruise Velocity to 0 will allow the profile to run
     * to the max possible velocity based on Expo_kV.  This control mode
     * does not use the Acceleration or Jerk configs.  Target position can
     * be changed on-the-fly and Motion Magic® will do its best to adjust
     * the profile.  This control mode is based on torque current, so
     * relevant closed-loop gains will use Amperes for the numerator.
     * 
     *   MotionMagicExpoTorqueCurrentFOC Parameters: 
     *    Position: Position to drive toward in rotations.
     *    FeedForward: Feedforward to apply in torque current in Amperes.  User can use
     *              motor's kT to scale Newton-meter to Amperes.
     *    Slot: Select which gains are applied by selecting the slot.  Use the
     *       configuration api to set the gain values for the selected slot before
     *       enabling this feature. Slot must be within [0,2].
     *    OverrideCoastDurNeutral: Set to true to coast the rotor when output is zero
     *                          (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0A (zero torque).
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::MotionMagicExpoTorqueCurrentFOC&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Requests Motion Magic® to target a final position using a
     * motion profile.  This dynamic request allows runtime changes to
     * Cruise Velocity, Acceleration, and Jerk.  Users can optionally
     * provide a duty cycle feedforward.  This control requires use of a
     * CANivore.
     * 
     * \details Motion Magic® produces a motion profile in real-time while
     * attempting to honor the specified Cruise Velocity, Acceleration,
     * and Jerk value.  This control mode does not use the Expo_kV or
     * Expo_kA configs.  Target position can be changed on-the-fly and
     * Motion Magic® will do its best to adjust the profile. This control
     * mode is duty cycle based, so relevant closed-loop gains will use
     * fractional duty cycle for the numerator:  +1.0 represents full
     * forward output.
     * 
     *   DynamicMotionMagicDutyCycle Parameters: 
     *    Position: Position to drive toward in rotations.
     *    Velocity: Cruise velocity for profiling.  The signage does not matter as the
     *           device will use the absolute value for profile generation.
     *    Acceleration: Acceleration for profiling.  The signage does not matter as the
     *               device will use the absolute value for profile generation
     *    Jerk: Jerk for profiling.  The signage does not matter as the device will use
     *       the absolute value for profile generation
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    FeedForward: Feedforward to apply in fractional units between -1 and +1.
     *    Slot: Select which gains are applied by selecting the slot.  Use the
     *       configuration api to set the gain values for the selected slot before
     *       enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::DynamicMotionMagicDutyCycle& request);
    /**
     * \brief Requests Motion Magic® to target a final position using a
     * motion profile.  This dynamic request allows runtime changes to
     * Cruise Velocity, Acceleration, and Jerk.  Users can optionally
     * provide a duty cycle feedforward.  This control requires use of a
     * CANivore.
     * 
     * \details Motion Magic® produces a motion profile in real-time while
     * attempting to honor the specified Cruise Velocity, Acceleration,
     * and Jerk value.  This control mode does not use the Expo_kV or
     * Expo_kA configs.  Target position can be changed on-the-fly and
     * Motion Magic® will do its best to adjust the profile. This control
     * mode is duty cycle based, so relevant closed-loop gains will use
     * fractional duty cycle for the numerator:  +1.0 represents full
     * forward output.
     * 
     *   DynamicMotionMagicDutyCycle Parameters: 
     *    Position: Position to drive toward in rotations.
     *    Velocity: Cruise velocity for profiling.  The signage does not matter as the
     *           device will use the absolute value for profile generation.
     *    Acceleration: Acceleration for profiling.  The signage does not matter as the
     *               device will use the absolute value for profile generation
     *    Jerk: Jerk for profiling.  The signage does not matter as the device will use
     *       the absolute value for profile generation
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    FeedForward: Feedforward to apply in fractional units between -1 and +1.
     *    Slot: Select which gains are applied by selecting the slot.  Use the
     *       configuration api to set the gain values for the selected slot before
     *       enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::DynamicMotionMagicDutyCycle&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Requests Motion Magic® to target a final position using a
     * motion profile.  This dynamic request allows runtime changes to
     * Cruise Velocity, Acceleration, and Jerk.  Users can optionally
     * provide a voltage feedforward.  This control requires use of a
     * CANivore.
     * 
     * \details Motion Magic® produces a motion profile in real-time while
     * attempting to honor the specified Cruise Velocity, Acceleration,
     * and Jerk value.  This control mode does not use the Expo_kV or
     * Expo_kA configs.  Target position can be changed on-the-fly and
     * Motion Magic® will do its best to adjust the profile.  This control
     * mode is voltage-based, so relevant closed-loop gains will use Volts
     * for the numerator.
     * 
     *   DynamicMotionMagicVoltage Parameters: 
     *    Position: Position to drive toward in rotations.
     *    Velocity: Cruise velocity for profiling.  The signage does not matter as the
     *           device will use the absolute value for profile generation.
     *    Acceleration: Acceleration for profiling.  The signage does not matter as the
     *               device will use the absolute value for profile generation.
     *    Jerk: Jerk for profiling.  The signage does not matter as the device will use
     *       the absolute value for profile generation.
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    FeedForward: Feedforward to apply in volts
     *    Slot: Select which gains are applied by selecting the slot.  Use the
     *       configuration api to set the gain values for the selected slot before
     *       enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::DynamicMotionMagicVoltage& request);
    /**
     * \brief Requests Motion Magic® to target a final position using a
     * motion profile.  This dynamic request allows runtime changes to
     * Cruise Velocity, Acceleration, and Jerk.  Users can optionally
     * provide a voltage feedforward.  This control requires use of a
     * CANivore.
     * 
     * \details Motion Magic® produces a motion profile in real-time while
     * attempting to honor the specified Cruise Velocity, Acceleration,
     * and Jerk value.  This control mode does not use the Expo_kV or
     * Expo_kA configs.  Target position can be changed on-the-fly and
     * Motion Magic® will do its best to adjust the profile.  This control
     * mode is voltage-based, so relevant closed-loop gains will use Volts
     * for the numerator.
     * 
     *   DynamicMotionMagicVoltage Parameters: 
     *    Position: Position to drive toward in rotations.
     *    Velocity: Cruise velocity for profiling.  The signage does not matter as the
     *           device will use the absolute value for profile generation.
     *    Acceleration: Acceleration for profiling.  The signage does not matter as the
     *               device will use the absolute value for profile generation.
     *    Jerk: Jerk for profiling.  The signage does not matter as the device will use
     *       the absolute value for profile generation.
     *    EnableFOC: Set to true to use FOC commutation (requires Phoenix Pro), which
     *            increases peak power by ~15%. Set to false to use trapezoidal
     *            commutation.  FOC improves motor performance by leveraging torque
     *            (current) control.  However, this may be inconvenient for
     *            applications that require specifying duty cycle or voltage. 
     *            CTR-Electronics has developed a hybrid method that combines the
     *            performances gains of FOC while still allowing applications to
     *            provide duty cycle or voltage demand.  This not to be confused
     *            with simple sinusoidal control or phase voltage control which
     *            lacks the performance gains.
     *    FeedForward: Feedforward to apply in volts
     *    Slot: Select which gains are applied by selecting the slot.  Use the
     *       configuration api to set the gain values for the selected slot before
     *       enabling this feature. Slot must be within [0,2].
     *    OverrideBrakeDurNeutral: Set to true to static-brake the rotor when output is
     *                          zero (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0V to the motor.
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::DynamicMotionMagicVoltage&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Requests Motion Magic® to target a final position using a
     * motion profile.  This dynamic request allows runtime changes to
     * Cruise Velocity, Acceleration, and Jerk.  Users can optionally
     * provide a torque current feedforward.  This control requires use of
     * a CANivore.
     * 
     * \details Motion Magic® produces a motion profile in real-time while
     * attempting to honor the specified Cruise Velocity, Acceleration,
     * and Jerk value.  This control mode does not use the Expo_kV or
     * Expo_kA configs.  Target position can be changed on-the-fly and
     * Motion Magic® will do its best to adjust the profile. This control
     * mode is based on torque current, so relevant closed-loop gains will
     * use Amperes for the numerator.
     * 
     *   DynamicMotionMagicTorqueCurrentFOC Parameters: 
     *    Position: Position to drive toward in rotations.
     *    Velocity: Cruise velocity for profiling.  The signage does not matter as the
     *           device will use the absolute value for profile generation.
     *    Acceleration: Acceleration for profiling.  The signage does not matter as the
     *               device will use the absolute value for profile generation.
     *    Jerk: Jerk for profiling.  The signage does not matter as the device will use
     *       the absolute value for profile generation.
     *    FeedForward: Feedforward to apply in torque current in Amperes.  User can use
     *              motor's kT to scale Newton-meter to Amperes.
     *    Slot: Select which gains are applied by selecting the slot.  Use the
     *       configuration api to set the gain values for the selected slot before
     *       enabling this feature. Slot must be within [0,2].
     *    OverrideCoastDurNeutral: Set to true to coast the rotor when output is zero
     *                          (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0A (zero torque).
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::DynamicMotionMagicTorqueCurrentFOC& request);
    /**
     * \brief Requests Motion Magic® to target a final position using a
     * motion profile.  This dynamic request allows runtime changes to
     * Cruise Velocity, Acceleration, and Jerk.  Users can optionally
     * provide a torque current feedforward.  This control requires use of
     * a CANivore.
     * 
     * \details Motion Magic® produces a motion profile in real-time while
     * attempting to honor the specified Cruise Velocity, Acceleration,
     * and Jerk value.  This control mode does not use the Expo_kV or
     * Expo_kA configs.  Target position can be changed on-the-fly and
     * Motion Magic® will do its best to adjust the profile. This control
     * mode is based on torque current, so relevant closed-loop gains will
     * use Amperes for the numerator.
     * 
     *   DynamicMotionMagicTorqueCurrentFOC Parameters: 
     *    Position: Position to drive toward in rotations.
     *    Velocity: Cruise velocity for profiling.  The signage does not matter as the
     *           device will use the absolute value for profile generation.
     *    Acceleration: Acceleration for profiling.  The signage does not matter as the
     *               device will use the absolute value for profile generation.
     *    Jerk: Jerk for profiling.  The signage does not matter as the device will use
     *       the absolute value for profile generation.
     *    FeedForward: Feedforward to apply in torque current in Amperes.  User can use
     *              motor's kT to scale Newton-meter to Amperes.
     *    Slot: Select which gains are applied by selecting the slot.  Use the
     *       configuration api to set the gain values for the selected slot before
     *       enabling this feature. Slot must be within [0,2].
     *    OverrideCoastDurNeutral: Set to true to coast the rotor when output is zero
     *                          (or within deadband).  Set to false to use the
     *                          NeutralMode configuration setting (default). This
     *                          flag exists to provide the fundamental behavior of
     *                          this control when output is zero, which is to
     *                          provide 0A (zero torque).
     *    LimitForwardMotion: Set to true to force forward limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *    LimitReverseMotion: Set to true to force reverse limiting.  This allows users
     *                     to use other limit switch sensors connected to robot
     *                     controller.  This also allows use of active sensors that
     *                     require external power.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::DynamicMotionMagicTorqueCurrentFOC&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Differential control with duty cycle average target and
     * position difference target.
     * 
     *   Diff_DutyCycleOut_Position Parameters: 
     *    AverageRequest: Average DutyCycleOut request of the mechanism.
     *    DifferentialRequest: Differential PositionDutyCycle request of the mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_DutyCycleOut_Position& request);
    /**
     * \brief Differential control with duty cycle average target and
     * position difference target.
     * 
     *   Diff_DutyCycleOut_Position Parameters: 
     *    AverageRequest: Average DutyCycleOut request of the mechanism.
     *    DifferentialRequest: Differential PositionDutyCycle request of the mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_DutyCycleOut_Position&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Differential control with position average target and
     * position difference target using dutycycle control.
     * 
     *   Diff_PositionDutyCycle_Position Parameters: 
     *    AverageRequest: Average PositionDutyCycle request of the mechanism.
     *    DifferentialRequest: Differential PositionDutyCycle request of the mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_PositionDutyCycle_Position& request);
    /**
     * \brief Differential control with position average target and
     * position difference target using dutycycle control.
     * 
     *   Diff_PositionDutyCycle_Position Parameters: 
     *    AverageRequest: Average PositionDutyCycle request of the mechanism.
     *    DifferentialRequest: Differential PositionDutyCycle request of the mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_PositionDutyCycle_Position&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Differential control with velocity average target and
     * position difference target using dutycycle control.
     * 
     *   Diff_VelocityDutyCycle_Position Parameters: 
     *    AverageRequest: Average VelocityDutyCYcle request of the mechanism.
     *    DifferentialRequest: Differential PositionDutyCycle request of the mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_VelocityDutyCycle_Position& request);
    /**
     * \brief Differential control with velocity average target and
     * position difference target using dutycycle control.
     * 
     *   Diff_VelocityDutyCycle_Position Parameters: 
     *    AverageRequest: Average VelocityDutyCYcle request of the mechanism.
     *    DifferentialRequest: Differential PositionDutyCycle request of the mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_VelocityDutyCycle_Position&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Differential control with Motion Magic® average target and
     * position difference target using dutycycle control.
     * 
     *   Diff_MotionMagicDutyCycle_Position Parameters: 
     *    AverageRequest: Average MotionMagicDutyCycle request of the mechanism.
     *    DifferentialRequest: Differential PositionDutyCycle request of the mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_MotionMagicDutyCycle_Position& request);
    /**
     * \brief Differential control with Motion Magic® average target and
     * position difference target using dutycycle control.
     * 
     *   Diff_MotionMagicDutyCycle_Position Parameters: 
     *    AverageRequest: Average MotionMagicDutyCycle request of the mechanism.
     *    DifferentialRequest: Differential PositionDutyCycle request of the mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_MotionMagicDutyCycle_Position&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Differential control with duty cycle average target and
     * velocity difference target.
     * 
     *   Diff_DutyCycleOut_Velocity Parameters: 
     *    AverageRequest: Average DutyCycleOut request of the mechanism.
     *    DifferentialRequest: Differential VelocityDutyCycle request of the mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_DutyCycleOut_Velocity& request);
    /**
     * \brief Differential control with duty cycle average target and
     * velocity difference target.
     * 
     *   Diff_DutyCycleOut_Velocity Parameters: 
     *    AverageRequest: Average DutyCycleOut request of the mechanism.
     *    DifferentialRequest: Differential VelocityDutyCycle request of the mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_DutyCycleOut_Velocity&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Differential control with position average target and
     * velocity difference target using dutycycle control.
     * 
     *   Diff_PositionDutyCycle_Velocity Parameters: 
     *    AverageRequest: Average PositionDutyCycle request of the mechanism.
     *    DifferentialRequest: Differential VelocityDutyCycle request of the mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_PositionDutyCycle_Velocity& request);
    /**
     * \brief Differential control with position average target and
     * velocity difference target using dutycycle control.
     * 
     *   Diff_PositionDutyCycle_Velocity Parameters: 
     *    AverageRequest: Average PositionDutyCycle request of the mechanism.
     *    DifferentialRequest: Differential VelocityDutyCycle request of the mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_PositionDutyCycle_Velocity&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Differential control with velocity average target and
     * velocity difference target using dutycycle control.
     * 
     *   Diff_VelocityDutyCycle_Velocity Parameters: 
     *    AverageRequest: Average VelocityDutyCycle request of the mechanism.
     *    DifferentialRequest: Differential VelocityDutyCycle request of the mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_VelocityDutyCycle_Velocity& request);
    /**
     * \brief Differential control with velocity average target and
     * velocity difference target using dutycycle control.
     * 
     *   Diff_VelocityDutyCycle_Velocity Parameters: 
     *    AverageRequest: Average VelocityDutyCycle request of the mechanism.
     *    DifferentialRequest: Differential VelocityDutyCycle request of the mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_VelocityDutyCycle_Velocity&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Differential control with Motion Magic® average target and
     * velocity difference target using dutycycle control.
     * 
     *   Diff_MotionMagicDutyCycle_Velocity Parameters: 
     *    AverageRequest: Average MotionMagicDutyCycle request of the mechanism.
     *    DifferentialRequest: Differential VelocityDutyCycle request of the mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_MotionMagicDutyCycle_Velocity& request);
    /**
     * \brief Differential control with Motion Magic® average target and
     * velocity difference target using dutycycle control.
     * 
     *   Diff_MotionMagicDutyCycle_Velocity Parameters: 
     *    AverageRequest: Average MotionMagicDutyCycle request of the mechanism.
     *    DifferentialRequest: Differential VelocityDutyCycle request of the mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_MotionMagicDutyCycle_Velocity&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Differential control with voltage average target and
     * position difference target.
     * 
     *   Diff_VoltageOut_Position Parameters: 
     *    AverageRequest: Average VoltageOut request of the mechanism.
     *    DifferentialRequest: Differential PositionVoltage request of the mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_VoltageOut_Position& request);
    /**
     * \brief Differential control with voltage average target and
     * position difference target.
     * 
     *   Diff_VoltageOut_Position Parameters: 
     *    AverageRequest: Average VoltageOut request of the mechanism.
     *    DifferentialRequest: Differential PositionVoltage request of the mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_VoltageOut_Position&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Differential control with position average target and
     * position difference target using voltage control.
     * 
     *   Diff_PositionVoltage_Position Parameters: 
     *    AverageRequest: Average PositionVoltage request of the mechanism.
     *    DifferentialRequest: Differential PositionVoltage request of the mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_PositionVoltage_Position& request);
    /**
     * \brief Differential control with position average target and
     * position difference target using voltage control.
     * 
     *   Diff_PositionVoltage_Position Parameters: 
     *    AverageRequest: Average PositionVoltage request of the mechanism.
     *    DifferentialRequest: Differential PositionVoltage request of the mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_PositionVoltage_Position&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Differential control with velocity average target and
     * position difference target using voltage control.
     * 
     *   Diff_VelocityVoltage_Position Parameters: 
     *    AverageRequest: Average VelocityVoltage request of the mechanism.
     *    DifferentialRequest: Differential PositionVoltage request of the mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_VelocityVoltage_Position& request);
    /**
     * \brief Differential control with velocity average target and
     * position difference target using voltage control.
     * 
     *   Diff_VelocityVoltage_Position Parameters: 
     *    AverageRequest: Average VelocityVoltage request of the mechanism.
     *    DifferentialRequest: Differential PositionVoltage request of the mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_VelocityVoltage_Position&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Differential control with Motion Magic® average target and
     * position difference target using voltage control.
     * 
     *   Diff_MotionMagicVoltage_Position Parameters: 
     *    AverageRequest: Average MotionMagicVoltage request of the mechanism.
     *    DifferentialRequest: Differential PositionVoltage request of the mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_MotionMagicVoltage_Position& request);
    /**
     * \brief Differential control with Motion Magic® average target and
     * position difference target using voltage control.
     * 
     *   Diff_MotionMagicVoltage_Position Parameters: 
     *    AverageRequest: Average MotionMagicVoltage request of the mechanism.
     *    DifferentialRequest: Differential PositionVoltage request of the mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_MotionMagicVoltage_Position&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Differential control with voltage average target and
     * velocity difference target.
     * 
     *   Diff_VoltageOut_Velocity Parameters: 
     *    AverageRequest: Average VoltageOut request of the mechanism.
     *    DifferentialRequest: Differential VelocityVoltage request of the mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_VoltageOut_Velocity& request);
    /**
     * \brief Differential control with voltage average target and
     * velocity difference target.
     * 
     *   Diff_VoltageOut_Velocity Parameters: 
     *    AverageRequest: Average VoltageOut request of the mechanism.
     *    DifferentialRequest: Differential VelocityVoltage request of the mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_VoltageOut_Velocity&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Differential control with position average target and
     * velocity difference target using voltage control.
     * 
     *   Diff_PositionVoltage_Velocity Parameters: 
     *    AverageRequest: Average PositionVoltage request of the mechanism.
     *    DifferentialRequest: Differential VelocityVoltage request of the mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_PositionVoltage_Velocity& request);
    /**
     * \brief Differential control with position average target and
     * velocity difference target using voltage control.
     * 
     *   Diff_PositionVoltage_Velocity Parameters: 
     *    AverageRequest: Average PositionVoltage request of the mechanism.
     *    DifferentialRequest: Differential VelocityVoltage request of the mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_PositionVoltage_Velocity&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Differential control with velocity average target and
     * velocity difference target using voltage control.
     * 
     *   Diff_VelocityVoltage_Velocity Parameters: 
     *    AverageRequest: Average VelocityVoltage request of the mechanism.
     *    DifferentialRequest: Differential VelocityVoltage request of the mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_VelocityVoltage_Velocity& request);
    /**
     * \brief Differential control with velocity average target and
     * velocity difference target using voltage control.
     * 
     *   Diff_VelocityVoltage_Velocity Parameters: 
     *    AverageRequest: Average VelocityVoltage request of the mechanism.
     *    DifferentialRequest: Differential VelocityVoltage request of the mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_VelocityVoltage_Velocity&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Differential control with Motion Magic® average target and
     * velocity difference target using voltage control.
     * 
     *   Diff_MotionMagicVoltage_Velocity Parameters: 
     *    AverageRequest: Average MotionMagicVoltage request of the mechanism.
     *    DifferentialRequest: Differential VelocityVoltage request of the mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_MotionMagicVoltage_Velocity& request);
    /**
     * \brief Differential control with Motion Magic® average target and
     * velocity difference target using voltage control.
     * 
     *   Diff_MotionMagicVoltage_Velocity Parameters: 
     *    AverageRequest: Average MotionMagicVoltage request of the mechanism.
     *    DifferentialRequest: Differential VelocityVoltage request of the mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_MotionMagicVoltage_Velocity&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Differential control with torque current average target and
     * position difference target.
     * 
     *   Diff_TorqueCurrentFOC_Position Parameters: 
     *    AverageRequest: Average TorqueCurrentFOC request of the mechanism.
     *    DifferentialRequest: Differential PositionTorqueCurrentFOC request of the
     *                      mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_TorqueCurrentFOC_Position& request);
    /**
     * \brief Differential control with torque current average target and
     * position difference target.
     * 
     *   Diff_TorqueCurrentFOC_Position Parameters: 
     *    AverageRequest: Average TorqueCurrentFOC request of the mechanism.
     *    DifferentialRequest: Differential PositionTorqueCurrentFOC request of the
     *                      mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_TorqueCurrentFOC_Position&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Differential control with position average target and
     * position difference target using torque current control.
     * 
     *   Diff_PositionTorqueCurrentFOC_Position Parameters: 
     *    AverageRequest: Average PositionTorqueCurrentFOC request of the mechanism.
     *    DifferentialRequest: Differential PositionTorqueCurrentFOC request of the
     *                      mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_PositionTorqueCurrentFOC_Position& request);
    /**
     * \brief Differential control with position average target and
     * position difference target using torque current control.
     * 
     *   Diff_PositionTorqueCurrentFOC_Position Parameters: 
     *    AverageRequest: Average PositionTorqueCurrentFOC request of the mechanism.
     *    DifferentialRequest: Differential PositionTorqueCurrentFOC request of the
     *                      mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_PositionTorqueCurrentFOC_Position&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Differential control with velocity average target and
     * position difference target using torque current control.
     * 
     *   Diff_VelocityTorqueCurrentFOC_Position Parameters: 
     *    AverageRequest: Average VelocityTorqueCurrentFOC request of the mechanism.
     *    DifferentialRequest: Differential PositionTorqueCurrentFOC request of the
     *                      mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_VelocityTorqueCurrentFOC_Position& request);
    /**
     * \brief Differential control with velocity average target and
     * position difference target using torque current control.
     * 
     *   Diff_VelocityTorqueCurrentFOC_Position Parameters: 
     *    AverageRequest: Average VelocityTorqueCurrentFOC request of the mechanism.
     *    DifferentialRequest: Differential PositionTorqueCurrentFOC request of the
     *                      mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_VelocityTorqueCurrentFOC_Position&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Differential control with Motion Magic® average target and
     * position difference target using torque current control.
     * 
     *   Diff_MotionMagicTorqueCurrentFOC_Position Parameters: 
     *    AverageRequest: Average MotionMagicTorqueCurrentFOC request of the mechanism.
     *    DifferentialRequest: Differential PositionTorqueCurrentFOC request of the
     *                      mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_MotionMagicTorqueCurrentFOC_Position& request);
    /**
     * \brief Differential control with Motion Magic® average target and
     * position difference target using torque current control.
     * 
     *   Diff_MotionMagicTorqueCurrentFOC_Position Parameters: 
     *    AverageRequest: Average MotionMagicTorqueCurrentFOC request of the mechanism.
     *    DifferentialRequest: Differential PositionTorqueCurrentFOC request of the
     *                      mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_MotionMagicTorqueCurrentFOC_Position&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Differential control with torque current average target and
     * velocity difference target.
     * 
     *   Diff_TorqueCurrentFOC_Velocity Parameters: 
     *    AverageRequest: Average TorqueCurrentFOC request of the mechanism.
     *    DifferentialRequest: Differential VelocityTorqueCurrentFOC request of the
     *                      mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_TorqueCurrentFOC_Velocity& request);
    /**
     * \brief Differential control with torque current average target and
     * velocity difference target.
     * 
     *   Diff_TorqueCurrentFOC_Velocity Parameters: 
     *    AverageRequest: Average TorqueCurrentFOC request of the mechanism.
     *    DifferentialRequest: Differential VelocityTorqueCurrentFOC request of the
     *                      mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_TorqueCurrentFOC_Velocity&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Differential control with position average target and
     * velocity difference target using torque current control.
     * 
     *   Diff_PositionTorqueCurrentFOC_Velocity Parameters: 
     *    AverageRequest: Average PositionTorqueCurrentFOC request of the mechanism.
     *    DifferentialRequest: Differential VelocityTorqueCurrentFOC request of the
     *                      mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_PositionTorqueCurrentFOC_Velocity& request);
    /**
     * \brief Differential control with position average target and
     * velocity difference target using torque current control.
     * 
     *   Diff_PositionTorqueCurrentFOC_Velocity Parameters: 
     *    AverageRequest: Average PositionTorqueCurrentFOC request of the mechanism.
     *    DifferentialRequest: Differential VelocityTorqueCurrentFOC request of the
     *                      mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_PositionTorqueCurrentFOC_Velocity&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Differential control with velocity average target and
     * velocity difference target using torque current control.
     * 
     *   Diff_VelocityTorqueCurrentFOC_Velocity Parameters: 
     *    AverageRequest: Average VelocityTorqueCurrentFOC request of the mechanism.
     *    DifferentialRequest: Differential VelocityTorqueCurrentFOC request of the
     *                      mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_VelocityTorqueCurrentFOC_Velocity& request);
    /**
     * \brief Differential control with velocity average target and
     * velocity difference target using torque current control.
     * 
     *   Diff_VelocityTorqueCurrentFOC_Velocity Parameters: 
     *    AverageRequest: Average VelocityTorqueCurrentFOC request of the mechanism.
     *    DifferentialRequest: Differential VelocityTorqueCurrentFOC request of the
     *                      mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_VelocityTorqueCurrentFOC_Velocity&& request)
    {
        return SetControl(request);
    }
    
    /**
     * \brief Differential control with Motion Magic® average target and
     * velocity difference target using torque current control.
     * 
     *   Diff_MotionMagicTorqueCurrentFOC_Velocity Parameters: 
     *    AverageRequest: Average MotionMagicTorqueCurrentFOC request of the mechanism.
     *    DifferentialRequest: Differential VelocityTorqueCurrentFOC request of the
     *                      mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_MotionMagicTorqueCurrentFOC_Velocity& request);
    /**
     * \brief Differential control with Motion Magic® average target and
     * velocity difference target using torque current control.
     * 
     *   Diff_MotionMagicTorqueCurrentFOC_Velocity Parameters: 
     *    AverageRequest: Average MotionMagicTorqueCurrentFOC request of the mechanism.
     *    DifferentialRequest: Differential VelocityTorqueCurrentFOC request of the
     *                      mechanism.
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::compound::Diff_MotionMagicTorqueCurrentFOC_Velocity&& request)
    {
        return SetControl(request);
    }

    /**
     * \brief Control motor with generic control request object. User must make
     *        sure the specified object is castable to a valid control request,
     *        otherwise this function will fail at run-time and return the NotSupported
     *        StatusCode
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::ControlRequest& request)
    {
        controls::ControlRequest *ptr = &request;
        (void)ptr;
        auto *DutyCycleOutValue = dynamic_cast<controls::DutyCycleOut *>(ptr);
        if (DutyCycleOutValue != nullptr)
            return SetControl(*DutyCycleOutValue);
        auto *TorqueCurrentFOCValue = dynamic_cast<controls::TorqueCurrentFOC *>(ptr);
        if (TorqueCurrentFOCValue != nullptr)
            return SetControl(*TorqueCurrentFOCValue);
        auto *VoltageOutValue = dynamic_cast<controls::VoltageOut *>(ptr);
        if (VoltageOutValue != nullptr)
            return SetControl(*VoltageOutValue);
        auto *PositionDutyCycleValue = dynamic_cast<controls::PositionDutyCycle *>(ptr);
        if (PositionDutyCycleValue != nullptr)
            return SetControl(*PositionDutyCycleValue);
        auto *PositionVoltageValue = dynamic_cast<controls::PositionVoltage *>(ptr);
        if (PositionVoltageValue != nullptr)
            return SetControl(*PositionVoltageValue);
        auto *PositionTorqueCurrentFOCValue = dynamic_cast<controls::PositionTorqueCurrentFOC *>(ptr);
        if (PositionTorqueCurrentFOCValue != nullptr)
            return SetControl(*PositionTorqueCurrentFOCValue);
        auto *VelocityDutyCycleValue = dynamic_cast<controls::VelocityDutyCycle *>(ptr);
        if (VelocityDutyCycleValue != nullptr)
            return SetControl(*VelocityDutyCycleValue);
        auto *VelocityVoltageValue = dynamic_cast<controls::VelocityVoltage *>(ptr);
        if (VelocityVoltageValue != nullptr)
            return SetControl(*VelocityVoltageValue);
        auto *VelocityTorqueCurrentFOCValue = dynamic_cast<controls::VelocityTorqueCurrentFOC *>(ptr);
        if (VelocityTorqueCurrentFOCValue != nullptr)
            return SetControl(*VelocityTorqueCurrentFOCValue);
        auto *MotionMagicDutyCycleValue = dynamic_cast<controls::MotionMagicDutyCycle *>(ptr);
        if (MotionMagicDutyCycleValue != nullptr)
            return SetControl(*MotionMagicDutyCycleValue);
        auto *MotionMagicVoltageValue = dynamic_cast<controls::MotionMagicVoltage *>(ptr);
        if (MotionMagicVoltageValue != nullptr)
            return SetControl(*MotionMagicVoltageValue);
        auto *MotionMagicTorqueCurrentFOCValue = dynamic_cast<controls::MotionMagicTorqueCurrentFOC *>(ptr);
        if (MotionMagicTorqueCurrentFOCValue != nullptr)
            return SetControl(*MotionMagicTorqueCurrentFOCValue);
        auto *DifferentialDutyCycleValue = dynamic_cast<controls::DifferentialDutyCycle *>(ptr);
        if (DifferentialDutyCycleValue != nullptr)
            return SetControl(*DifferentialDutyCycleValue);
        auto *DifferentialVoltageValue = dynamic_cast<controls::DifferentialVoltage *>(ptr);
        if (DifferentialVoltageValue != nullptr)
            return SetControl(*DifferentialVoltageValue);
        auto *DifferentialPositionDutyCycleValue = dynamic_cast<controls::DifferentialPositionDutyCycle *>(ptr);
        if (DifferentialPositionDutyCycleValue != nullptr)
            return SetControl(*DifferentialPositionDutyCycleValue);
        auto *DifferentialPositionVoltageValue = dynamic_cast<controls::DifferentialPositionVoltage *>(ptr);
        if (DifferentialPositionVoltageValue != nullptr)
            return SetControl(*DifferentialPositionVoltageValue);
        auto *DifferentialVelocityDutyCycleValue = dynamic_cast<controls::DifferentialVelocityDutyCycle *>(ptr);
        if (DifferentialVelocityDutyCycleValue != nullptr)
            return SetControl(*DifferentialVelocityDutyCycleValue);
        auto *DifferentialVelocityVoltageValue = dynamic_cast<controls::DifferentialVelocityVoltage *>(ptr);
        if (DifferentialVelocityVoltageValue != nullptr)
            return SetControl(*DifferentialVelocityVoltageValue);
        auto *DifferentialMotionMagicDutyCycleValue = dynamic_cast<controls::DifferentialMotionMagicDutyCycle *>(ptr);
        if (DifferentialMotionMagicDutyCycleValue != nullptr)
            return SetControl(*DifferentialMotionMagicDutyCycleValue);
        auto *DifferentialMotionMagicVoltageValue = dynamic_cast<controls::DifferentialMotionMagicVoltage *>(ptr);
        if (DifferentialMotionMagicVoltageValue != nullptr)
            return SetControl(*DifferentialMotionMagicVoltageValue);
        auto *FollowerValue = dynamic_cast<controls::Follower *>(ptr);
        if (FollowerValue != nullptr)
            return SetControl(*FollowerValue);
        auto *StrictFollowerValue = dynamic_cast<controls::StrictFollower *>(ptr);
        if (StrictFollowerValue != nullptr)
            return SetControl(*StrictFollowerValue);
        auto *DifferentialFollowerValue = dynamic_cast<controls::DifferentialFollower *>(ptr);
        if (DifferentialFollowerValue != nullptr)
            return SetControl(*DifferentialFollowerValue);
        auto *DifferentialStrictFollowerValue = dynamic_cast<controls::DifferentialStrictFollower *>(ptr);
        if (DifferentialStrictFollowerValue != nullptr)
            return SetControl(*DifferentialStrictFollowerValue);
        auto *NeutralOutValue = dynamic_cast<controls::NeutralOut *>(ptr);
        if (NeutralOutValue != nullptr)
            return SetControl(*NeutralOutValue);
        auto *CoastOutValue = dynamic_cast<controls::CoastOut *>(ptr);
        if (CoastOutValue != nullptr)
            return SetControl(*CoastOutValue);
        auto *StaticBrakeValue = dynamic_cast<controls::StaticBrake *>(ptr);
        if (StaticBrakeValue != nullptr)
            return SetControl(*StaticBrakeValue);
        auto *MusicToneValue = dynamic_cast<controls::MusicTone *>(ptr);
        if (MusicToneValue != nullptr)
            return SetControl(*MusicToneValue);
        auto *MotionMagicVelocityDutyCycleValue = dynamic_cast<controls::MotionMagicVelocityDutyCycle *>(ptr);
        if (MotionMagicVelocityDutyCycleValue != nullptr)
            return SetControl(*MotionMagicVelocityDutyCycleValue);
        auto *MotionMagicVelocityTorqueCurrentFOCValue = dynamic_cast<controls::MotionMagicVelocityTorqueCurrentFOC *>(ptr);
        if (MotionMagicVelocityTorqueCurrentFOCValue != nullptr)
            return SetControl(*MotionMagicVelocityTorqueCurrentFOCValue);
        auto *MotionMagicVelocityVoltageValue = dynamic_cast<controls::MotionMagicVelocityVoltage *>(ptr);
        if (MotionMagicVelocityVoltageValue != nullptr)
            return SetControl(*MotionMagicVelocityVoltageValue);
        auto *MotionMagicExpoDutyCycleValue = dynamic_cast<controls::MotionMagicExpoDutyCycle *>(ptr);
        if (MotionMagicExpoDutyCycleValue != nullptr)
            return SetControl(*MotionMagicExpoDutyCycleValue);
        auto *MotionMagicExpoVoltageValue = dynamic_cast<controls::MotionMagicExpoVoltage *>(ptr);
        if (MotionMagicExpoVoltageValue != nullptr)
            return SetControl(*MotionMagicExpoVoltageValue);
        auto *MotionMagicExpoTorqueCurrentFOCValue = dynamic_cast<controls::MotionMagicExpoTorqueCurrentFOC *>(ptr);
        if (MotionMagicExpoTorqueCurrentFOCValue != nullptr)
            return SetControl(*MotionMagicExpoTorqueCurrentFOCValue);
        auto *DynamicMotionMagicDutyCycleValue = dynamic_cast<controls::DynamicMotionMagicDutyCycle *>(ptr);
        if (DynamicMotionMagicDutyCycleValue != nullptr)
            return SetControl(*DynamicMotionMagicDutyCycleValue);
        auto *DynamicMotionMagicVoltageValue = dynamic_cast<controls::DynamicMotionMagicVoltage *>(ptr);
        if (DynamicMotionMagicVoltageValue != nullptr)
            return SetControl(*DynamicMotionMagicVoltageValue);
        auto *DynamicMotionMagicTorqueCurrentFOCValue = dynamic_cast<controls::DynamicMotionMagicTorqueCurrentFOC *>(ptr);
        if (DynamicMotionMagicTorqueCurrentFOCValue != nullptr)
            return SetControl(*DynamicMotionMagicTorqueCurrentFOCValue);
        auto *Diff_DutyCycleOut_PositionValue = dynamic_cast<controls::compound::Diff_DutyCycleOut_Position *>(ptr);
        if (Diff_DutyCycleOut_PositionValue != nullptr)
            return SetControl(*Diff_DutyCycleOut_PositionValue);
        auto *Diff_PositionDutyCycle_PositionValue = dynamic_cast<controls::compound::Diff_PositionDutyCycle_Position *>(ptr);
        if (Diff_PositionDutyCycle_PositionValue != nullptr)
            return SetControl(*Diff_PositionDutyCycle_PositionValue);
        auto *Diff_VelocityDutyCycle_PositionValue = dynamic_cast<controls::compound::Diff_VelocityDutyCycle_Position *>(ptr);
        if (Diff_VelocityDutyCycle_PositionValue != nullptr)
            return SetControl(*Diff_VelocityDutyCycle_PositionValue);
        auto *Diff_MotionMagicDutyCycle_PositionValue = dynamic_cast<controls::compound::Diff_MotionMagicDutyCycle_Position *>(ptr);
        if (Diff_MotionMagicDutyCycle_PositionValue != nullptr)
            return SetControl(*Diff_MotionMagicDutyCycle_PositionValue);
        auto *Diff_DutyCycleOut_VelocityValue = dynamic_cast<controls::compound::Diff_DutyCycleOut_Velocity *>(ptr);
        if (Diff_DutyCycleOut_VelocityValue != nullptr)
            return SetControl(*Diff_DutyCycleOut_VelocityValue);
        auto *Diff_PositionDutyCycle_VelocityValue = dynamic_cast<controls::compound::Diff_PositionDutyCycle_Velocity *>(ptr);
        if (Diff_PositionDutyCycle_VelocityValue != nullptr)
            return SetControl(*Diff_PositionDutyCycle_VelocityValue);
        auto *Diff_VelocityDutyCycle_VelocityValue = dynamic_cast<controls::compound::Diff_VelocityDutyCycle_Velocity *>(ptr);
        if (Diff_VelocityDutyCycle_VelocityValue != nullptr)
            return SetControl(*Diff_VelocityDutyCycle_VelocityValue);
        auto *Diff_MotionMagicDutyCycle_VelocityValue = dynamic_cast<controls::compound::Diff_MotionMagicDutyCycle_Velocity *>(ptr);
        if (Diff_MotionMagicDutyCycle_VelocityValue != nullptr)
            return SetControl(*Diff_MotionMagicDutyCycle_VelocityValue);
        auto *Diff_VoltageOut_PositionValue = dynamic_cast<controls::compound::Diff_VoltageOut_Position *>(ptr);
        if (Diff_VoltageOut_PositionValue != nullptr)
            return SetControl(*Diff_VoltageOut_PositionValue);
        auto *Diff_PositionVoltage_PositionValue = dynamic_cast<controls::compound::Diff_PositionVoltage_Position *>(ptr);
        if (Diff_PositionVoltage_PositionValue != nullptr)
            return SetControl(*Diff_PositionVoltage_PositionValue);
        auto *Diff_VelocityVoltage_PositionValue = dynamic_cast<controls::compound::Diff_VelocityVoltage_Position *>(ptr);
        if (Diff_VelocityVoltage_PositionValue != nullptr)
            return SetControl(*Diff_VelocityVoltage_PositionValue);
        auto *Diff_MotionMagicVoltage_PositionValue = dynamic_cast<controls::compound::Diff_MotionMagicVoltage_Position *>(ptr);
        if (Diff_MotionMagicVoltage_PositionValue != nullptr)
            return SetControl(*Diff_MotionMagicVoltage_PositionValue);
        auto *Diff_VoltageOut_VelocityValue = dynamic_cast<controls::compound::Diff_VoltageOut_Velocity *>(ptr);
        if (Diff_VoltageOut_VelocityValue != nullptr)
            return SetControl(*Diff_VoltageOut_VelocityValue);
        auto *Diff_PositionVoltage_VelocityValue = dynamic_cast<controls::compound::Diff_PositionVoltage_Velocity *>(ptr);
        if (Diff_PositionVoltage_VelocityValue != nullptr)
            return SetControl(*Diff_PositionVoltage_VelocityValue);
        auto *Diff_VelocityVoltage_VelocityValue = dynamic_cast<controls::compound::Diff_VelocityVoltage_Velocity *>(ptr);
        if (Diff_VelocityVoltage_VelocityValue != nullptr)
            return SetControl(*Diff_VelocityVoltage_VelocityValue);
        auto *Diff_MotionMagicVoltage_VelocityValue = dynamic_cast<controls::compound::Diff_MotionMagicVoltage_Velocity *>(ptr);
        if (Diff_MotionMagicVoltage_VelocityValue != nullptr)
            return SetControl(*Diff_MotionMagicVoltage_VelocityValue);
        auto *Diff_TorqueCurrentFOC_PositionValue = dynamic_cast<controls::compound::Diff_TorqueCurrentFOC_Position *>(ptr);
        if (Diff_TorqueCurrentFOC_PositionValue != nullptr)
            return SetControl(*Diff_TorqueCurrentFOC_PositionValue);
        auto *Diff_PositionTorqueCurrentFOC_PositionValue = dynamic_cast<controls::compound::Diff_PositionTorqueCurrentFOC_Position *>(ptr);
        if (Diff_PositionTorqueCurrentFOC_PositionValue != nullptr)
            return SetControl(*Diff_PositionTorqueCurrentFOC_PositionValue);
        auto *Diff_VelocityTorqueCurrentFOC_PositionValue = dynamic_cast<controls::compound::Diff_VelocityTorqueCurrentFOC_Position *>(ptr);
        if (Diff_VelocityTorqueCurrentFOC_PositionValue != nullptr)
            return SetControl(*Diff_VelocityTorqueCurrentFOC_PositionValue);
        auto *Diff_MotionMagicTorqueCurrentFOC_PositionValue = dynamic_cast<controls::compound::Diff_MotionMagicTorqueCurrentFOC_Position *>(ptr);
        if (Diff_MotionMagicTorqueCurrentFOC_PositionValue != nullptr)
            return SetControl(*Diff_MotionMagicTorqueCurrentFOC_PositionValue);
        auto *Diff_TorqueCurrentFOC_VelocityValue = dynamic_cast<controls::compound::Diff_TorqueCurrentFOC_Velocity *>(ptr);
        if (Diff_TorqueCurrentFOC_VelocityValue != nullptr)
            return SetControl(*Diff_TorqueCurrentFOC_VelocityValue);
        auto *Diff_PositionTorqueCurrentFOC_VelocityValue = dynamic_cast<controls::compound::Diff_PositionTorqueCurrentFOC_Velocity *>(ptr);
        if (Diff_PositionTorqueCurrentFOC_VelocityValue != nullptr)
            return SetControl(*Diff_PositionTorqueCurrentFOC_VelocityValue);
        auto *Diff_VelocityTorqueCurrentFOC_VelocityValue = dynamic_cast<controls::compound::Diff_VelocityTorqueCurrentFOC_Velocity *>(ptr);
        if (Diff_VelocityTorqueCurrentFOC_VelocityValue != nullptr)
            return SetControl(*Diff_VelocityTorqueCurrentFOC_VelocityValue);
        auto *Diff_MotionMagicTorqueCurrentFOC_VelocityValue = dynamic_cast<controls::compound::Diff_MotionMagicTorqueCurrentFOC_Velocity *>(ptr);
        if (Diff_MotionMagicTorqueCurrentFOC_VelocityValue != nullptr)
            return SetControl(*Diff_MotionMagicTorqueCurrentFOC_VelocityValue);
        return ctre::phoenix::StatusCode::NotSupported;
    }
    /**
     * \brief Control motor with generic control request object. User must make
     *        sure the specified object is castable to a valid control request,
     *        otherwise this function will fail at run-time and return the corresponding
     *        StatusCode
     *
     * \param request                Control object to request of the device
     * \returns Status Code of the request, 0 is OK
     */
    ctre::phoenix::StatusCode SetControl(controls::ControlRequest&& request)
    {
        return SetControl(request);
    }

    
    /**
     * \brief Sets the mechanism position of the device in mechanism
     * rotations.
     * 
     * \param newValue Value to set to. Units are in rotations.
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode SetPosition(units::angle::turn_t newValue, units::time::second_t timeoutSeconds)
    {
        return GetConfigurator().SetPosition(newValue, timeoutSeconds);
    }
    /**
     * \brief Sets the mechanism position of the device in mechanism
     * rotations.
     * 
     * This will wait up to 0.050 seconds (50ms) by default.
     * 
     * \param newValue Value to set to. Units are in rotations.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode SetPosition(units::angle::turn_t newValue)
    {
        return SetPosition(newValue, 0.050_s);
    }
    
    /**
     * \brief Clear the sticky faults in the device.
     * 
     * \details This typically has no impact on the device functionality. 
     * Instead, it just clears telemetry faults that are accessible via
     * API and Tuner Self-Test.
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFaults(units::time::second_t timeoutSeconds)
    {
        return GetConfigurator().ClearStickyFaults(timeoutSeconds);
    }
    /**
     * \brief Clear the sticky faults in the device.
     * 
     * \details This typically has no impact on the device functionality. 
     * Instead, it just clears telemetry faults that are accessible via
     * API and Tuner Self-Test.
     * 
     * This will wait up to 0.050 seconds (50ms) by default.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFaults()
    {
        return ClearStickyFaults(0.050_s);
    }
    
    /**
     * \brief Clear sticky fault: Hardware fault occurred
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_Hardware(units::time::second_t timeoutSeconds)
    {
        return GetConfigurator().ClearStickyFault_Hardware(timeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: Hardware fault occurred
     * 
     * This will wait up to 0.050 seconds (50ms) by default.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_Hardware()
    {
        return ClearStickyFault_Hardware(0.050_s);
    }
    
    /**
     * \brief Clear sticky fault: Processor temperature exceeded limit
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_ProcTemp(units::time::second_t timeoutSeconds)
    {
        return GetConfigurator().ClearStickyFault_ProcTemp(timeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: Processor temperature exceeded limit
     * 
     * This will wait up to 0.050 seconds (50ms) by default.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_ProcTemp()
    {
        return ClearStickyFault_ProcTemp(0.050_s);
    }
    
    /**
     * \brief Clear sticky fault: Device temperature exceeded limit
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_DeviceTemp(units::time::second_t timeoutSeconds)
    {
        return GetConfigurator().ClearStickyFault_DeviceTemp(timeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: Device temperature exceeded limit
     * 
     * This will wait up to 0.050 seconds (50ms) by default.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_DeviceTemp()
    {
        return ClearStickyFault_DeviceTemp(0.050_s);
    }
    
    /**
     * \brief Clear sticky fault: Device supply voltage dropped to near
     * brownout levels
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_Undervoltage(units::time::second_t timeoutSeconds)
    {
        return GetConfigurator().ClearStickyFault_Undervoltage(timeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: Device supply voltage dropped to near
     * brownout levels
     * 
     * This will wait up to 0.050 seconds (50ms) by default.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_Undervoltage()
    {
        return ClearStickyFault_Undervoltage(0.050_s);
    }
    
    /**
     * \brief Clear sticky fault: Device boot while detecting the enable
     * signal
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_BootDuringEnable(units::time::second_t timeoutSeconds)
    {
        return GetConfigurator().ClearStickyFault_BootDuringEnable(timeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: Device boot while detecting the enable
     * signal
     * 
     * This will wait up to 0.050 seconds (50ms) by default.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_BootDuringEnable()
    {
        return ClearStickyFault_BootDuringEnable(0.050_s);
    }
    
    /**
     * \brief Clear sticky fault: Bridge was disabled most likely due to
     * supply voltage dropping too low.
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_BridgeBrownout(units::time::second_t timeoutSeconds)
    {
        return GetConfigurator().ClearStickyFault_BridgeBrownout(timeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: Bridge was disabled most likely due to
     * supply voltage dropping too low.
     * 
     * This will wait up to 0.050 seconds (50ms) by default.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_BridgeBrownout()
    {
        return ClearStickyFault_BridgeBrownout(0.050_s);
    }
    
    /**
     * \brief Clear sticky fault: The remote sensor has reset.
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_RemoteSensorReset(units::time::second_t timeoutSeconds)
    {
        return GetConfigurator().ClearStickyFault_RemoteSensorReset(timeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: The remote sensor has reset.
     * 
     * This will wait up to 0.050 seconds (50ms) by default.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_RemoteSensorReset()
    {
        return ClearStickyFault_RemoteSensorReset(0.050_s);
    }
    
    /**
     * \brief Clear sticky fault: The remote Talon FX used for
     * differential control is not present on CAN Bus.
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_MissingDifferentialFX(units::time::second_t timeoutSeconds)
    {
        return GetConfigurator().ClearStickyFault_MissingDifferentialFX(timeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: The remote Talon FX used for
     * differential control is not present on CAN Bus.
     * 
     * This will wait up to 0.050 seconds (50ms) by default.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_MissingDifferentialFX()
    {
        return ClearStickyFault_MissingDifferentialFX(0.050_s);
    }
    
    /**
     * \brief Clear sticky fault: The remote sensor position has
     * overflowed. Because of the nature of remote sensors, it is possible
     * for the remote sensor position to overflow beyond what is supported
     * by the status signal frame. However, this is rare and cannot occur
     * over the course of an FRC match under normal use.
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_RemoteSensorPosOverflow(units::time::second_t timeoutSeconds)
    {
        return GetConfigurator().ClearStickyFault_RemoteSensorPosOverflow(timeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: The remote sensor position has
     * overflowed. Because of the nature of remote sensors, it is possible
     * for the remote sensor position to overflow beyond what is supported
     * by the status signal frame. However, this is rare and cannot occur
     * over the course of an FRC match under normal use.
     * 
     * This will wait up to 0.050 seconds (50ms) by default.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_RemoteSensorPosOverflow()
    {
        return ClearStickyFault_RemoteSensorPosOverflow(0.050_s);
    }
    
    /**
     * \brief Clear sticky fault: Supply Voltage has exceeded the maximum
     * voltage rating of device.
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_OverSupplyV(units::time::second_t timeoutSeconds)
    {
        return GetConfigurator().ClearStickyFault_OverSupplyV(timeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: Supply Voltage has exceeded the maximum
     * voltage rating of device.
     * 
     * This will wait up to 0.050 seconds (50ms) by default.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_OverSupplyV()
    {
        return ClearStickyFault_OverSupplyV(0.050_s);
    }
    
    /**
     * \brief Clear sticky fault: Supply Voltage is unstable.  Ensure you
     * are using a battery and current limited power supply.
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_UnstableSupplyV(units::time::second_t timeoutSeconds)
    {
        return GetConfigurator().ClearStickyFault_UnstableSupplyV(timeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: Supply Voltage is unstable.  Ensure you
     * are using a battery and current limited power supply.
     * 
     * This will wait up to 0.050 seconds (50ms) by default.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_UnstableSupplyV()
    {
        return ClearStickyFault_UnstableSupplyV(0.050_s);
    }
    
    /**
     * \brief Clear sticky fault: Reverse limit switch has been asserted. 
     * Output is set to neutral.
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_ReverseHardLimit(units::time::second_t timeoutSeconds)
    {
        return GetConfigurator().ClearStickyFault_ReverseHardLimit(timeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: Reverse limit switch has been asserted. 
     * Output is set to neutral.
     * 
     * This will wait up to 0.050 seconds (50ms) by default.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_ReverseHardLimit()
    {
        return ClearStickyFault_ReverseHardLimit(0.050_s);
    }
    
    /**
     * \brief Clear sticky fault: Forward limit switch has been asserted. 
     * Output is set to neutral.
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_ForwardHardLimit(units::time::second_t timeoutSeconds)
    {
        return GetConfigurator().ClearStickyFault_ForwardHardLimit(timeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: Forward limit switch has been asserted. 
     * Output is set to neutral.
     * 
     * This will wait up to 0.050 seconds (50ms) by default.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_ForwardHardLimit()
    {
        return ClearStickyFault_ForwardHardLimit(0.050_s);
    }
    
    /**
     * \brief Clear sticky fault: Reverse soft limit has been asserted. 
     * Output is set to neutral.
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_ReverseSoftLimit(units::time::second_t timeoutSeconds)
    {
        return GetConfigurator().ClearStickyFault_ReverseSoftLimit(timeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: Reverse soft limit has been asserted. 
     * Output is set to neutral.
     * 
     * This will wait up to 0.050 seconds (50ms) by default.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_ReverseSoftLimit()
    {
        return ClearStickyFault_ReverseSoftLimit(0.050_s);
    }
    
    /**
     * \brief Clear sticky fault: Forward soft limit has been asserted. 
     * Output is set to neutral.
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_ForwardSoftLimit(units::time::second_t timeoutSeconds)
    {
        return GetConfigurator().ClearStickyFault_ForwardSoftLimit(timeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: Forward soft limit has been asserted. 
     * Output is set to neutral.
     * 
     * This will wait up to 0.050 seconds (50ms) by default.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_ForwardSoftLimit()
    {
        return ClearStickyFault_ForwardSoftLimit(0.050_s);
    }
    
    /**
     * \brief Clear sticky fault: The remote sensor's data is no longer
     * trusted. This can happen if the remote sensor disappears from the
     * CAN bus or if the remote sensor indicates its data is no longer
     * valid, such as when a CANcoder's magnet strength falls into the
     * "red" range.
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_RemoteSensorDataInvalid(units::time::second_t timeoutSeconds)
    {
        return GetConfigurator().ClearStickyFault_RemoteSensorDataInvalid(timeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: The remote sensor's data is no longer
     * trusted. This can happen if the remote sensor disappears from the
     * CAN bus or if the remote sensor indicates its data is no longer
     * valid, such as when a CANcoder's magnet strength falls into the
     * "red" range.
     * 
     * This will wait up to 0.050 seconds (50ms) by default.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_RemoteSensorDataInvalid()
    {
        return ClearStickyFault_RemoteSensorDataInvalid(0.050_s);
    }
    
    /**
     * \brief Clear sticky fault: The remote sensor used for fusion has
     * fallen out of sync to the local sensor. A re-synchronization has
     * occurred, which may cause a discontinuity. This typically happens
     * if there is significant slop in the mechanism, or if the
     * RotorToSensorRatio configuration parameter is incorrect.
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_FusedSensorOutOfSync(units::time::second_t timeoutSeconds)
    {
        return GetConfigurator().ClearStickyFault_FusedSensorOutOfSync(timeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: The remote sensor used for fusion has
     * fallen out of sync to the local sensor. A re-synchronization has
     * occurred, which may cause a discontinuity. This typically happens
     * if there is significant slop in the mechanism, or if the
     * RotorToSensorRatio configuration parameter is incorrect.
     * 
     * This will wait up to 0.050 seconds (50ms) by default.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_FusedSensorOutOfSync()
    {
        return ClearStickyFault_FusedSensorOutOfSync(0.050_s);
    }
    
    /**
     * \brief Clear sticky fault: Stator current limit occured.
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_StatorCurrLimit(units::time::second_t timeoutSeconds)
    {
        return GetConfigurator().ClearStickyFault_StatorCurrLimit(timeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: Stator current limit occured.
     * 
     * This will wait up to 0.050 seconds (50ms) by default.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_StatorCurrLimit()
    {
        return ClearStickyFault_StatorCurrLimit(0.050_s);
    }
    
    /**
     * \brief Clear sticky fault: Supply current limit occured.
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_SupplyCurrLimit(units::time::second_t timeoutSeconds)
    {
        return GetConfigurator().ClearStickyFault_SupplyCurrLimit(timeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: Supply current limit occured.
     * 
     * This will wait up to 0.050 seconds (50ms) by default.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_SupplyCurrLimit()
    {
        return ClearStickyFault_SupplyCurrLimit(0.050_s);
    }
};

}
}

}
}

