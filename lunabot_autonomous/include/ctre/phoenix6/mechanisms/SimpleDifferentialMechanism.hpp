/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
#pragma once

#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/Pigeon2.hpp"
#include "ctre/phoenix6/CANcoder.hpp"
#include "ctre/phoenix6/mechanisms/MechanismState.hpp"
#include <atomic>
#include <optional>

namespace ctre {
namespace phoenix6 {
namespace mechanisms {

/**
 * \brief Manages control of a simple two-axis differential mechanism.
 *
 * This mechanism provides limited differential functionality. Pro users
 * on a CAN FD bus can use the \c DifferentialMechanism class instead
 * for full functionality.
 */
class SimpleDifferentialMechanism {
public:
    /**
     * \brief Sensor sources for a differential Pigeon 2.
     */
    enum class DifferentialPigeon2Source {
        Yaw,
        Pitch,
        Roll
    };

    /**
     * \brief Possible reasons for the mechanism to disable.
     */
    enum class DisabledReason {
        /**
         * \brief No reason given.
         */
        None,
        /**
         * \brief A remote sensor is not present on CAN Bus.
         */
        MissingRemoteSensor,
        /**
         * \brief The remote Talon FX used for differential
         * control is not present on CAN Bus.
         */
        MissingDifferentialFX,
        /**
         * \brief A remote sensor position has overflowed. Because of the
         * nature of remote sensors, it is possible for a remote sensor
         * position to overflow beyond what is supported by the status signal
         * frame. However, this is rare and cannot occur over the course of an
         * FRC match under normal use.
         */
        RemoteSensorPosOverflow,
        /**
         * \brief A device or remote sensor has reset.
         */
        DeviceHasReset,
    };

    /**
     * \brief Possible reasons for the mechanism to require
     * user action to resume control.
     */
    enum class RequiresUserReason {
        /**
         * \brief No reason given.
         */
        None,
        /**
         * \brief A remote sensor position has overflowed. Because of the
         * nature of remote sensors, it is possible for a remote sensor
         * position to overflow beyond what is supported by the status signal
         * frame. However, this is rare and cannot occur over the course of an
         * FRC match under normal use.
         */
        RemoteSensorPosOverflow,
        /**
         * \brief A device or remote sensor has reset.
         */
        DeviceHasReset,
    };

private:
    hardware::TalonFX *_diffAddFX;
    hardware::TalonFX *_diffSubFX;
    std::optional<hardware::Pigeon2 *> _pigeon2;
    DifferentialPigeon2Source _pigeonSource;
    std::optional<hardware::CANcoder *> _cancoder;

    controls::DifferentialFollower _diffFollow;

    controls::NeutralOut _neutral{};
    controls::CoastOut _coast{};
    controls::StaticBrake _brake{};

    std::function<bool()> _diffAddFXResetChecker;
    std::function<bool()> _diffSubFXResetChecker;
    std::optional<std::function<bool()>> _pigeon2ResetChecker;
    std::optional<std::function<bool()>> _cancoderResetChecker;

    bool _hasAppliedConfigs{false};
    ctre::phoenix::StatusCode BeforeControl();

    std::atomic<bool> _mechanismDisabled{false};
    std::atomic<bool> _requiresUserAction{false};

    DisabledReason _disabledReason{DisabledReason::None};
    RequiresUserReason _requiresUserReason{RequiresUserReason::None};

public:
    /**
     * \brief The default number of retries for config applies.
     */
    static constexpr int kDefaultConfigRetries = 5;

    /**
     * \brief Creates a new simple differential mechanism using the given two hardware#TalonFX devices.
     * The mechanism will use the average of the two Talon FX sensors on the primary axis,
     * and the difference between the two Talon FX sensors on the differential axis.
     *
     * This mechanism provides limited differential functionality. Pro users on a CAN FD
     * bus can use the \c DifferentialMechanism class instead for full functionality.
     *
     * \param differentialAddFX The Talon FX that will have the differential output added to its regular output.
     * \param differentialSubFX The Talon FX that will have the differential output subtracted from its regular output.
     * \param motorDirectionsAlign Whether the differential motors' directions are aligned.
     */
    SimpleDifferentialMechanism(hardware::TalonFX &differentialAddFX, hardware::TalonFX &differentialSubFX, bool motorDirectionsAlign) :
        _diffAddFX{&differentialAddFX},
        _diffSubFX{&differentialSubFX},
        _pigeon2{std::nullopt},
        _pigeonSource{},
        _cancoder{std::nullopt},
        _diffFollow{_diffAddFX->GetDeviceID(), !motorDirectionsAlign},
        _diffAddFXResetChecker{_diffAddFX->GetResetOccurredChecker()},
        _diffSubFXResetChecker{_diffSubFX->GetResetOccurredChecker()},
        _pigeon2ResetChecker{std::nullopt},
        _cancoderResetChecker{std::nullopt}
    {}

    /**
     * \brief Creates a new simple differential mechanism using the given two hardware#TalonFX devices and
     * a hardware#Pigeon2. The mechanism will use the average of the two Talon FX sensors on the primary
     * axis, and the selected Pigeon 2 sensor source on the differential axis.
     *
     * This mechanism provides limited differential functionality. Pro users on a CAN FD
     * bus can use the \c DifferentialMechanism class instead for full functionality.
     *
     * \param differentialAddFX The Talon FX that will have the differential output added to its regular output.
     * \param differentialSubFX The Talon FX that will have the differential output subtracted from its regular output.
     * \param motorDirectionsAlign Whether the differential motors' directions are aligned.
     * \param pigeon2 The Pigeon 2 to use for the differential axis.
     * \param pigeonSource The sensor source to use for the Pigeon 2 (Yaw, Pitch, or Roll).
     */
    SimpleDifferentialMechanism(hardware::TalonFX &differentialAddFX, hardware::TalonFX &differentialSubFX, bool motorDirectionsAlign, hardware::Pigeon2 &pigeon2, DifferentialPigeon2Source pigeonSource) :
        _diffAddFX{&differentialAddFX},
        _diffSubFX{&differentialSubFX},
        _pigeon2{&pigeon2},
        _pigeonSource{pigeonSource},
        _cancoder{std::nullopt},
        _diffFollow{_diffAddFX->GetDeviceID(), !motorDirectionsAlign},
        _diffAddFXResetChecker{_diffAddFX->GetResetOccurredChecker()},
        _diffSubFXResetChecker{_diffSubFX->GetResetOccurredChecker()},
        _pigeon2ResetChecker{(*_pigeon2)->GetResetOccurredChecker()},
        _cancoderResetChecker{std::nullopt}
    {}

    /**
     * \brief Creates a new simple differential mechanism using the given two hardware#TalonFX devices and
     * a hardware#CANcoder. The mechanism will use the average of the two Talon FX sensors on the primary
     * axis, and the CANcoder position/velocity on the differential axis.
     *
     * This mechanism provides limited differential functionality. Pro users on a CAN FD
     * bus can use the \c DifferentialMechanism class instead for full functionality.
     *
     * \param differentialAddFX The Talon FX that will have the differential output added to its regular output.
     * \param differentialSubFX The Talon FX that will have the differential output subtracted from its regular output.
     * \param motorDirectionsAlign Whether the differential motors' directions are aligned.
     * \param cancoder The CANcoder to use for the differential axis.
     */
    SimpleDifferentialMechanism(hardware::TalonFX &differentialAddFX, hardware::TalonFX &differentialSubFX, bool motorDirectionsAlign, hardware::CANcoder &cancoder) :
        _diffAddFX{&differentialAddFX},
        _diffSubFX{&differentialSubFX},
        _pigeon2{std::nullopt},
        _pigeonSource{},
        _cancoder{&cancoder},
        _diffFollow{_diffAddFX->GetDeviceID(), !motorDirectionsAlign},
        _diffAddFXResetChecker{_diffAddFX->GetResetOccurredChecker()},
        _diffSubFXResetChecker{_diffSubFX->GetResetOccurredChecker()},
        _pigeon2ResetChecker{std::nullopt},
        _cancoderResetChecker{(*_cancoder)->GetResetOccurredChecker()}
    {}

    /**
     * \brief Get the Talon FX that is differential leader. The differential
     * leader calculates the output for the differential follower. The differential
     * leader is also used for fault detection, and it reports status signals for
     * the differential controller.
     *
     * \returns Differential leader Talon FX
     */
    hardware::TalonFX &GetDifferentialLeader()
    {
        return *_diffAddFX;
    }
    /**
     * \brief Get the Talon FX that is differential leader. The differential
     * leader calculates the output for the differential follower. The differential
     * leader is also useful for fault detection, and it reports status signals for
     * the differential controller.
     *
     * \returns Differential leader Talon FX
     */
    hardware::TalonFX const &GetDifferentialLeader() const
    {
        return *_diffAddFX;
    }

    /**
     * \brief Get the Talon FX that is differential follower. The differential
     * follower's position and velocity are used by the differential leader
     * for the differential controller.
     *
     * \returns Differential follower Talon FX
     */
    hardware::TalonFX &GetDifferentialFollower()
    {
        return *_diffSubFX;
    }
    /**
     * \brief Get the Talon FX that is differential follower. The differential
     * follower's position and velocity are used by the differential leader
     * for the differential controller.
     *
     * \returns Differential follower Talon FX
     */
    hardware::TalonFX const &GetDifferentialFollower() const
    {
        return *_diffSubFX;
    }

    /**
     * \brief Apply the mechanism configs to the devices. This should be
     * called after applying all other configs to the devices.
     *
     * \details If the user does not call this function by the time SetControl
     * is called, SetControl will apply the configs once.
     *
     * \param numRetries Number of retries when applying the configs
     * \returns Status Code of the config applies.
     */
    ctre::phoenix::StatusCode ApplyConfigs(int numRetries = kDefaultConfigRetries);

    /**
     * \brief Call this method periodically to keep the mechanism state updated.
     */
    void Periodic();

    /**
     * \brief Get whether the mechanism is currently disabled due to an issue.
     *
     * \returns true if the mechanism is temporarily disabled
     */
    bool IsDisabled() const
    {
        return _mechanismDisabled.load(std::memory_order_acquire);
    }

    /**
     * \brief Get whether the mechanism is currently disabled and requires
     * user action to re-enable mechanism control.
     *
     * \returns true if the mechanism is disabled and the user must manually
     *          perform an action
     */
    bool RequiresUserAction() const
    {
        return _requiresUserAction.load(std::memory_order_acquire);
    }

    /**
     * \brief Gets the state of the mechanism.
     *
     * \returns MechanismState representing the state of the mechanism
     */
    MechanismState GetMechanismState() const
    {
        if (RequiresUserAction()) {
            return MechanismState::RequiresUserAction;
        } else if (IsDisabled()) {
            return MechanismState::Disabled;
        } else {
            return MechanismState::OK;
        }
    }

    /**
     * \brief Indicate to the mechanism that the user has performed the required
     * action to resume mechanism control.
     */
    void ClearUserRequirement();

    /**
     * \returns The reason for the mechanism being disabled
     */
    DisabledReason GetDisabledReason() const
    {
        return _disabledReason;
    }
    /**
     * \returns The reason for the mechanism requiring user
     *          action to resume control
     */
    RequiresUserReason GetRequiresUserReason() const
    {
        return _requiresUserReason;
    }

    /**
     * \brief Request neutral output of mechanism. The applied brake type
     * is determined by the NeutralMode configuration of each device.
     *
     * \details Since the NeutralMode configuration of devices may not align, users
     * may prefer to use the \c SetCoastOut() or \c SetStaticBrake() method.
     *
     * \returns Status Code of the request.
     */
    ctre::phoenix::StatusCode SetNeutralOut();

    /**
     * \brief Request coast neutral output of mechanism. The bridge is
     * disabled and the rotor is allowed to coast.
     *
     * \returns Status Code of the request.
     */
    ctre::phoenix::StatusCode SetCoastOut();

    /**
     * \brief Applies full neutral-brake on the mechanism by shorting
     * motor leads together.
     *
     * \returns Status Code of the request.
     */
    ctre::phoenix::StatusCode SetStaticBrake();


private:
    

public:
    /**
     * \brief Sets the control request for this mechanism.
     *
     * \param _diffAddFXRequest    Request a specified motor duty cycle with a
     *                             differential position closed-loop.
     * \returns Status Code of the request.
     */
    ctre::phoenix::StatusCode SetControl(controls::DifferentialDutyCycle _diffAddFXRequest);
    /**
     * \brief Sets the control request for this mechanism.
     *
     * \param _diffAddFXRequest    Request a specified voltage with a differential
     *                             position closed-loop.
     * \returns Status Code of the request.
     */
    ctre::phoenix::StatusCode SetControl(controls::DifferentialVoltage _diffAddFXRequest);
    /**
     * \brief Sets the control request for this mechanism.
     *
     * \param _diffAddFXRequest    Request PID to target position with a
     *                             differential position setpoint.
     * \returns Status Code of the request.
     */
    ctre::phoenix::StatusCode SetControl(controls::DifferentialPositionDutyCycle _diffAddFXRequest);
    /**
     * \brief Sets the control request for this mechanism.
     *
     * \param _diffAddFXRequest    Request PID to target position with a
     *                             differential position setpoint
     * \returns Status Code of the request.
     */
    ctre::phoenix::StatusCode SetControl(controls::DifferentialPositionVoltage _diffAddFXRequest);
    /**
     * \brief Sets the control request for this mechanism.
     *
     * \param _diffAddFXRequest    Request PID to target velocity with a
     *                             differential position setpoint.
     * \returns Status Code of the request.
     */
    ctre::phoenix::StatusCode SetControl(controls::DifferentialVelocityDutyCycle _diffAddFXRequest);
    /**
     * \brief Sets the control request for this mechanism.
     *
     * \param _diffAddFXRequest    Request PID to target velocity with a
     *                             differential position setpoint.
     * \returns Status Code of the request.
     */
    ctre::phoenix::StatusCode SetControl(controls::DifferentialVelocityVoltage _diffAddFXRequest);
    /**
     * \brief Sets the control request for this mechanism.
     *
     * \param _diffAddFXRequest    Requests Motion Magic® to target a final position
     *                             using a motion profile, and PID to a differential
     *                             position setpoint.
     * \returns Status Code of the request.
     */
    ctre::phoenix::StatusCode SetControl(controls::DifferentialMotionMagicDutyCycle _diffAddFXRequest);
    /**
     * \brief Sets the control request for this mechanism.
     *
     * \param _diffAddFXRequest    Requests Motion Magic® to target a final position
     *                             using a motion profile, and PID to a differential
     *                             position setpoint.
     * \returns Status Code of the request.
     */
    ctre::phoenix::StatusCode SetControl(controls::DifferentialMotionMagicVoltage _diffAddFXRequest);
};

}
}
}
