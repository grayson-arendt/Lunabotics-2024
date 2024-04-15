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
 * \brief Manages control of a two-axis differential mechanism.
 *
 * This mechanism requires the devices to be Pro licensed and
 * connected to a CAN FD bus. Unlicensed users and users on a
 * CAN 2.0 bus can use the \c SimpleDifferentialMechanism instead
 * with limited functionality.
 */
class DifferentialMechanism {
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
     * \brief Creates a new differential mechanism using the given two hardware#TalonFX devices.
     * The mechanism will use the average of the two Talon FX sensors on the primary axis,
     * and the difference between the two Talon FX sensors on the differential axis.
     *
     * This mechanism requires the devices to be Pro licensed and connected to a CAN FD bus.
     * Unlicensed users and users on a CAN 2.0 bus can use the \c SimpleDifferentialMechanism
     * instead with limited functionality.
     *
     * \param differentialAddFX The Talon FX that will have the differential output added to its regular output.
     * \param differentialSubFX The Talon FX that will have the differential output subtracted from its regular output.
     * \param motorDirectionsAlign Whether the differential motors' directions are aligned.
     */
    DifferentialMechanism(hardware::TalonFX &differentialAddFX, hardware::TalonFX &differentialSubFX, bool motorDirectionsAlign) :
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
     * \brief Creates a new differential mechanism using the given two hardware#TalonFX devices and
     * a hardware#Pigeon2. The mechanism will use the average of the two Talon FX sensors on the primary
     * axis, and the selected Pigeon 2 sensor source on the differential axis.
     *
     * This mechanism requires the devices to be Pro licensed and connected to a CAN FD bus.
     * Unlicensed users and users on a CAN 2.0 bus can use the \c SimpleDifferentialMechanism
     * instead with limited functionality.
     *
     * \param differentialAddFX The Talon FX that will have the differential output added to its regular output.
     * \param differentialSubFX The Talon FX that will have the differential output subtracted from its regular output.
     * \param motorDirectionsAlign Whether the differential motors' directions are aligned.
     * \param pigeon2 The Pigeon 2 to use for the differential axis.
     * \param pigeonSource The sensor source to use for the Pigeon 2 (Yaw, Pitch, or Roll).
     */
    DifferentialMechanism(hardware::TalonFX &differentialAddFX, hardware::TalonFX &differentialSubFX, bool motorDirectionsAlign, hardware::Pigeon2 &pigeon2, DifferentialPigeon2Source pigeonSource) :
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
     * \brief Creates a new differential mechanism using the given two hardware#TalonFX devices and
     * a hardware#CANcoder. The mechanism will use the average of the two Talon FX sensors on the primary
     * axis, and the CANcoder position/velocity on the differential axis.
     *
     * This mechanism requires the devices to be Pro licensed and connected to a CAN FD bus.
     * Unlicensed users and users on a CAN 2.0 bus can use the \c SimpleDifferentialMechanism
     * instead with limited functionality.
     *
     * \param differentialAddFX The Talon FX that will have the differential output added to its regular output.
     * \param differentialSubFX The Talon FX that will have the differential output subtracted from its regular output.
     * \param motorDirectionsAlign Whether the differential motors' directions are aligned.
     * \param cancoder The CANcoder to use for the differential axis.
     */
    DifferentialMechanism(hardware::TalonFX &differentialAddFX, hardware::TalonFX &differentialSubFX, bool motorDirectionsAlign, hardware::CANcoder &cancoder) :
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
    std::unique_ptr<controls::ControlRequest> _diffAddFX_req{};

public:
    /**
     * \brief Sets the control request for this mechanism.
     *
     * \param AverageRequest    Average DutyCycleOut request of the mechanism.
     * \param DifferentialRequest    Differential PositionDutyCycle request of the
     *                               mechanism. Note: The UpdateFreqHz parameter for
     *                               this control request will be ignored by the
     *                               control frame.
     * \returns Status Code of the request.
     */
    ctre::phoenix::StatusCode SetControl(controls::DutyCycleOut AverageRequest, controls::PositionDutyCycle DifferentialRequest);
    /**
     * \brief Sets the control request for this mechanism.
     *
     * \param AverageRequest    Average PositionDutyCycle request of the mechanism.
     * \param DifferentialRequest    Differential PositionDutyCycle request of the
     *                               mechanism. Note: The UpdateFreqHz parameter for
     *                               this control request will be ignored by the
     *                               control frame.
     * \returns Status Code of the request.
     */
    ctre::phoenix::StatusCode SetControl(controls::PositionDutyCycle AverageRequest, controls::PositionDutyCycle DifferentialRequest);
    /**
     * \brief Sets the control request for this mechanism.
     *
     * \param AverageRequest    Average VelocityDutyCYcle request of the mechanism.
     * \param DifferentialRequest    Differential PositionDutyCycle request of the
     *                               mechanism. Note: The UpdateFreqHz parameter for
     *                               this control request will be ignored by the
     *                               control frame.
     * \returns Status Code of the request.
     */
    ctre::phoenix::StatusCode SetControl(controls::VelocityDutyCycle AverageRequest, controls::PositionDutyCycle DifferentialRequest);
    /**
     * \brief Sets the control request for this mechanism.
     *
     * \param AverageRequest    Average MotionMagicDutyCycle request of the
     *                          mechanism.
     * \param DifferentialRequest    Differential PositionDutyCycle request of the
     *                               mechanism. Note: The UpdateFreqHz parameter for
     *                               this control request will be ignored by the
     *                               control frame.
     * \returns Status Code of the request.
     */
    ctre::phoenix::StatusCode SetControl(controls::MotionMagicDutyCycle AverageRequest, controls::PositionDutyCycle DifferentialRequest);
    /**
     * \brief Sets the control request for this mechanism.
     *
     * \param AverageRequest    Average DutyCycleOut request of the mechanism.
     * \param DifferentialRequest    Differential VelocityDutyCycle request of the
     *                               mechanism. Note: The UpdateFreqHz parameter for
     *                               this control request will be ignored by the
     *                               control frame.
     * \returns Status Code of the request.
     */
    ctre::phoenix::StatusCode SetControl(controls::DutyCycleOut AverageRequest, controls::VelocityDutyCycle DifferentialRequest);
    /**
     * \brief Sets the control request for this mechanism.
     *
     * \param AverageRequest    Average PositionDutyCycle request of the mechanism.
     * \param DifferentialRequest    Differential VelocityDutyCycle request of the
     *                               mechanism. Note: The UpdateFreqHz parameter for
     *                               this control request will be ignored by the
     *                               control frame.
     * \returns Status Code of the request.
     */
    ctre::phoenix::StatusCode SetControl(controls::PositionDutyCycle AverageRequest, controls::VelocityDutyCycle DifferentialRequest);
    /**
     * \brief Sets the control request for this mechanism.
     *
     * \param AverageRequest    Average VelocityDutyCycle request of the mechanism.
     * \param DifferentialRequest    Differential VelocityDutyCycle request of the
     *                               mechanism. Note: The UpdateFreqHz parameter for
     *                               this control request will be ignored by the
     *                               control frame.
     * \returns Status Code of the request.
     */
    ctre::phoenix::StatusCode SetControl(controls::VelocityDutyCycle AverageRequest, controls::VelocityDutyCycle DifferentialRequest);
    /**
     * \brief Sets the control request for this mechanism.
     *
     * \param AverageRequest    Average MotionMagicDutyCycle request of the
     *                          mechanism.
     * \param DifferentialRequest    Differential VelocityDutyCycle request of the
     *                               mechanism. Note: The UpdateFreqHz parameter for
     *                               this control request will be ignored by the
     *                               control frame.
     * \returns Status Code of the request.
     */
    ctre::phoenix::StatusCode SetControl(controls::MotionMagicDutyCycle AverageRequest, controls::VelocityDutyCycle DifferentialRequest);
    /**
     * \brief Sets the control request for this mechanism.
     *
     * \param AverageRequest    Average VoltageOut request of the mechanism.
     * \param DifferentialRequest    Differential PositionVoltage request of the
     *                               mechanism. Note: The UpdateFreqHz parameter for
     *                               this control request will be ignored by the
     *                               control frame.
     * \returns Status Code of the request.
     */
    ctre::phoenix::StatusCode SetControl(controls::VoltageOut AverageRequest, controls::PositionVoltage DifferentialRequest);
    /**
     * \brief Sets the control request for this mechanism.
     *
     * \param AverageRequest    Average PositionVoltage request of the mechanism.
     * \param DifferentialRequest    Differential PositionVoltage request of the
     *                               mechanism. Note: The UpdateFreqHz parameter for
     *                               this control request will be ignored by the
     *                               control frame.
     * \returns Status Code of the request.
     */
    ctre::phoenix::StatusCode SetControl(controls::PositionVoltage AverageRequest, controls::PositionVoltage DifferentialRequest);
    /**
     * \brief Sets the control request for this mechanism.
     *
     * \param AverageRequest    Average VelocityVoltage request of the mechanism.
     * \param DifferentialRequest    Differential PositionVoltage request of the
     *                               mechanism. Note: The UpdateFreqHz parameter for
     *                               this control request will be ignored by the
     *                               control frame.
     * \returns Status Code of the request.
     */
    ctre::phoenix::StatusCode SetControl(controls::VelocityVoltage AverageRequest, controls::PositionVoltage DifferentialRequest);
    /**
     * \brief Sets the control request for this mechanism.
     *
     * \param AverageRequest    Average MotionMagicVoltage request of the mechanism.
     * \param DifferentialRequest    Differential PositionVoltage request of the
     *                               mechanism. Note: The UpdateFreqHz parameter for
     *                               this control request will be ignored by the
     *                               control frame.
     * \returns Status Code of the request.
     */
    ctre::phoenix::StatusCode SetControl(controls::MotionMagicVoltage AverageRequest, controls::PositionVoltage DifferentialRequest);
    /**
     * \brief Sets the control request for this mechanism.
     *
     * \param AverageRequest    Average VoltageOut request of the mechanism.
     * \param DifferentialRequest    Differential VelocityVoltage request of the
     *                               mechanism. Note: The UpdateFreqHz parameter for
     *                               this control request will be ignored by the
     *                               control frame.
     * \returns Status Code of the request.
     */
    ctre::phoenix::StatusCode SetControl(controls::VoltageOut AverageRequest, controls::VelocityVoltage DifferentialRequest);
    /**
     * \brief Sets the control request for this mechanism.
     *
     * \param AverageRequest    Average PositionVoltage request of the mechanism.
     * \param DifferentialRequest    Differential VelocityVoltage request of the
     *                               mechanism. Note: The UpdateFreqHz parameter for
     *                               this control request will be ignored by the
     *                               control frame.
     * \returns Status Code of the request.
     */
    ctre::phoenix::StatusCode SetControl(controls::PositionVoltage AverageRequest, controls::VelocityVoltage DifferentialRequest);
    /**
     * \brief Sets the control request for this mechanism.
     *
     * \param AverageRequest    Average VelocityVoltage request of the mechanism.
     * \param DifferentialRequest    Differential VelocityVoltage request of the
     *                               mechanism. Note: The UpdateFreqHz parameter for
     *                               this control request will be ignored by the
     *                               control frame.
     * \returns Status Code of the request.
     */
    ctre::phoenix::StatusCode SetControl(controls::VelocityVoltage AverageRequest, controls::VelocityVoltage DifferentialRequest);
    /**
     * \brief Sets the control request for this mechanism.
     *
     * \param AverageRequest    Average MotionMagicVoltage request of the mechanism.
     * \param DifferentialRequest    Differential VelocityVoltage request of the
     *                               mechanism. Note: The UpdateFreqHz parameter for
     *                               this control request will be ignored by the
     *                               control frame.
     * \returns Status Code of the request.
     */
    ctre::phoenix::StatusCode SetControl(controls::MotionMagicVoltage AverageRequest, controls::VelocityVoltage DifferentialRequest);
    /**
     * \brief Sets the control request for this mechanism.
     *
     * \param AverageRequest    Average TorqueCurrentFOC request of the mechanism.
     * \param DifferentialRequest    Differential PositionTorqueCurrentFOC request
     *                               of the mechanism. Note: The UpdateFreqHz
     *                               parameter for this control request will be
     *                               ignored by the control frame.
     * \returns Status Code of the request.
     */
    ctre::phoenix::StatusCode SetControl(controls::TorqueCurrentFOC AverageRequest, controls::PositionTorqueCurrentFOC DifferentialRequest);
    /**
     * \brief Sets the control request for this mechanism.
     *
     * \param AverageRequest    Average PositionTorqueCurrentFOC request of the
     *                          mechanism.
     * \param DifferentialRequest    Differential PositionTorqueCurrentFOC request
     *                               of the mechanism. Note: The UpdateFreqHz
     *                               parameter for this control request will be
     *                               ignored by the control frame.
     * \returns Status Code of the request.
     */
    ctre::phoenix::StatusCode SetControl(controls::PositionTorqueCurrentFOC AverageRequest, controls::PositionTorqueCurrentFOC DifferentialRequest);
    /**
     * \brief Sets the control request for this mechanism.
     *
     * \param AverageRequest    Average VelocityTorqueCurrentFOC request of the
     *                          mechanism.
     * \param DifferentialRequest    Differential PositionTorqueCurrentFOC request
     *                               of the mechanism. Note: The UpdateFreqHz
     *                               parameter for this control request will be
     *                               ignored by the control frame.
     * \returns Status Code of the request.
     */
    ctre::phoenix::StatusCode SetControl(controls::VelocityTorqueCurrentFOC AverageRequest, controls::PositionTorqueCurrentFOC DifferentialRequest);
    /**
     * \brief Sets the control request for this mechanism.
     *
     * \param AverageRequest    Average MotionMagicTorqueCurrentFOC request of the
     *                          mechanism.
     * \param DifferentialRequest    Differential PositionTorqueCurrentFOC request
     *                               of the mechanism. Note: The UpdateFreqHz
     *                               parameter for this control request will be
     *                               ignored by the control frame.
     * \returns Status Code of the request.
     */
    ctre::phoenix::StatusCode SetControl(controls::MotionMagicTorqueCurrentFOC AverageRequest, controls::PositionTorqueCurrentFOC DifferentialRequest);
    /**
     * \brief Sets the control request for this mechanism.
     *
     * \param AverageRequest    Average TorqueCurrentFOC request of the mechanism.
     * \param DifferentialRequest    Differential VelocityTorqueCurrentFOC request
     *                               of the mechanism. Note: The UpdateFreqHz
     *                               parameter for this control request will be
     *                               ignored by the control frame.
     * \returns Status Code of the request.
     */
    ctre::phoenix::StatusCode SetControl(controls::TorqueCurrentFOC AverageRequest, controls::VelocityTorqueCurrentFOC DifferentialRequest);
    /**
     * \brief Sets the control request for this mechanism.
     *
     * \param AverageRequest    Average PositionTorqueCurrentFOC request of the
     *                          mechanism.
     * \param DifferentialRequest    Differential VelocityTorqueCurrentFOC request
     *                               of the mechanism. Note: The UpdateFreqHz
     *                               parameter for this control request will be
     *                               ignored by the control frame.
     * \returns Status Code of the request.
     */
    ctre::phoenix::StatusCode SetControl(controls::PositionTorqueCurrentFOC AverageRequest, controls::VelocityTorqueCurrentFOC DifferentialRequest);
    /**
     * \brief Sets the control request for this mechanism.
     *
     * \param AverageRequest    Average VelocityTorqueCurrentFOC request of the
     *                          mechanism.
     * \param DifferentialRequest    Differential VelocityTorqueCurrentFOC request
     *                               of the mechanism. Note: The UpdateFreqHz
     *                               parameter for this control request will be
     *                               ignored by the control frame.
     * \returns Status Code of the request.
     */
    ctre::phoenix::StatusCode SetControl(controls::VelocityTorqueCurrentFOC AverageRequest, controls::VelocityTorqueCurrentFOC DifferentialRequest);
    /**
     * \brief Sets the control request for this mechanism.
     *
     * \param AverageRequest    Average MotionMagicTorqueCurrentFOC request of the
     *                          mechanism.
     * \param DifferentialRequest    Differential VelocityTorqueCurrentFOC request
     *                               of the mechanism. Note: The UpdateFreqHz
     *                               parameter for this control request will be
     *                               ignored by the control frame.
     * \returns Status Code of the request.
     */
    ctre::phoenix::StatusCode SetControl(controls::MotionMagicTorqueCurrentFOC AverageRequest, controls::VelocityTorqueCurrentFOC DifferentialRequest);
};

}
}
}
