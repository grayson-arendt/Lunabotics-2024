/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
#pragma once

#include "ctre/phoenix/StatusCodes.h"
#include "ctre/phoenix6/Serializable.hpp"
#include "ctre/phoenix6/networking/interfaces/ConfigSerializer.h"
#include "ctre/phoenix6/signals/SpnEnums.hpp"
#include "ctre/phoenix6/spns/SpnValue.hpp"
#include <sstream>
#include <map>
#include <string>

namespace ctre {
namespace phoenix6 {

namespace hardware { namespace core { class CoreCANcoder; } }
namespace configs { class SlotConfigs; }

namespace configs {

class ParentConfiguration : public ISerializable
{
public:
    virtual std::string ToString() const = 0;
    friend std::ostream &operator<<(std::ostream &str, const ParentConfiguration &v)
    {
        str << v.ToString();
        return str;
    }
    virtual ctre::phoenix::StatusCode Deserialize(const std::string &string) = 0;
};


/**
 * \brief Configs that affect the magnet sensor and how to interpret
 *        it.
 * 
 * \details Includes sensor range and other configs related to sensor.
 */
class MagnetSensorConfigs : public ParentConfiguration
{
public:
    /**
     * \brief Direction of the sensor to determine positive facing the LED
     * side of the CANcoder.
     * 
     */
    signals::SensorDirectionValue SensorDirection = signals::SensorDirectionValue::CounterClockwise_Positive;
    /**
     * \brief This offset is added to the reported position, allowing the
     * application to trim the zero position.  When set to the default
     * value of zero, position reports zero when magnet north pole aligns
     * with the LED.
     * 
     *   Minimum Value: -1
     *   Maximum Value: 1
     *   Default Value: 0
     *   Units: rotations
     */
    double MagnetOffset = 0;
    /**
     * \brief The range of the absolute sensor, either [-0.5, 0.5) or [0,
     * 1).
     * 
     */
    signals::AbsoluteSensorRangeValue AbsoluteSensorRange = signals::AbsoluteSensorRangeValue::Signed_PlusMinusHalf;
    
    /**
     * \brief Modifies this configuration's SensorDirection parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newSensorDirection Parameter to modify
     * \returns Itself
     */
    MagnetSensorConfigs& WithSensorDirection(signals::SensorDirectionValue newSensorDirection)
    {
        SensorDirection = std::move(newSensorDirection);
        return *this;
    }
    /**
     * \brief Modifies this configuration's MagnetOffset parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newMagnetOffset Parameter to modify
     * \returns Itself
     */
    MagnetSensorConfigs& WithMagnetOffset(double newMagnetOffset)
    {
        MagnetOffset = std::move(newMagnetOffset);
        return *this;
    }
    /**
     * \brief Modifies this configuration's AbsoluteSensorRange parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newAbsoluteSensorRange Parameter to modify
     * \returns Itself
     */
    MagnetSensorConfigs& WithAbsoluteSensorRange(signals::AbsoluteSensorRangeValue newAbsoluteSensorRange)
    {
        AbsoluteSensorRange = std::move(newAbsoluteSensorRange);
        return *this;
    }

    

    std::string ToString() const override
    {
        std::stringstream ss;
        ss << "{" << std::endl;
        ss << "Config Group: MagnetSensor" << std::endl;
        ss << "Name: \"SensorDirection\" Value: \"" << SensorDirection << "\"" << std::endl;
        ss << "Name: \"MagnetOffset\" Value: \"" << MagnetOffset << "rotations\"" << std::endl;
        ss << "Name: \"AbsoluteSensorRange\" Value: \"" << AbsoluteSensorRange << "\"" << std::endl;
        ss << "}" << std::endl;
        return ss.str();
    }

    std::string Serialize() const override
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_int(ctre::phoenix6::spns::SpnValue::CANcoder_SensorDirection, SensorDirection.value, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::CANCoder_MagnetOffset, MagnetOffset, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_int(ctre::phoenix6::spns::SpnValue::CANcoder_AbsoluteSensorRange, AbsoluteSensorRange.value, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return ss.str();
    }

    ctre::phoenix::StatusCode Deserialize(const std::string& to_deserialize) override
    {
        const char *string_c_str = to_deserialize.c_str();
        size_t string_length = to_deserialize.length();
        c_ctre_phoenix6_deserialize_int(ctre::phoenix6::spns::SpnValue::CANcoder_SensorDirection, string_c_str, string_length, &SensorDirection.value);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::CANCoder_MagnetOffset, string_c_str, string_length, &MagnetOffset);
        c_ctre_phoenix6_deserialize_int(ctre::phoenix6::spns::SpnValue::CANcoder_AbsoluteSensorRange, string_c_str, string_length, &AbsoluteSensorRange.value);
        return 0;
    }
};


/**
 * \brief Configs for Pigeon 2's Mount Pose configuration.
 * 
 * \details These configs allow the Pigeon2 to be mounted in whatever
 *          orientation that's desired and ensure the reported
 *          Yaw/Pitch/Roll is from the robot's reference.
 */
class MountPoseConfigs : public ParentConfiguration
{
public:
    /**
     * \brief The mounting calibration yaw-component
     * 
     *   Minimum Value: -360
     *   Maximum Value: 360
     *   Default Value: 0
     *   Units: deg
     */
    double MountPoseYaw = 0;
    /**
     * \brief The mounting calibration pitch-component
     * 
     *   Minimum Value: -360
     *   Maximum Value: 360
     *   Default Value: 0
     *   Units: deg
     */
    double MountPosePitch = 0;
    /**
     * \brief The mounting calibration roll-component
     * 
     *   Minimum Value: -360
     *   Maximum Value: 360
     *   Default Value: 0
     *   Units: deg
     */
    double MountPoseRoll = 0;
    
    /**
     * \brief Modifies this configuration's MountPoseYaw parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newMountPoseYaw Parameter to modify
     * \returns Itself
     */
    MountPoseConfigs& WithMountPoseYaw(double newMountPoseYaw)
    {
        MountPoseYaw = std::move(newMountPoseYaw);
        return *this;
    }
    /**
     * \brief Modifies this configuration's MountPosePitch parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newMountPosePitch Parameter to modify
     * \returns Itself
     */
    MountPoseConfigs& WithMountPosePitch(double newMountPosePitch)
    {
        MountPosePitch = std::move(newMountPosePitch);
        return *this;
    }
    /**
     * \brief Modifies this configuration's MountPoseRoll parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newMountPoseRoll Parameter to modify
     * \returns Itself
     */
    MountPoseConfigs& WithMountPoseRoll(double newMountPoseRoll)
    {
        MountPoseRoll = std::move(newMountPoseRoll);
        return *this;
    }

    

    std::string ToString() const override
    {
        std::stringstream ss;
        ss << "{" << std::endl;
        ss << "Config Group: MountPose" << std::endl;
        ss << "Name: \"MountPoseYaw\" Value: \"" << MountPoseYaw << "deg\"" << std::endl;
        ss << "Name: \"MountPosePitch\" Value: \"" << MountPosePitch << "deg\"" << std::endl;
        ss << "Name: \"MountPoseRoll\" Value: \"" << MountPoseRoll << "deg\"" << std::endl;
        ss << "}" << std::endl;
        return ss.str();
    }

    std::string Serialize() const override
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Pigeon2MountPoseYaw, MountPoseYaw, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Pigeon2MountPosePitch, MountPosePitch, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Pigeon2MountPoseRoll, MountPoseRoll, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return ss.str();
    }

    ctre::phoenix::StatusCode Deserialize(const std::string& to_deserialize) override
    {
        const char *string_c_str = to_deserialize.c_str();
        size_t string_length = to_deserialize.length();
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Pigeon2MountPoseYaw, string_c_str, string_length, &MountPoseYaw);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Pigeon2MountPosePitch, string_c_str, string_length, &MountPosePitch);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Pigeon2MountPoseRoll, string_c_str, string_length, &MountPoseRoll);
        return 0;
    }
};


/**
 * \brief Configs to trim the Pigeon2's gyroscope.
 * 
 * \details Pigeon2 allows the user to trim the gyroscope's
 *          sensitivity. While this isn't necessary for the Pigeon2,
 *          as it comes calibrated out-of-the-box, users can make use
 *          of this to make the Pigeon2 even more accurate for their
 *          application.
 */
class GyroTrimConfigs : public ParentConfiguration
{
public:
    /**
     * \brief The gyro scalar component for the X axis
     * 
     *   Minimum Value: -180
     *   Maximum Value: 180
     *   Default Value: 0
     *   Units: deg per rotation
     */
    double GyroScalarX = 0;
    /**
     * \brief The gyro scalar component for the Y axis
     * 
     *   Minimum Value: -180
     *   Maximum Value: 180
     *   Default Value: 0
     *   Units: deg per rotation
     */
    double GyroScalarY = 0;
    /**
     * \brief The gyro scalar component for the Z axis
     * 
     *   Minimum Value: -180
     *   Maximum Value: 180
     *   Default Value: 0
     *   Units: deg per rotation
     */
    double GyroScalarZ = 0;
    
    /**
     * \brief Modifies this configuration's GyroScalarX parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newGyroScalarX Parameter to modify
     * \returns Itself
     */
    GyroTrimConfigs& WithGyroScalarX(double newGyroScalarX)
    {
        GyroScalarX = std::move(newGyroScalarX);
        return *this;
    }
    /**
     * \brief Modifies this configuration's GyroScalarY parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newGyroScalarY Parameter to modify
     * \returns Itself
     */
    GyroTrimConfigs& WithGyroScalarY(double newGyroScalarY)
    {
        GyroScalarY = std::move(newGyroScalarY);
        return *this;
    }
    /**
     * \brief Modifies this configuration's GyroScalarZ parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newGyroScalarZ Parameter to modify
     * \returns Itself
     */
    GyroTrimConfigs& WithGyroScalarZ(double newGyroScalarZ)
    {
        GyroScalarZ = std::move(newGyroScalarZ);
        return *this;
    }

    

    std::string ToString() const override
    {
        std::stringstream ss;
        ss << "{" << std::endl;
        ss << "Config Group: GyroTrim" << std::endl;
        ss << "Name: \"GyroScalarX\" Value: \"" << GyroScalarX << "deg per rotation\"" << std::endl;
        ss << "Name: \"GyroScalarY\" Value: \"" << GyroScalarY << "deg per rotation\"" << std::endl;
        ss << "Name: \"GyroScalarZ\" Value: \"" << GyroScalarZ << "deg per rotation\"" << std::endl;
        ss << "}" << std::endl;
        return ss.str();
    }

    std::string Serialize() const override
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Pigeon2GyroScalarX, GyroScalarX, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Pigeon2GyroScalarY, GyroScalarY, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Pigeon2GyroScalarZ, GyroScalarZ, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return ss.str();
    }

    ctre::phoenix::StatusCode Deserialize(const std::string& to_deserialize) override
    {
        const char *string_c_str = to_deserialize.c_str();
        size_t string_length = to_deserialize.length();
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Pigeon2GyroScalarX, string_c_str, string_length, &GyroScalarX);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Pigeon2GyroScalarY, string_c_str, string_length, &GyroScalarY);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Pigeon2GyroScalarZ, string_c_str, string_length, &GyroScalarZ);
        return 0;
    }
};


/**
 * \brief Configs to enable/disable various features of the Pigeon2.
 * 
 * \details These configs allow the user to enable or disable various
 *          aspects of the Pigeon2.
 */
class Pigeon2FeaturesConfigs : public ParentConfiguration
{
public:
    /**
     * \brief Turns on or off the magnetometer fusing for 9-axis. FRC
     * users are not recommended to turn this on, as the magnetic
     * influence of the robot will likely negatively affect the
     * performance of the Pigeon2.
     * 
     *   Default Value: False
     */
    bool EnableCompass = false;
    /**
     * \brief Disables using the temperature compensation feature
     * 
     *   Default Value: False
     */
    bool DisableTemperatureCompensation = false;
    /**
     * \brief Disables using the no-motion calibration feature
     * 
     *   Default Value: False
     */
    bool DisableNoMotionCalibration = false;
    
    /**
     * \brief Modifies this configuration's EnableCompass parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newEnableCompass Parameter to modify
     * \returns Itself
     */
    Pigeon2FeaturesConfigs& WithEnableCompass(bool newEnableCompass)
    {
        EnableCompass = std::move(newEnableCompass);
        return *this;
    }
    /**
     * \brief Modifies this configuration's DisableTemperatureCompensation parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newDisableTemperatureCompensation Parameter to modify
     * \returns Itself
     */
    Pigeon2FeaturesConfigs& WithDisableTemperatureCompensation(bool newDisableTemperatureCompensation)
    {
        DisableTemperatureCompensation = std::move(newDisableTemperatureCompensation);
        return *this;
    }
    /**
     * \brief Modifies this configuration's DisableNoMotionCalibration parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newDisableNoMotionCalibration Parameter to modify
     * \returns Itself
     */
    Pigeon2FeaturesConfigs& WithDisableNoMotionCalibration(bool newDisableNoMotionCalibration)
    {
        DisableNoMotionCalibration = std::move(newDisableNoMotionCalibration);
        return *this;
    }

    

    std::string ToString() const override
    {
        std::stringstream ss;
        ss << "{" << std::endl;
        ss << "Config Group: Pigeon2Features" << std::endl;
        ss << "Name: \"EnableCompass\" Value: \"" << EnableCompass << "\"" << std::endl;
        ss << "Name: \"DisableTemperatureCompensation\" Value: \"" << DisableTemperatureCompensation << "\"" << std::endl;
        ss << "Name: \"DisableNoMotionCalibration\" Value: \"" << DisableNoMotionCalibration << "\"" << std::endl;
        ss << "}" << std::endl;
        return ss.str();
    }

    std::string Serialize() const override
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_bool(ctre::phoenix6::spns::SpnValue::Pigeon2UseCompass, EnableCompass, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_bool(ctre::phoenix6::spns::SpnValue::Pigeon2DisableTemperatureCompensation, DisableTemperatureCompensation, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_bool(ctre::phoenix6::spns::SpnValue::Pigeon2DisableNoMotionCalibration, DisableNoMotionCalibration, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return ss.str();
    }

    ctre::phoenix::StatusCode Deserialize(const std::string& to_deserialize) override
    {
        const char *string_c_str = to_deserialize.c_str();
        size_t string_length = to_deserialize.length();
        c_ctre_phoenix6_deserialize_bool(ctre::phoenix6::spns::SpnValue::Pigeon2UseCompass, string_c_str, string_length, &EnableCompass);
        c_ctre_phoenix6_deserialize_bool(ctre::phoenix6::spns::SpnValue::Pigeon2DisableTemperatureCompensation, string_c_str, string_length, &DisableTemperatureCompensation);
        c_ctre_phoenix6_deserialize_bool(ctre::phoenix6::spns::SpnValue::Pigeon2DisableNoMotionCalibration, string_c_str, string_length, &DisableNoMotionCalibration);
        return 0;
    }
};


/**
 * \brief Configs that directly affect motor-output.
 * 
 * \details Includes Motor Invert and various limit features.
 */
class MotorOutputConfigs : public ParentConfiguration
{
public:
    /**
     * \brief Invert state of the device
     * 
     */
    signals::InvertedValue Inverted = signals::InvertedValue::CounterClockwise_Positive;
    /**
     * \brief The state of the motor controller bridge when output is
     * neutral or disabled.
     * 
     */
    signals::NeutralModeValue NeutralMode = signals::NeutralModeValue::Coast;
    /**
     * \brief Configures the output deadband percentage.
     * 
     *   Minimum Value: 0.0
     *   Maximum Value: 0.25
     *   Default Value: 0
     *   Units: fractional
     */
    double DutyCycleNeutralDeadband = 0;
    /**
     * \brief Maximum (forward) output during duty cycle based control
     * modes.
     * 
     *   Minimum Value: -1.0
     *   Maximum Value: 1.0
     *   Default Value: 1
     *   Units: fractional
     */
    double PeakForwardDutyCycle = 1;
    /**
     * \brief Minimum (reverse) output during duty cycle based control
     * modes.
     * 
     *   Minimum Value: -1.0
     *   Maximum Value: 1.0
     *   Default Value: -1
     *   Units: fractional
     */
    double PeakReverseDutyCycle = -1;
    
    /**
     * \brief Modifies this configuration's Inverted parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newInverted Parameter to modify
     * \returns Itself
     */
    MotorOutputConfigs& WithInverted(signals::InvertedValue newInverted)
    {
        Inverted = std::move(newInverted);
        return *this;
    }
    /**
     * \brief Modifies this configuration's NeutralMode parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newNeutralMode Parameter to modify
     * \returns Itself
     */
    MotorOutputConfigs& WithNeutralMode(signals::NeutralModeValue newNeutralMode)
    {
        NeutralMode = std::move(newNeutralMode);
        return *this;
    }
    /**
     * \brief Modifies this configuration's DutyCycleNeutralDeadband parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newDutyCycleNeutralDeadband Parameter to modify
     * \returns Itself
     */
    MotorOutputConfigs& WithDutyCycleNeutralDeadband(double newDutyCycleNeutralDeadband)
    {
        DutyCycleNeutralDeadband = std::move(newDutyCycleNeutralDeadband);
        return *this;
    }
    /**
     * \brief Modifies this configuration's PeakForwardDutyCycle parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newPeakForwardDutyCycle Parameter to modify
     * \returns Itself
     */
    MotorOutputConfigs& WithPeakForwardDutyCycle(double newPeakForwardDutyCycle)
    {
        PeakForwardDutyCycle = std::move(newPeakForwardDutyCycle);
        return *this;
    }
    /**
     * \brief Modifies this configuration's PeakReverseDutyCycle parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newPeakReverseDutyCycle Parameter to modify
     * \returns Itself
     */
    MotorOutputConfigs& WithPeakReverseDutyCycle(double newPeakReverseDutyCycle)
    {
        PeakReverseDutyCycle = std::move(newPeakReverseDutyCycle);
        return *this;
    }

    

    std::string ToString() const override
    {
        std::stringstream ss;
        ss << "{" << std::endl;
        ss << "Config Group: MotorOutput" << std::endl;
        ss << "Name: \"Inverted\" Value: \"" << Inverted << "\"" << std::endl;
        ss << "Name: \"NeutralMode\" Value: \"" << NeutralMode << "\"" << std::endl;
        ss << "Name: \"DutyCycleNeutralDeadband\" Value: \"" << DutyCycleNeutralDeadband << "fractional\"" << std::endl;
        ss << "Name: \"PeakForwardDutyCycle\" Value: \"" << PeakForwardDutyCycle << "fractional\"" << std::endl;
        ss << "Name: \"PeakReverseDutyCycle\" Value: \"" << PeakReverseDutyCycle << "fractional\"" << std::endl;
        ss << "}" << std::endl;
        return ss.str();
    }

    std::string Serialize() const override
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_int(ctre::phoenix6::spns::SpnValue::Config_Inverted, Inverted.value, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_int(ctre::phoenix6::spns::SpnValue::Config_NeutralMode, NeutralMode.value, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Config_DutyCycleNeutralDB, DutyCycleNeutralDeadband, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Config_PeakForwardDC, PeakForwardDutyCycle, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Config_PeakReverseDC, PeakReverseDutyCycle, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return ss.str();
    }

    ctre::phoenix::StatusCode Deserialize(const std::string& to_deserialize) override
    {
        const char *string_c_str = to_deserialize.c_str();
        size_t string_length = to_deserialize.length();
        c_ctre_phoenix6_deserialize_int(ctre::phoenix6::spns::SpnValue::Config_Inverted, string_c_str, string_length, &Inverted.value);
        c_ctre_phoenix6_deserialize_int(ctre::phoenix6::spns::SpnValue::Config_NeutralMode, string_c_str, string_length, &NeutralMode.value);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Config_DutyCycleNeutralDB, string_c_str, string_length, &DutyCycleNeutralDeadband);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Config_PeakForwardDC, string_c_str, string_length, &PeakForwardDutyCycle);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Config_PeakReverseDC, string_c_str, string_length, &PeakReverseDutyCycle);
        return 0;
    }
};


/**
 * \brief Configs that directly affect current limiting features.
 * 
 * \details Contains the supply/stator current limit thresholds and
 *          whether to enable them or not.
 */
class CurrentLimitsConfigs : public ParentConfiguration
{
public:
    /**
     * \brief The amount of current allowed in the motor (motoring and
     * regen current).  This is only applicable for non-torque current
     * control modes.  Note this requires the corresponding enable to be
     * true.
     * 
     *   Minimum Value: 0.0
     *   Maximum Value: 800.0
     *   Default Value: 0
     *   Units: A
     */
    double StatorCurrentLimit = 0;
    /**
     * \brief Enable motor stator current limiting.
     * 
     *   Default Value: False
     */
    bool StatorCurrentLimitEnable = false;
    /**
     * \brief The amount of supply current allowed.  This is only
     * applicable for non-torque current control modes.  Note this
     * requires the corresponding enable to be true.  Use
     * SupplyCurrentThreshold and SupplyTimeThreshold to allow brief
     * periods of high-current before limiting occurs.
     * 
     *   Minimum Value: 0.0
     *   Maximum Value: 800.0
     *   Default Value: 0
     *   Units: A
     */
    double SupplyCurrentLimit = 0;
    /**
     * \brief Enable motor supply current limiting.
     * 
     *   Default Value: False
     */
    bool SupplyCurrentLimitEnable = false;
    /**
     * \brief Delay supply current limiting until current exceeds this
     * threshold for longer than SupplyTimeThreshold.  This allows current
     * draws above SupplyCurrentLimit for a fixed period of time.  This
     * has no effect if SupplyCurrentLimit is greater than this value.
     * 
     *   Minimum Value: 0.0
     *   Maximum Value: 511
     *   Default Value: 0
     *   Units: A
     */
    double SupplyCurrentThreshold = 0;
    /**
     * \brief Allows unlimited current for a period of time before current
     * limiting occurs.  Current threshold is the maximum of
     * SupplyCurrentThreshold and SupplyCurrentLimit.
     * 
     *   Minimum Value: 0.0
     *   Maximum Value: 1.275
     *   Default Value: 0
     *   Units: sec
     */
    double SupplyTimeThreshold = 0;
    
    /**
     * \brief Modifies this configuration's StatorCurrentLimit parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newStatorCurrentLimit Parameter to modify
     * \returns Itself
     */
    CurrentLimitsConfigs& WithStatorCurrentLimit(double newStatorCurrentLimit)
    {
        StatorCurrentLimit = std::move(newStatorCurrentLimit);
        return *this;
    }
    /**
     * \brief Modifies this configuration's StatorCurrentLimitEnable parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newStatorCurrentLimitEnable Parameter to modify
     * \returns Itself
     */
    CurrentLimitsConfigs& WithStatorCurrentLimitEnable(bool newStatorCurrentLimitEnable)
    {
        StatorCurrentLimitEnable = std::move(newStatorCurrentLimitEnable);
        return *this;
    }
    /**
     * \brief Modifies this configuration's SupplyCurrentLimit parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newSupplyCurrentLimit Parameter to modify
     * \returns Itself
     */
    CurrentLimitsConfigs& WithSupplyCurrentLimit(double newSupplyCurrentLimit)
    {
        SupplyCurrentLimit = std::move(newSupplyCurrentLimit);
        return *this;
    }
    /**
     * \brief Modifies this configuration's SupplyCurrentLimitEnable parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newSupplyCurrentLimitEnable Parameter to modify
     * \returns Itself
     */
    CurrentLimitsConfigs& WithSupplyCurrentLimitEnable(bool newSupplyCurrentLimitEnable)
    {
        SupplyCurrentLimitEnable = std::move(newSupplyCurrentLimitEnable);
        return *this;
    }
    /**
     * \brief Modifies this configuration's SupplyCurrentThreshold parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newSupplyCurrentThreshold Parameter to modify
     * \returns Itself
     */
    CurrentLimitsConfigs& WithSupplyCurrentThreshold(double newSupplyCurrentThreshold)
    {
        SupplyCurrentThreshold = std::move(newSupplyCurrentThreshold);
        return *this;
    }
    /**
     * \brief Modifies this configuration's SupplyTimeThreshold parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newSupplyTimeThreshold Parameter to modify
     * \returns Itself
     */
    CurrentLimitsConfigs& WithSupplyTimeThreshold(double newSupplyTimeThreshold)
    {
        SupplyTimeThreshold = std::move(newSupplyTimeThreshold);
        return *this;
    }

    

    std::string ToString() const override
    {
        std::stringstream ss;
        ss << "{" << std::endl;
        ss << "Config Group: CurrentLimits" << std::endl;
        ss << "Name: \"StatorCurrentLimit\" Value: \"" << StatorCurrentLimit << "A\"" << std::endl;
        ss << "Name: \"StatorCurrentLimitEnable\" Value: \"" << StatorCurrentLimitEnable << "\"" << std::endl;
        ss << "Name: \"SupplyCurrentLimit\" Value: \"" << SupplyCurrentLimit << "A\"" << std::endl;
        ss << "Name: \"SupplyCurrentLimitEnable\" Value: \"" << SupplyCurrentLimitEnable << "\"" << std::endl;
        ss << "Name: \"SupplyCurrentThreshold\" Value: \"" << SupplyCurrentThreshold << "A\"" << std::endl;
        ss << "Name: \"SupplyTimeThreshold\" Value: \"" << SupplyTimeThreshold << "sec\"" << std::endl;
        ss << "}" << std::endl;
        return ss.str();
    }

    std::string Serialize() const override
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Config_StatorCurrentLimit, StatorCurrentLimit, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_bool(ctre::phoenix6::spns::SpnValue::Config_StatorCurrLimitEn, StatorCurrentLimitEnable, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Config_SupplyCurrentLimit, SupplyCurrentLimit, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_bool(ctre::phoenix6::spns::SpnValue::Config_SupplyCurrLimitEn, SupplyCurrentLimitEnable, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Config_SupplyCurrThres, SupplyCurrentThreshold, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Config_SupplyTimeThres, SupplyTimeThreshold, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return ss.str();
    }

    ctre::phoenix::StatusCode Deserialize(const std::string& to_deserialize) override
    {
        const char *string_c_str = to_deserialize.c_str();
        size_t string_length = to_deserialize.length();
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Config_StatorCurrentLimit, string_c_str, string_length, &StatorCurrentLimit);
        c_ctre_phoenix6_deserialize_bool(ctre::phoenix6::spns::SpnValue::Config_StatorCurrLimitEn, string_c_str, string_length, &StatorCurrentLimitEnable);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Config_SupplyCurrentLimit, string_c_str, string_length, &SupplyCurrentLimit);
        c_ctre_phoenix6_deserialize_bool(ctre::phoenix6::spns::SpnValue::Config_SupplyCurrLimitEn, string_c_str, string_length, &SupplyCurrentLimitEnable);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Config_SupplyCurrThres, string_c_str, string_length, &SupplyCurrentThreshold);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Config_SupplyTimeThres, string_c_str, string_length, &SupplyTimeThreshold);
        return 0;
    }
};


/**
 * \brief Voltage-specific configs
 * 
 * \details Voltage-specific configs
 */
class VoltageConfigs : public ParentConfiguration
{
public:
    /**
     * \brief The time constant (in seconds) of the low-pass filter for
     * the supply voltage.
     * 
     * \details This impacts the filtering for the reported supply
     * voltage, and any control strategies that use the supply voltage
     * (such as voltage control on a motor controller).
     * 
     *   Minimum Value: 0.0
     *   Maximum Value: 0.1
     *   Default Value: 0
     *   Units: sec
     */
    double SupplyVoltageTimeConstant = 0;
    /**
     * \brief Maximum (forward) output during voltage based control modes.
     * 
     *   Minimum Value: -16
     *   Maximum Value: 16
     *   Default Value: 16
     *   Units: V
     */
    double PeakForwardVoltage = 16;
    /**
     * \brief Minimum (reverse) output during voltage based control modes.
     * 
     *   Minimum Value: -16
     *   Maximum Value: 16
     *   Default Value: -16
     *   Units: V
     */
    double PeakReverseVoltage = -16;
    
    /**
     * \brief Modifies this configuration's SupplyVoltageTimeConstant parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newSupplyVoltageTimeConstant Parameter to modify
     * \returns Itself
     */
    VoltageConfigs& WithSupplyVoltageTimeConstant(double newSupplyVoltageTimeConstant)
    {
        SupplyVoltageTimeConstant = std::move(newSupplyVoltageTimeConstant);
        return *this;
    }
    /**
     * \brief Modifies this configuration's PeakForwardVoltage parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newPeakForwardVoltage Parameter to modify
     * \returns Itself
     */
    VoltageConfigs& WithPeakForwardVoltage(double newPeakForwardVoltage)
    {
        PeakForwardVoltage = std::move(newPeakForwardVoltage);
        return *this;
    }
    /**
     * \brief Modifies this configuration's PeakReverseVoltage parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newPeakReverseVoltage Parameter to modify
     * \returns Itself
     */
    VoltageConfigs& WithPeakReverseVoltage(double newPeakReverseVoltage)
    {
        PeakReverseVoltage = std::move(newPeakReverseVoltage);
        return *this;
    }

    

    std::string ToString() const override
    {
        std::stringstream ss;
        ss << "{" << std::endl;
        ss << "Config Group: Voltage" << std::endl;
        ss << "Name: \"SupplyVoltageTimeConstant\" Value: \"" << SupplyVoltageTimeConstant << "sec\"" << std::endl;
        ss << "Name: \"PeakForwardVoltage\" Value: \"" << PeakForwardVoltage << "V\"" << std::endl;
        ss << "Name: \"PeakReverseVoltage\" Value: \"" << PeakReverseVoltage << "V\"" << std::endl;
        ss << "}" << std::endl;
        return ss.str();
    }

    std::string Serialize() const override
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Config_SupplyVLowpassTau, SupplyVoltageTimeConstant, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Config_PeakForwardV, PeakForwardVoltage, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Config_PeakReverseV, PeakReverseVoltage, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return ss.str();
    }

    ctre::phoenix::StatusCode Deserialize(const std::string& to_deserialize) override
    {
        const char *string_c_str = to_deserialize.c_str();
        size_t string_length = to_deserialize.length();
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Config_SupplyVLowpassTau, string_c_str, string_length, &SupplyVoltageTimeConstant);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Config_PeakForwardV, string_c_str, string_length, &PeakForwardVoltage);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Config_PeakReverseV, string_c_str, string_length, &PeakReverseVoltage);
        return 0;
    }
};


/**
 * \brief Configs to control the maximum and minimum applied torque
 *        when using Torque Current control types.
 * 
 * \details Similar to peak output, but for the TorqueCurrentFOC
 *          control type requests.
 */
class TorqueCurrentConfigs : public ParentConfiguration
{
public:
    /**
     * \brief Maximum (forward) output during torque current based control
     * modes.
     * 
     *   Minimum Value: -800
     *   Maximum Value: 800
     *   Default Value: 800
     *   Units: A
     */
    double PeakForwardTorqueCurrent = 800;
    /**
     * \brief Minimum (reverse) output during torque current based control
     * modes.
     * 
     *   Minimum Value: -800
     *   Maximum Value: 800
     *   Default Value: -800
     *   Units: A
     */
    double PeakReverseTorqueCurrent = -800;
    /**
     * \brief Configures the output deadband during torque current based
     * control modes.
     * 
     *   Minimum Value: 0
     *   Maximum Value: 25
     *   Default Value: 0.0
     *   Units: A
     */
    double TorqueNeutralDeadband = 0.0;
    
    /**
     * \brief Modifies this configuration's PeakForwardTorqueCurrent parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newPeakForwardTorqueCurrent Parameter to modify
     * \returns Itself
     */
    TorqueCurrentConfigs& WithPeakForwardTorqueCurrent(double newPeakForwardTorqueCurrent)
    {
        PeakForwardTorqueCurrent = std::move(newPeakForwardTorqueCurrent);
        return *this;
    }
    /**
     * \brief Modifies this configuration's PeakReverseTorqueCurrent parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newPeakReverseTorqueCurrent Parameter to modify
     * \returns Itself
     */
    TorqueCurrentConfigs& WithPeakReverseTorqueCurrent(double newPeakReverseTorqueCurrent)
    {
        PeakReverseTorqueCurrent = std::move(newPeakReverseTorqueCurrent);
        return *this;
    }
    /**
     * \brief Modifies this configuration's TorqueNeutralDeadband parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newTorqueNeutralDeadband Parameter to modify
     * \returns Itself
     */
    TorqueCurrentConfigs& WithTorqueNeutralDeadband(double newTorqueNeutralDeadband)
    {
        TorqueNeutralDeadband = std::move(newTorqueNeutralDeadband);
        return *this;
    }

    

    std::string ToString() const override
    {
        std::stringstream ss;
        ss << "{" << std::endl;
        ss << "Config Group: TorqueCurrent" << std::endl;
        ss << "Name: \"PeakForwardTorqueCurrent\" Value: \"" << PeakForwardTorqueCurrent << "A\"" << std::endl;
        ss << "Name: \"PeakReverseTorqueCurrent\" Value: \"" << PeakReverseTorqueCurrent << "A\"" << std::endl;
        ss << "Name: \"TorqueNeutralDeadband\" Value: \"" << TorqueNeutralDeadband << "A\"" << std::endl;
        ss << "}" << std::endl;
        return ss.str();
    }

    std::string Serialize() const override
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Config_PeakForTorqCurr, PeakForwardTorqueCurrent, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Config_PeakRevTorqCurr, PeakReverseTorqueCurrent, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Config_TorqueNeutralDB, TorqueNeutralDeadband, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return ss.str();
    }

    ctre::phoenix::StatusCode Deserialize(const std::string& to_deserialize) override
    {
        const char *string_c_str = to_deserialize.c_str();
        size_t string_length = to_deserialize.length();
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Config_PeakForTorqCurr, string_c_str, string_length, &PeakForwardTorqueCurrent);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Config_PeakRevTorqCurr, string_c_str, string_length, &PeakReverseTorqueCurrent);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Config_TorqueNeutralDB, string_c_str, string_length, &TorqueNeutralDeadband);
        return 0;
    }
};


/**
 * \brief Configs that affect the feedback of this motor controller.
 * 
 * \details Includes feedback sensor source, any offsets for the
 *          feedback sensor, and various ratios to describe the
 *          relationship between the sensor and the mechanism for
 *          closed looping.
 */
class FeedbackConfigs : public ParentConfiguration
{
public:
    /**
     * \brief This offset is applied to the absolute integrated rotor
     * sensor.  This can be used to zero the rotor in applications that
     * are within one rotor rotation.
     * 
     *   Minimum Value: -1
     *   Maximum Value: 1
     *   Default Value: 0.0
     *   Units: rotations
     */
    double FeedbackRotorOffset = 0.0;
    /**
     * \brief This is the ratio of sensor rotations to the mechanism's
     * output.  This is equivalent to the mechanism's gear ratio if the
     * sensor is located on the input of a gearbox.  If sensor is on the
     * output of a gearbox, then this is typically set to 1.  Note if this
     * is set to zero, device will reset back to one.
     * 
     *   Minimum Value: -1000
     *   Maximum Value: 1000
     *   Default Value: 1.0
     *   Units: scalar
     */
    double SensorToMechanismRatio = 1.0;
    /**
     * \brief Talon FX is capable of fusing a remote CANcoder with its
     * rotor sensor to produce a high-bandwidth sensor source.  This
     * feature requires specifying the ratio between the remote sensor and
     * the motor rotor.  Note if this is set to zero, device will reset
     * back to one.
     * 
     *   Minimum Value: -1000
     *   Maximum Value: 1000
     *   Default Value: 1.0
     *   Units: scalar
     */
    double RotorToSensorRatio = 1.0;
    /**
     * \brief Choose what sensor source is reported via API and used by
     * closed-loop and limit features.  The default is RotorSensor, which
     * uses the internal rotor sensor in the Talon FX.  Choose
     * RemoteCANcoder to use another CANcoder on the same CAN bus (this
     * also requires setting FeedbackRemoteSensorID).  Talon FX will
     * update its position and velocity whenever CANcoder publishes its
     * information on CAN bus.  Choose FusedCANcoder (requires Phoenix
     * Pro) and Talon FX will fuse another CANcoder's information with the
     * internal rotor, which provides the best possible position and
     * velocity for accuracy and bandwidth (note this requires setting
     * FeedbackRemoteSensorID).  FusedCANcoder was developed for
     * applications such as swerve-azimuth.  Choose SyncCANcoder (requires
     * Phoenix Pro) and Talon FX will synchronize its internal rotor
     * position against another CANcoder, then continue to use the rotor
     * sensor for closed loop control (note this requires setting
     * FeedbackRemoteSensorID).  The TalonFX will report if its internal
     * position differs significantly from the reported CANcoder position.
     *  SyncCANcoder was developed for mechanisms where there is a risk of
     * the CANcoder failing in such a way that it reports a position that
     * does not match the mechanism, such as the sensor mounting assembly
     * breaking off.  Choose RemotePigeon2_Yaw, RemotePigeon2_Pitch, and
     * RemotePigeon2_Roll to use another Pigeon2 on the same CAN bus (this
     * also requires setting FeedbackRemoteSensorID).  Talon FX will
     * update its position to match the selected value whenever Pigeon2
     * publishes its information on CAN bus. Note that the Talon FX
     * position will be in rotations and not degrees.
     * 
     * \details Note: When the Talon Source is changed to FusedCANcoder,
     * the Talon needs a period of time to fuse before sensor-based
     * (soft-limit, closed loop, etc.) features are used. This period of
     * time is determined by the update frequency of the CANcoder's
     * Position signal.
     * 
     */
    signals::FeedbackSensorSourceValue FeedbackSensorSource = signals::FeedbackSensorSourceValue::RotorSensor;
    /**
     * \brief Device ID of which remote device to use.  This is not used
     * if the Sensor Source is the internal rotor sensor.
     * 
     *   Minimum Value: 0
     *   Maximum Value: 62
     *   Default Value: 0
     *   Units: 
     */
    int FeedbackRemoteSensorID = 0;
    
    /**
     * \brief Modifies this configuration's FeedbackRotorOffset parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newFeedbackRotorOffset Parameter to modify
     * \returns Itself
     */
    FeedbackConfigs& WithFeedbackRotorOffset(double newFeedbackRotorOffset)
    {
        FeedbackRotorOffset = std::move(newFeedbackRotorOffset);
        return *this;
    }
    /**
     * \brief Modifies this configuration's SensorToMechanismRatio parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newSensorToMechanismRatio Parameter to modify
     * \returns Itself
     */
    FeedbackConfigs& WithSensorToMechanismRatio(double newSensorToMechanismRatio)
    {
        SensorToMechanismRatio = std::move(newSensorToMechanismRatio);
        return *this;
    }
    /**
     * \brief Modifies this configuration's RotorToSensorRatio parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newRotorToSensorRatio Parameter to modify
     * \returns Itself
     */
    FeedbackConfigs& WithRotorToSensorRatio(double newRotorToSensorRatio)
    {
        RotorToSensorRatio = std::move(newRotorToSensorRatio);
        return *this;
    }
    /**
     * \brief Modifies this configuration's FeedbackSensorSource parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newFeedbackSensorSource Parameter to modify
     * \returns Itself
     */
    FeedbackConfigs& WithFeedbackSensorSource(signals::FeedbackSensorSourceValue newFeedbackSensorSource)
    {
        FeedbackSensorSource = std::move(newFeedbackSensorSource);
        return *this;
    }
    /**
     * \brief Modifies this configuration's FeedbackRemoteSensorID parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newFeedbackRemoteSensorID Parameter to modify
     * \returns Itself
     */
    FeedbackConfigs& WithFeedbackRemoteSensorID(int newFeedbackRemoteSensorID)
    {
        FeedbackRemoteSensorID = std::move(newFeedbackRemoteSensorID);
        return *this;
    }
    
    /**
     * \brief Helper method to configure this feedback group to use Fused
     *        CANcoder by passing in the CANcoder object
     * 
     * \param device    CANcoder reference to use for FusedCANcoder
     */
    FeedbackConfigs& WithFusedCANcoder(const hardware::core::CoreCANcoder& device);
    
    /**
     * \brief Helper method to configure this feedback group to use Remote
     *        CANcoder by passing in the CANcoder object
     * 
     * \param device    CANcoder reference to use for FusedCANcoder
     */
    FeedbackConfigs& WithRemoteCANcoder(const hardware::core::CoreCANcoder& device);
    
    

    std::string ToString() const override
    {
        std::stringstream ss;
        ss << "{" << std::endl;
        ss << "Config Group: Feedback" << std::endl;
        ss << "Name: \"FeedbackRotorOffset\" Value: \"" << FeedbackRotorOffset << "rotations\"" << std::endl;
        ss << "Name: \"SensorToMechanismRatio\" Value: \"" << SensorToMechanismRatio << "scalar\"" << std::endl;
        ss << "Name: \"RotorToSensorRatio\" Value: \"" << RotorToSensorRatio << "scalar\"" << std::endl;
        ss << "Name: \"FeedbackSensorSource\" Value: \"" << FeedbackSensorSource << "\"" << std::endl;
        ss << "Name: \"FeedbackRemoteSensorID\" Value: \"" << FeedbackRemoteSensorID << "\"" << std::endl;
        ss << "}" << std::endl;
        return ss.str();
    }

    std::string Serialize() const override
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Config_FeedbackRotorOffset, FeedbackRotorOffset, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Config_SensorToMechanismRatio, SensorToMechanismRatio, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Config_RotorToSensorRatio, RotorToSensorRatio, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_int(ctre::phoenix6::spns::SpnValue::Config_FeedbackSensorSource, FeedbackSensorSource.value, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_int(ctre::phoenix6::spns::SpnValue::Config_FeedbackRemoteSensorID, FeedbackRemoteSensorID, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return ss.str();
    }

    ctre::phoenix::StatusCode Deserialize(const std::string& to_deserialize) override
    {
        const char *string_c_str = to_deserialize.c_str();
        size_t string_length = to_deserialize.length();
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Config_FeedbackRotorOffset, string_c_str, string_length, &FeedbackRotorOffset);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Config_SensorToMechanismRatio, string_c_str, string_length, &SensorToMechanismRatio);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Config_RotorToSensorRatio, string_c_str, string_length, &RotorToSensorRatio);
        c_ctre_phoenix6_deserialize_int(ctre::phoenix6::spns::SpnValue::Config_FeedbackSensorSource, string_c_str, string_length, &FeedbackSensorSource.value);
        c_ctre_phoenix6_deserialize_int(ctre::phoenix6::spns::SpnValue::Config_FeedbackRemoteSensorID, string_c_str, string_length, &FeedbackRemoteSensorID);
        return 0;
    }
};


/**
 * \brief Configs related to sensors used for differential control of
 *        a mechanism.
 * 
 * \details Includes the differential sensor sources and IDs.
 */
class DifferentialSensorsConfigs : public ParentConfiguration
{
public:
    /**
     * \brief Choose what sensor source is used for differential control
     * of a mechanism.  The default is Disabled.  All other options
     * require setting the DifferentialTalonFXSensorID, as the average of
     * this Talon FX's sensor and the remote TalonFX's sensor is used for
     * the differential controller's primary targets.  Choose
     * RemoteTalonFX_Diff to use another TalonFX on the same CAN bus. 
     * Talon FX will update its differential position and velocity
     * whenever the remote TalonFX publishes its information on CAN bus. 
     * The differential controller will use the difference between this
     * TalonFX's sensor and the remote Talon FX's sensor for the
     * differential component of the output.  Choose RemotePigeon2_Yaw,
     * RemotePigeon2_Pitch, and RemotePigeon2_Roll to use another Pigeon2
     * on the same CAN bus (this also requires setting
     * DifferentialRemoteSensorID).  Talon FX will update its differential
     * position to match the selected value whenever Pigeon2 publishes its
     * information on CAN bus. Note that the Talon FX differential
     * position will be in rotations and not degrees. Choose
     * RemoteCANcoder to use another CANcoder on the same CAN bus (this
     * also requires setting DifferentialRemoteSensorID).  Talon FX will
     * update its differential position and velocity to match the CANcoder
     * whenever CANcoder publishes its information on CAN bus.
     * 
     */
    signals::DifferentialSensorSourceValue DifferentialSensorSource = signals::DifferentialSensorSourceValue::Disabled;
    /**
     * \brief Device ID of which remote Talon FX to use.  This is used
     * when the Differential Sensor Source is not disabled.
     * 
     *   Minimum Value: 0
     *   Maximum Value: 62
     *   Default Value: 0
     *   Units: 
     */
    int DifferentialTalonFXSensorID = 0;
    /**
     * \brief Device ID of which remote sensor to use on the differential
     * axis.  This is used when the Differential Sensor Source is not
     * RemoteTalonFX_Diff.
     * 
     *   Minimum Value: 0
     *   Maximum Value: 62
     *   Default Value: 0
     *   Units: 
     */
    int DifferentialRemoteSensorID = 0;
    
    /**
     * \brief Modifies this configuration's DifferentialSensorSource parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newDifferentialSensorSource Parameter to modify
     * \returns Itself
     */
    DifferentialSensorsConfigs& WithDifferentialSensorSource(signals::DifferentialSensorSourceValue newDifferentialSensorSource)
    {
        DifferentialSensorSource = std::move(newDifferentialSensorSource);
        return *this;
    }
    /**
     * \brief Modifies this configuration's DifferentialTalonFXSensorID parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newDifferentialTalonFXSensorID Parameter to modify
     * \returns Itself
     */
    DifferentialSensorsConfigs& WithDifferentialTalonFXSensorID(int newDifferentialTalonFXSensorID)
    {
        DifferentialTalonFXSensorID = std::move(newDifferentialTalonFXSensorID);
        return *this;
    }
    /**
     * \brief Modifies this configuration's DifferentialRemoteSensorID parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newDifferentialRemoteSensorID Parameter to modify
     * \returns Itself
     */
    DifferentialSensorsConfigs& WithDifferentialRemoteSensorID(int newDifferentialRemoteSensorID)
    {
        DifferentialRemoteSensorID = std::move(newDifferentialRemoteSensorID);
        return *this;
    }

    

    std::string ToString() const override
    {
        std::stringstream ss;
        ss << "{" << std::endl;
        ss << "Config Group: DifferentialSensors" << std::endl;
        ss << "Name: \"DifferentialSensorSource\" Value: \"" << DifferentialSensorSource << "\"" << std::endl;
        ss << "Name: \"DifferentialTalonFXSensorID\" Value: \"" << DifferentialTalonFXSensorID << "\"" << std::endl;
        ss << "Name: \"DifferentialRemoteSensorID\" Value: \"" << DifferentialRemoteSensorID << "\"" << std::endl;
        ss << "}" << std::endl;
        return ss.str();
    }

    std::string Serialize() const override
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_int(ctre::phoenix6::spns::SpnValue::Config_DifferentialSensorSource, DifferentialSensorSource.value, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_int(ctre::phoenix6::spns::SpnValue::Config_DifferentialTalonFXSensorID, DifferentialTalonFXSensorID, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_int(ctre::phoenix6::spns::SpnValue::Config_DifferentialRemoteSensorID, DifferentialRemoteSensorID, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return ss.str();
    }

    ctre::phoenix::StatusCode Deserialize(const std::string& to_deserialize) override
    {
        const char *string_c_str = to_deserialize.c_str();
        size_t string_length = to_deserialize.length();
        c_ctre_phoenix6_deserialize_int(ctre::phoenix6::spns::SpnValue::Config_DifferentialSensorSource, string_c_str, string_length, &DifferentialSensorSource.value);
        c_ctre_phoenix6_deserialize_int(ctre::phoenix6::spns::SpnValue::Config_DifferentialTalonFXSensorID, string_c_str, string_length, &DifferentialTalonFXSensorID);
        c_ctre_phoenix6_deserialize_int(ctre::phoenix6::spns::SpnValue::Config_DifferentialRemoteSensorID, string_c_str, string_length, &DifferentialRemoteSensorID);
        return 0;
    }
};


/**
 * \brief Configs related to constants used for differential control
 *        of a mechanism.
 * 
 * \details Includes the differential peak outputs.
 */
class DifferentialConstantsConfigs : public ParentConfiguration
{
public:
    /**
     * \brief Maximum differential output during duty cycle based
     * differential control modes.
     * 
     *   Minimum Value: 0.0
     *   Maximum Value: 2.0
     *   Default Value: 2
     *   Units: fractional
     */
    double PeakDifferentialDutyCycle = 2;
    /**
     * \brief Maximum differential output during voltage based
     * differential control modes.
     * 
     *   Minimum Value: 0.0
     *   Maximum Value: 32
     *   Default Value: 32
     *   Units: V
     */
    double PeakDifferentialVoltage = 32;
    /**
     * \brief Maximum differential output during torque current based
     * differential control modes.
     * 
     *   Minimum Value: 0.0
     *   Maximum Value: 1600
     *   Default Value: 1600
     *   Units: A
     */
    double PeakDifferentialTorqueCurrent = 1600;
    
    /**
     * \brief Modifies this configuration's PeakDifferentialDutyCycle parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newPeakDifferentialDutyCycle Parameter to modify
     * \returns Itself
     */
    DifferentialConstantsConfigs& WithPeakDifferentialDutyCycle(double newPeakDifferentialDutyCycle)
    {
        PeakDifferentialDutyCycle = std::move(newPeakDifferentialDutyCycle);
        return *this;
    }
    /**
     * \brief Modifies this configuration's PeakDifferentialVoltage parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newPeakDifferentialVoltage Parameter to modify
     * \returns Itself
     */
    DifferentialConstantsConfigs& WithPeakDifferentialVoltage(double newPeakDifferentialVoltage)
    {
        PeakDifferentialVoltage = std::move(newPeakDifferentialVoltage);
        return *this;
    }
    /**
     * \brief Modifies this configuration's PeakDifferentialTorqueCurrent parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newPeakDifferentialTorqueCurrent Parameter to modify
     * \returns Itself
     */
    DifferentialConstantsConfigs& WithPeakDifferentialTorqueCurrent(double newPeakDifferentialTorqueCurrent)
    {
        PeakDifferentialTorqueCurrent = std::move(newPeakDifferentialTorqueCurrent);
        return *this;
    }

    

    std::string ToString() const override
    {
        std::stringstream ss;
        ss << "{" << std::endl;
        ss << "Config Group: DifferentialConstants" << std::endl;
        ss << "Name: \"PeakDifferentialDutyCycle\" Value: \"" << PeakDifferentialDutyCycle << "fractional\"" << std::endl;
        ss << "Name: \"PeakDifferentialVoltage\" Value: \"" << PeakDifferentialVoltage << "V\"" << std::endl;
        ss << "Name: \"PeakDifferentialTorqueCurrent\" Value: \"" << PeakDifferentialTorqueCurrent << "A\"" << std::endl;
        ss << "}" << std::endl;
        return ss.str();
    }

    std::string Serialize() const override
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Config_PeakDiffDC, PeakDifferentialDutyCycle, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Config_PeakDiffV, PeakDifferentialVoltage, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Config_PeakDiffTorqCurr, PeakDifferentialTorqueCurrent, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return ss.str();
    }

    ctre::phoenix::StatusCode Deserialize(const std::string& to_deserialize) override
    {
        const char *string_c_str = to_deserialize.c_str();
        size_t string_length = to_deserialize.length();
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Config_PeakDiffDC, string_c_str, string_length, &PeakDifferentialDutyCycle);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Config_PeakDiffV, string_c_str, string_length, &PeakDifferentialVoltage);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Config_PeakDiffTorqCurr, string_c_str, string_length, &PeakDifferentialTorqueCurrent);
        return 0;
    }
};


/**
 * \brief Configs that affect the open-loop control of this motor
 *        controller.
 * 
 * \details Open-loop ramp rates for the various control types.
 */
class OpenLoopRampsConfigs : public ParentConfiguration
{
public:
    /**
     * \brief If non-zero, this determines how much time to ramp from 0%
     * output to 100% during open-loop modes.
     * 
     *   Minimum Value: 0
     *   Maximum Value: 1
     *   Default Value: 0
     *   Units: sec
     */
    double DutyCycleOpenLoopRampPeriod = 0;
    /**
     * \brief If non-zero, this determines how much time to ramp from 0V
     * output to 12V during open-loop modes.
     * 
     *   Minimum Value: 0
     *   Maximum Value: 1
     *   Default Value: 0
     *   Units: sec
     */
    double VoltageOpenLoopRampPeriod = 0;
    /**
     * \brief If non-zero, this determines how much time to ramp from 0A
     * output to 300A during open-loop modes.
     * 
     *   Minimum Value: 0
     *   Maximum Value: 10
     *   Default Value: 0
     *   Units: sec
     */
    double TorqueOpenLoopRampPeriod = 0;
    
    /**
     * \brief Modifies this configuration's DutyCycleOpenLoopRampPeriod parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newDutyCycleOpenLoopRampPeriod Parameter to modify
     * \returns Itself
     */
    OpenLoopRampsConfigs& WithDutyCycleOpenLoopRampPeriod(double newDutyCycleOpenLoopRampPeriod)
    {
        DutyCycleOpenLoopRampPeriod = std::move(newDutyCycleOpenLoopRampPeriod);
        return *this;
    }
    /**
     * \brief Modifies this configuration's VoltageOpenLoopRampPeriod parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newVoltageOpenLoopRampPeriod Parameter to modify
     * \returns Itself
     */
    OpenLoopRampsConfigs& WithVoltageOpenLoopRampPeriod(double newVoltageOpenLoopRampPeriod)
    {
        VoltageOpenLoopRampPeriod = std::move(newVoltageOpenLoopRampPeriod);
        return *this;
    }
    /**
     * \brief Modifies this configuration's TorqueOpenLoopRampPeriod parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newTorqueOpenLoopRampPeriod Parameter to modify
     * \returns Itself
     */
    OpenLoopRampsConfigs& WithTorqueOpenLoopRampPeriod(double newTorqueOpenLoopRampPeriod)
    {
        TorqueOpenLoopRampPeriod = std::move(newTorqueOpenLoopRampPeriod);
        return *this;
    }

    

    std::string ToString() const override
    {
        std::stringstream ss;
        ss << "{" << std::endl;
        ss << "Config Group: OpenLoopRamps" << std::endl;
        ss << "Name: \"DutyCycleOpenLoopRampPeriod\" Value: \"" << DutyCycleOpenLoopRampPeriod << "sec\"" << std::endl;
        ss << "Name: \"VoltageOpenLoopRampPeriod\" Value: \"" << VoltageOpenLoopRampPeriod << "sec\"" << std::endl;
        ss << "Name: \"TorqueOpenLoopRampPeriod\" Value: \"" << TorqueOpenLoopRampPeriod << "sec\"" << std::endl;
        ss << "}" << std::endl;
        return ss.str();
    }

    std::string Serialize() const override
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Config_DutyCycleOpenLoopRampPeriod, DutyCycleOpenLoopRampPeriod, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Config_VoltageOpenLoopRampPeriod, VoltageOpenLoopRampPeriod, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Config_TorqueOpenLoopRampPeriod, TorqueOpenLoopRampPeriod, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return ss.str();
    }

    ctre::phoenix::StatusCode Deserialize(const std::string& to_deserialize) override
    {
        const char *string_c_str = to_deserialize.c_str();
        size_t string_length = to_deserialize.length();
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Config_DutyCycleOpenLoopRampPeriod, string_c_str, string_length, &DutyCycleOpenLoopRampPeriod);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Config_VoltageOpenLoopRampPeriod, string_c_str, string_length, &VoltageOpenLoopRampPeriod);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Config_TorqueOpenLoopRampPeriod, string_c_str, string_length, &TorqueOpenLoopRampPeriod);
        return 0;
    }
};


/**
 * \brief Configs that affect the closed-loop control of this motor
 *        controller.
 * 
 * \details Closed-loop ramp rates for the various control types.
 */
class ClosedLoopRampsConfigs : public ParentConfiguration
{
public:
    /**
     * \brief If non-zero, this determines how much time to ramp from 0%
     * output to 100% during closed-loop modes.
     * 
     *   Minimum Value: 0
     *   Maximum Value: 1
     *   Default Value: 0
     *   Units: sec
     */
    double DutyCycleClosedLoopRampPeriod = 0;
    /**
     * \brief If non-zero, this determines how much time to ramp from 0V
     * output to 12V during closed-loop modes.
     * 
     *   Minimum Value: 0
     *   Maximum Value: 1
     *   Default Value: 0
     *   Units: sec
     */
    double VoltageClosedLoopRampPeriod = 0;
    /**
     * \brief If non-zero, this determines how much time to ramp from 0A
     * output to 300A during closed-loop modes.
     * 
     *   Minimum Value: 0
     *   Maximum Value: 10
     *   Default Value: 0
     *   Units: sec
     */
    double TorqueClosedLoopRampPeriod = 0;
    
    /**
     * \brief Modifies this configuration's DutyCycleClosedLoopRampPeriod parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newDutyCycleClosedLoopRampPeriod Parameter to modify
     * \returns Itself
     */
    ClosedLoopRampsConfigs& WithDutyCycleClosedLoopRampPeriod(double newDutyCycleClosedLoopRampPeriod)
    {
        DutyCycleClosedLoopRampPeriod = std::move(newDutyCycleClosedLoopRampPeriod);
        return *this;
    }
    /**
     * \brief Modifies this configuration's VoltageClosedLoopRampPeriod parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newVoltageClosedLoopRampPeriod Parameter to modify
     * \returns Itself
     */
    ClosedLoopRampsConfigs& WithVoltageClosedLoopRampPeriod(double newVoltageClosedLoopRampPeriod)
    {
        VoltageClosedLoopRampPeriod = std::move(newVoltageClosedLoopRampPeriod);
        return *this;
    }
    /**
     * \brief Modifies this configuration's TorqueClosedLoopRampPeriod parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newTorqueClosedLoopRampPeriod Parameter to modify
     * \returns Itself
     */
    ClosedLoopRampsConfigs& WithTorqueClosedLoopRampPeriod(double newTorqueClosedLoopRampPeriod)
    {
        TorqueClosedLoopRampPeriod = std::move(newTorqueClosedLoopRampPeriod);
        return *this;
    }

    

    std::string ToString() const override
    {
        std::stringstream ss;
        ss << "{" << std::endl;
        ss << "Config Group: ClosedLoopRamps" << std::endl;
        ss << "Name: \"DutyCycleClosedLoopRampPeriod\" Value: \"" << DutyCycleClosedLoopRampPeriod << "sec\"" << std::endl;
        ss << "Name: \"VoltageClosedLoopRampPeriod\" Value: \"" << VoltageClosedLoopRampPeriod << "sec\"" << std::endl;
        ss << "Name: \"TorqueClosedLoopRampPeriod\" Value: \"" << TorqueClosedLoopRampPeriod << "sec\"" << std::endl;
        ss << "}" << std::endl;
        return ss.str();
    }

    std::string Serialize() const override
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Config_DutyCycleClosedLoopRampPeriod, DutyCycleClosedLoopRampPeriod, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Config_VoltageClosedLoopRampPeriod, VoltageClosedLoopRampPeriod, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Config_TorqueClosedLoopRampPeriod, TorqueClosedLoopRampPeriod, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return ss.str();
    }

    ctre::phoenix::StatusCode Deserialize(const std::string& to_deserialize) override
    {
        const char *string_c_str = to_deserialize.c_str();
        size_t string_length = to_deserialize.length();
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Config_DutyCycleClosedLoopRampPeriod, string_c_str, string_length, &DutyCycleClosedLoopRampPeriod);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Config_VoltageClosedLoopRampPeriod, string_c_str, string_length, &VoltageClosedLoopRampPeriod);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Config_TorqueClosedLoopRampPeriod, string_c_str, string_length, &TorqueClosedLoopRampPeriod);
        return 0;
    }
};


/**
 * \brief Configs that change how the motor controller behaves under
 *        different limit switch statse.
 * 
 * \details Includes configs such as enabling limit switches,
 *          configuring the remote sensor ID, the source, and the
 *          position to set on limit.
 */
class HardwareLimitSwitchConfigs : public ParentConfiguration
{
public:
    /**
     * \brief Determines if limit is normally-open (default) or
     * normally-closed.
     * 
     */
    signals::ForwardLimitTypeValue ForwardLimitType = signals::ForwardLimitTypeValue::NormallyOpen;
    /**
     * \brief If enabled, the position is auto-set to a specific value,
     * specified by ForwardLimitAutosetPositionValue
     * 
     *   Default Value: False
     */
    bool ForwardLimitAutosetPositionEnable = false;
    /**
     * \brief The value to auto-set the position to.  This has no effect
     * if ForwardLimitAutosetPositionEnable is false.
     * 
     *   Minimum Value: -3.4e+38
     *   Maximum Value: 3.4e+38
     *   Default Value: 0
     *   Units: rotations
     */
    double ForwardLimitAutosetPositionValue = 0;
    /**
     * \brief If enabled, motor output is set to neutral when forward
     * limit switch is asseted and positive output is requested.
     * 
     *   Default Value: True
     */
    bool ForwardLimitEnable = true;
    /**
     * \brief Determines where to poll the forward limit switch.  This
     * defaults to the limit switch pin on the limit switch connector.
     * 
     */
    signals::ForwardLimitSourceValue ForwardLimitSource = signals::ForwardLimitSourceValue::LimitSwitchPin;
    /**
     * \brief Device ID of the device if using remote limit switch
     * features.
     * 
     *   Minimum Value: 0
     *   Maximum Value: 62
     *   Default Value: 0
     *   Units: 
     */
    int ForwardLimitRemoteSensorID = 0;
    /**
     * \brief Determines if limit is normally-open (default) or
     * normally-closed.
     * 
     */
    signals::ReverseLimitTypeValue ReverseLimitType = signals::ReverseLimitTypeValue::NormallyOpen;
    /**
     * \brief If enabled, the position is auto-set to a specific value,
     * specified by ReverseLimitAutosetPositionValue
     * 
     *   Default Value: False
     */
    bool ReverseLimitAutosetPositionEnable = false;
    /**
     * \brief The value to auto-set the position to.  This has no effect
     * if ReverseLimitAutosetPositionEnable is false.
     * 
     *   Minimum Value: -3.4e+38
     *   Maximum Value: 3.4e+38
     *   Default Value: 0
     *   Units: rotations
     */
    double ReverseLimitAutosetPositionValue = 0;
    /**
     * \brief If enabled, motor output is set to neutral when reverse
     * limit switch is asseted and positive output is requested.
     * 
     *   Default Value: True
     */
    bool ReverseLimitEnable = true;
    /**
     * \brief Determines where to poll the reverse limit switch.  This
     * defaults to the limit switch pin on the limit switch connector.
     * 
     */
    signals::ReverseLimitSourceValue ReverseLimitSource = signals::ReverseLimitSourceValue::LimitSwitchPin;
    /**
     * \brief Device ID of the device if using remote limit switch
     * features.
     * 
     *   Minimum Value: 0
     *   Maximum Value: 62
     *   Default Value: 0
     *   Units: 
     */
    int ReverseLimitRemoteSensorID = 0;
    
    /**
     * \brief Modifies this configuration's ForwardLimitType parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newForwardLimitType Parameter to modify
     * \returns Itself
     */
    HardwareLimitSwitchConfigs& WithForwardLimitType(signals::ForwardLimitTypeValue newForwardLimitType)
    {
        ForwardLimitType = std::move(newForwardLimitType);
        return *this;
    }
    /**
     * \brief Modifies this configuration's ForwardLimitAutosetPositionEnable parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newForwardLimitAutosetPositionEnable Parameter to modify
     * \returns Itself
     */
    HardwareLimitSwitchConfigs& WithForwardLimitAutosetPositionEnable(bool newForwardLimitAutosetPositionEnable)
    {
        ForwardLimitAutosetPositionEnable = std::move(newForwardLimitAutosetPositionEnable);
        return *this;
    }
    /**
     * \brief Modifies this configuration's ForwardLimitAutosetPositionValue parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newForwardLimitAutosetPositionValue Parameter to modify
     * \returns Itself
     */
    HardwareLimitSwitchConfigs& WithForwardLimitAutosetPositionValue(double newForwardLimitAutosetPositionValue)
    {
        ForwardLimitAutosetPositionValue = std::move(newForwardLimitAutosetPositionValue);
        return *this;
    }
    /**
     * \brief Modifies this configuration's ForwardLimitEnable parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newForwardLimitEnable Parameter to modify
     * \returns Itself
     */
    HardwareLimitSwitchConfigs& WithForwardLimitEnable(bool newForwardLimitEnable)
    {
        ForwardLimitEnable = std::move(newForwardLimitEnable);
        return *this;
    }
    /**
     * \brief Modifies this configuration's ForwardLimitSource parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newForwardLimitSource Parameter to modify
     * \returns Itself
     */
    HardwareLimitSwitchConfigs& WithForwardLimitSource(signals::ForwardLimitSourceValue newForwardLimitSource)
    {
        ForwardLimitSource = std::move(newForwardLimitSource);
        return *this;
    }
    /**
     * \brief Modifies this configuration's ForwardLimitRemoteSensorID parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newForwardLimitRemoteSensorID Parameter to modify
     * \returns Itself
     */
    HardwareLimitSwitchConfigs& WithForwardLimitRemoteSensorID(int newForwardLimitRemoteSensorID)
    {
        ForwardLimitRemoteSensorID = std::move(newForwardLimitRemoteSensorID);
        return *this;
    }
    /**
     * \brief Modifies this configuration's ReverseLimitType parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newReverseLimitType Parameter to modify
     * \returns Itself
     */
    HardwareLimitSwitchConfigs& WithReverseLimitType(signals::ReverseLimitTypeValue newReverseLimitType)
    {
        ReverseLimitType = std::move(newReverseLimitType);
        return *this;
    }
    /**
     * \brief Modifies this configuration's ReverseLimitAutosetPositionEnable parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newReverseLimitAutosetPositionEnable Parameter to modify
     * \returns Itself
     */
    HardwareLimitSwitchConfigs& WithReverseLimitAutosetPositionEnable(bool newReverseLimitAutosetPositionEnable)
    {
        ReverseLimitAutosetPositionEnable = std::move(newReverseLimitAutosetPositionEnable);
        return *this;
    }
    /**
     * \brief Modifies this configuration's ReverseLimitAutosetPositionValue parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newReverseLimitAutosetPositionValue Parameter to modify
     * \returns Itself
     */
    HardwareLimitSwitchConfigs& WithReverseLimitAutosetPositionValue(double newReverseLimitAutosetPositionValue)
    {
        ReverseLimitAutosetPositionValue = std::move(newReverseLimitAutosetPositionValue);
        return *this;
    }
    /**
     * \brief Modifies this configuration's ReverseLimitEnable parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newReverseLimitEnable Parameter to modify
     * \returns Itself
     */
    HardwareLimitSwitchConfigs& WithReverseLimitEnable(bool newReverseLimitEnable)
    {
        ReverseLimitEnable = std::move(newReverseLimitEnable);
        return *this;
    }
    /**
     * \brief Modifies this configuration's ReverseLimitSource parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newReverseLimitSource Parameter to modify
     * \returns Itself
     */
    HardwareLimitSwitchConfigs& WithReverseLimitSource(signals::ReverseLimitSourceValue newReverseLimitSource)
    {
        ReverseLimitSource = std::move(newReverseLimitSource);
        return *this;
    }
    /**
     * \brief Modifies this configuration's ReverseLimitRemoteSensorID parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newReverseLimitRemoteSensorID Parameter to modify
     * \returns Itself
     */
    HardwareLimitSwitchConfigs& WithReverseLimitRemoteSensorID(int newReverseLimitRemoteSensorID)
    {
        ReverseLimitRemoteSensorID = std::move(newReverseLimitRemoteSensorID);
        return *this;
    }

    

    std::string ToString() const override
    {
        std::stringstream ss;
        ss << "{" << std::endl;
        ss << "Config Group: HardwareLimitSwitch" << std::endl;
        ss << "Name: \"ForwardLimitType\" Value: \"" << ForwardLimitType << "\"" << std::endl;
        ss << "Name: \"ForwardLimitAutosetPositionEnable\" Value: \"" << ForwardLimitAutosetPositionEnable << "\"" << std::endl;
        ss << "Name: \"ForwardLimitAutosetPositionValue\" Value: \"" << ForwardLimitAutosetPositionValue << "rotations\"" << std::endl;
        ss << "Name: \"ForwardLimitEnable\" Value: \"" << ForwardLimitEnable << "\"" << std::endl;
        ss << "Name: \"ForwardLimitSource\" Value: \"" << ForwardLimitSource << "\"" << std::endl;
        ss << "Name: \"ForwardLimitRemoteSensorID\" Value: \"" << ForwardLimitRemoteSensorID << "\"" << std::endl;
        ss << "Name: \"ReverseLimitType\" Value: \"" << ReverseLimitType << "\"" << std::endl;
        ss << "Name: \"ReverseLimitAutosetPositionEnable\" Value: \"" << ReverseLimitAutosetPositionEnable << "\"" << std::endl;
        ss << "Name: \"ReverseLimitAutosetPositionValue\" Value: \"" << ReverseLimitAutosetPositionValue << "rotations\"" << std::endl;
        ss << "Name: \"ReverseLimitEnable\" Value: \"" << ReverseLimitEnable << "\"" << std::endl;
        ss << "Name: \"ReverseLimitSource\" Value: \"" << ReverseLimitSource << "\"" << std::endl;
        ss << "Name: \"ReverseLimitRemoteSensorID\" Value: \"" << ReverseLimitRemoteSensorID << "\"" << std::endl;
        ss << "}" << std::endl;
        return ss.str();
    }

    std::string Serialize() const override
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_int(ctre::phoenix6::spns::SpnValue::Config_ForwardLimitType, ForwardLimitType.value, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_bool(ctre::phoenix6::spns::SpnValue::Config_ForwardLimitAutosetPosEnable, ForwardLimitAutosetPositionEnable, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Config_ForwardLimitAutosetPosValue, ForwardLimitAutosetPositionValue, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_bool(ctre::phoenix6::spns::SpnValue::Config_ForwardLimitEnable, ForwardLimitEnable, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_int(ctre::phoenix6::spns::SpnValue::Config_ForwardLimitSource, ForwardLimitSource.value, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_int(ctre::phoenix6::spns::SpnValue::Config_ForwardLimitRemoteSensorID, ForwardLimitRemoteSensorID, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_int(ctre::phoenix6::spns::SpnValue::Config_ReverseLimitType, ReverseLimitType.value, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_bool(ctre::phoenix6::spns::SpnValue::Config_ReverseLimitAutosetPosEnable, ReverseLimitAutosetPositionEnable, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Config_ReverseLimitAutosetPosValue, ReverseLimitAutosetPositionValue, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_bool(ctre::phoenix6::spns::SpnValue::Config_ReverseLimitEnable, ReverseLimitEnable, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_int(ctre::phoenix6::spns::SpnValue::Config_ReverseLimitSource, ReverseLimitSource.value, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_int(ctre::phoenix6::spns::SpnValue::Config_ReverseLimitRemoteSensorID, ReverseLimitRemoteSensorID, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return ss.str();
    }

    ctre::phoenix::StatusCode Deserialize(const std::string& to_deserialize) override
    {
        const char *string_c_str = to_deserialize.c_str();
        size_t string_length = to_deserialize.length();
        c_ctre_phoenix6_deserialize_int(ctre::phoenix6::spns::SpnValue::Config_ForwardLimitType, string_c_str, string_length, &ForwardLimitType.value);
        c_ctre_phoenix6_deserialize_bool(ctre::phoenix6::spns::SpnValue::Config_ForwardLimitAutosetPosEnable, string_c_str, string_length, &ForwardLimitAutosetPositionEnable);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Config_ForwardLimitAutosetPosValue, string_c_str, string_length, &ForwardLimitAutosetPositionValue);
        c_ctre_phoenix6_deserialize_bool(ctre::phoenix6::spns::SpnValue::Config_ForwardLimitEnable, string_c_str, string_length, &ForwardLimitEnable);
        c_ctre_phoenix6_deserialize_int(ctre::phoenix6::spns::SpnValue::Config_ForwardLimitSource, string_c_str, string_length, &ForwardLimitSource.value);
        c_ctre_phoenix6_deserialize_int(ctre::phoenix6::spns::SpnValue::Config_ForwardLimitRemoteSensorID, string_c_str, string_length, &ForwardLimitRemoteSensorID);
        c_ctre_phoenix6_deserialize_int(ctre::phoenix6::spns::SpnValue::Config_ReverseLimitType, string_c_str, string_length, &ReverseLimitType.value);
        c_ctre_phoenix6_deserialize_bool(ctre::phoenix6::spns::SpnValue::Config_ReverseLimitAutosetPosEnable, string_c_str, string_length, &ReverseLimitAutosetPositionEnable);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Config_ReverseLimitAutosetPosValue, string_c_str, string_length, &ReverseLimitAutosetPositionValue);
        c_ctre_phoenix6_deserialize_bool(ctre::phoenix6::spns::SpnValue::Config_ReverseLimitEnable, string_c_str, string_length, &ReverseLimitEnable);
        c_ctre_phoenix6_deserialize_int(ctre::phoenix6::spns::SpnValue::Config_ReverseLimitSource, string_c_str, string_length, &ReverseLimitSource.value);
        c_ctre_phoenix6_deserialize_int(ctre::phoenix6::spns::SpnValue::Config_ReverseLimitRemoteSensorID, string_c_str, string_length, &ReverseLimitRemoteSensorID);
        return 0;
    }
};


/**
 * \brief Configs that affect audible components of the device.
 * 
 * \details Includes configuration for the beep on boot.
 */
class AudioConfigs : public ParentConfiguration
{
public:
    /**
     * \brief If true, the TalonFX will beep during boot-up.  This is
     * useful for general debugging, and defaults to true.  If rotor is
     * moving during boot-up, the beep will not occur regardless of this
     * setting.
     * 
     *   Default Value: True
     */
    bool BeepOnBoot = true;
    /**
     * \brief If true, the TalonFX will beep during configuration API
     * calls if device is disabled.  This is useful for general debugging,
     * and defaults to true.  Note that if the rotor is moving, the beep
     * will not occur regardless of this setting.
     * 
     *   Default Value: True
     */
    bool BeepOnConfig = true;
    /**
     * \brief If true, the TalonFX will allow Orchestra and MusicTone
     * requests during disabled state.  This can be used to address corner
     * cases when music features are needed when disabled.  This setting
     * defaults to false.  Note that if the rotor is moving, music
     * features are always disabled regardless of this setting.
     * 
     *   Default Value: False
     */
    bool AllowMusicDurDisable = false;
    
    /**
     * \brief Modifies this configuration's BeepOnBoot parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newBeepOnBoot Parameter to modify
     * \returns Itself
     */
    AudioConfigs& WithBeepOnBoot(bool newBeepOnBoot)
    {
        BeepOnBoot = std::move(newBeepOnBoot);
        return *this;
    }
    /**
     * \brief Modifies this configuration's BeepOnConfig parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newBeepOnConfig Parameter to modify
     * \returns Itself
     */
    AudioConfigs& WithBeepOnConfig(bool newBeepOnConfig)
    {
        BeepOnConfig = std::move(newBeepOnConfig);
        return *this;
    }
    /**
     * \brief Modifies this configuration's AllowMusicDurDisable parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newAllowMusicDurDisable Parameter to modify
     * \returns Itself
     */
    AudioConfigs& WithAllowMusicDurDisable(bool newAllowMusicDurDisable)
    {
        AllowMusicDurDisable = std::move(newAllowMusicDurDisable);
        return *this;
    }

    

    std::string ToString() const override
    {
        std::stringstream ss;
        ss << "{" << std::endl;
        ss << "Config Group: Audio" << std::endl;
        ss << "Name: \"BeepOnBoot\" Value: \"" << BeepOnBoot << "\"" << std::endl;
        ss << "Name: \"BeepOnConfig\" Value: \"" << BeepOnConfig << "\"" << std::endl;
        ss << "Name: \"AllowMusicDurDisable\" Value: \"" << AllowMusicDurDisable << "\"" << std::endl;
        ss << "}" << std::endl;
        return ss.str();
    }

    std::string Serialize() const override
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_bool(ctre::phoenix6::spns::SpnValue::Config_BeepOnBoot, BeepOnBoot, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_bool(ctre::phoenix6::spns::SpnValue::Config_BeepOnConfig, BeepOnConfig, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_bool(ctre::phoenix6::spns::SpnValue::Config_AllowMusicDurDisable, AllowMusicDurDisable, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return ss.str();
    }

    ctre::phoenix::StatusCode Deserialize(const std::string& to_deserialize) override
    {
        const char *string_c_str = to_deserialize.c_str();
        size_t string_length = to_deserialize.length();
        c_ctre_phoenix6_deserialize_bool(ctre::phoenix6::spns::SpnValue::Config_BeepOnBoot, string_c_str, string_length, &BeepOnBoot);
        c_ctre_phoenix6_deserialize_bool(ctre::phoenix6::spns::SpnValue::Config_BeepOnConfig, string_c_str, string_length, &BeepOnConfig);
        c_ctre_phoenix6_deserialize_bool(ctre::phoenix6::spns::SpnValue::Config_AllowMusicDurDisable, string_c_str, string_length, &AllowMusicDurDisable);
        return 0;
    }
};


/**
 * \brief Configs that affect how software-limit switches behave.
 * 
 * \details Includes enabling software-limit switches and the
 *          threshold at which they're tripped.
 */
class SoftwareLimitSwitchConfigs : public ParentConfiguration
{
public:
    /**
     * \brief If enabled, the motor output is set to neutral if position
     * exceeds ForwardSoftLimitThreshold and forward output is requested.
     * 
     *   Default Value: False
     */
    bool ForwardSoftLimitEnable = false;
    /**
     * \brief If enabled, the motor output is set to neutral if position
     * exceeds ReverseSoftLimitThreshold and reverse output is requested.
     * 
     *   Default Value: False
     */
    bool ReverseSoftLimitEnable = false;
    /**
     * \brief Position threshold for forward soft limit features.
     * ForwardSoftLimitEnable must be enabled for this to take effect.
     * 
     *   Minimum Value: -3.4e+38
     *   Maximum Value: 3.4e+38
     *   Default Value: 0
     *   Units: rotations
     */
    double ForwardSoftLimitThreshold = 0;
    /**
     * \brief Position threshold for reverse soft limit features.
     * ReverseSoftLimitEnable must be enabled for this to take effect.
     * 
     *   Minimum Value: -3.4e+38
     *   Maximum Value: 3.4e+38
     *   Default Value: 0
     *   Units: rotations
     */
    double ReverseSoftLimitThreshold = 0;
    
    /**
     * \brief Modifies this configuration's ForwardSoftLimitEnable parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newForwardSoftLimitEnable Parameter to modify
     * \returns Itself
     */
    SoftwareLimitSwitchConfigs& WithForwardSoftLimitEnable(bool newForwardSoftLimitEnable)
    {
        ForwardSoftLimitEnable = std::move(newForwardSoftLimitEnable);
        return *this;
    }
    /**
     * \brief Modifies this configuration's ReverseSoftLimitEnable parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newReverseSoftLimitEnable Parameter to modify
     * \returns Itself
     */
    SoftwareLimitSwitchConfigs& WithReverseSoftLimitEnable(bool newReverseSoftLimitEnable)
    {
        ReverseSoftLimitEnable = std::move(newReverseSoftLimitEnable);
        return *this;
    }
    /**
     * \brief Modifies this configuration's ForwardSoftLimitThreshold parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newForwardSoftLimitThreshold Parameter to modify
     * \returns Itself
     */
    SoftwareLimitSwitchConfigs& WithForwardSoftLimitThreshold(double newForwardSoftLimitThreshold)
    {
        ForwardSoftLimitThreshold = std::move(newForwardSoftLimitThreshold);
        return *this;
    }
    /**
     * \brief Modifies this configuration's ReverseSoftLimitThreshold parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newReverseSoftLimitThreshold Parameter to modify
     * \returns Itself
     */
    SoftwareLimitSwitchConfigs& WithReverseSoftLimitThreshold(double newReverseSoftLimitThreshold)
    {
        ReverseSoftLimitThreshold = std::move(newReverseSoftLimitThreshold);
        return *this;
    }

    

    std::string ToString() const override
    {
        std::stringstream ss;
        ss << "{" << std::endl;
        ss << "Config Group: SoftwareLimitSwitch" << std::endl;
        ss << "Name: \"ForwardSoftLimitEnable\" Value: \"" << ForwardSoftLimitEnable << "\"" << std::endl;
        ss << "Name: \"ReverseSoftLimitEnable\" Value: \"" << ReverseSoftLimitEnable << "\"" << std::endl;
        ss << "Name: \"ForwardSoftLimitThreshold\" Value: \"" << ForwardSoftLimitThreshold << "rotations\"" << std::endl;
        ss << "Name: \"ReverseSoftLimitThreshold\" Value: \"" << ReverseSoftLimitThreshold << "rotations\"" << std::endl;
        ss << "}" << std::endl;
        return ss.str();
    }

    std::string Serialize() const override
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_bool(ctre::phoenix6::spns::SpnValue::Config_ForwardSoftLimitEnable, ForwardSoftLimitEnable, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_bool(ctre::phoenix6::spns::SpnValue::Config_ReverseSoftLimitEnable, ReverseSoftLimitEnable, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Config_ForwardSoftLimitThreshold, ForwardSoftLimitThreshold, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Config_ReverseSoftLimitThreshold, ReverseSoftLimitThreshold, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return ss.str();
    }

    ctre::phoenix::StatusCode Deserialize(const std::string& to_deserialize) override
    {
        const char *string_c_str = to_deserialize.c_str();
        size_t string_length = to_deserialize.length();
        c_ctre_phoenix6_deserialize_bool(ctre::phoenix6::spns::SpnValue::Config_ForwardSoftLimitEnable, string_c_str, string_length, &ForwardSoftLimitEnable);
        c_ctre_phoenix6_deserialize_bool(ctre::phoenix6::spns::SpnValue::Config_ReverseSoftLimitEnable, string_c_str, string_length, &ReverseSoftLimitEnable);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Config_ForwardSoftLimitThreshold, string_c_str, string_length, &ForwardSoftLimitThreshold);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Config_ReverseSoftLimitThreshold, string_c_str, string_length, &ReverseSoftLimitThreshold);
        return 0;
    }
};


/**
 * \brief Configs for Motion Magic®.
 * 
 * \details Includes Velocity, Acceleration, Jerk, and Expo
 *          parameters.
 */
class MotionMagicConfigs : public ParentConfiguration
{
public:
    /**
     * \brief This is the maximum velocity Motion Magic® based control
     * modes are allowed to use.  Motion Magic® Velocity control modes do
     * not use this config.  When using Motion Magic® Expo control modes,
     * setting this to 0 will allow the profile to run to the max possible
     * velocity based on Expo_kV.
     * 
     *   Minimum Value: 0
     *   Maximum Value: 9999
     *   Default Value: 0
     *   Units: rps
     */
    double MotionMagicCruiseVelocity = 0;
    /**
     * \brief This is the target acceleration Motion Magic® based control
     * modes are allowed to use.  Motion Magic® Expo control modes do not
     * use this config.
     * 
     *   Minimum Value: 0
     *   Maximum Value: 9999
     *   Default Value: 0
     *   Units: rot per sec²
     */
    double MotionMagicAcceleration = 0;
    /**
     * \brief This is the target jerk (acceleration derivative) Motion
     * Magic® based control modes are allowed to use.  Motion Magic® Expo
     * control modes do not use this config.  This allows Motion Magic®
     * support of S-Curves.  If this is set to zero, then Motion Magic®
     * will not apply a Jerk limit.
     * 
     *   Minimum Value: 0
     *   Maximum Value: 9999
     *   Default Value: 0
     *   Units: rot per sec³
     */
    double MotionMagicJerk = 0;
    /**
     * \brief This is the target kV used only by Motion Magic® Expo
     * control modes, in units of V/rps. This represents the amount of
     * voltage necessary to hold a velocity. In terms of the Motion Magic®
     * Expo profile, a higher kV results in a slower maximum velocity. A
     * kV of 0 will be promoted to a reasonable default of 0.12.
     * 
     *   Minimum Value: 0.001
     *   Maximum Value: 100
     *   Default Value: 0
     *   Units: V/rps
     */
    double MotionMagicExpo_kV = 0;
    /**
     * \brief This is the target kA used only by Motion Magic® Expo
     * control modes, in units of V/rps². This represents the amount of
     * voltage necessary to achieve an acceleration. In terms of the
     * Motion Magic® Expo profile, a higher kA results in a slower
     * acceleration. A kA of 0 will be promoted to a reasonable default of
     * 0.1.
     * 
     *   Minimum Value: 1e-05
     *   Maximum Value: 100
     *   Default Value: 0
     *   Units: V/rps²
     */
    double MotionMagicExpo_kA = 0;
    
    /**
     * \brief Modifies this configuration's MotionMagicCruiseVelocity parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newMotionMagicCruiseVelocity Parameter to modify
     * \returns Itself
     */
    MotionMagicConfigs& WithMotionMagicCruiseVelocity(double newMotionMagicCruiseVelocity)
    {
        MotionMagicCruiseVelocity = std::move(newMotionMagicCruiseVelocity);
        return *this;
    }
    /**
     * \brief Modifies this configuration's MotionMagicAcceleration parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newMotionMagicAcceleration Parameter to modify
     * \returns Itself
     */
    MotionMagicConfigs& WithMotionMagicAcceleration(double newMotionMagicAcceleration)
    {
        MotionMagicAcceleration = std::move(newMotionMagicAcceleration);
        return *this;
    }
    /**
     * \brief Modifies this configuration's MotionMagicJerk parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newMotionMagicJerk Parameter to modify
     * \returns Itself
     */
    MotionMagicConfigs& WithMotionMagicJerk(double newMotionMagicJerk)
    {
        MotionMagicJerk = std::move(newMotionMagicJerk);
        return *this;
    }
    /**
     * \brief Modifies this configuration's MotionMagicExpo_kV parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newMotionMagicExpo_kV Parameter to modify
     * \returns Itself
     */
    MotionMagicConfigs& WithMotionMagicExpo_kV(double newMotionMagicExpo_kV)
    {
        MotionMagicExpo_kV = std::move(newMotionMagicExpo_kV);
        return *this;
    }
    /**
     * \brief Modifies this configuration's MotionMagicExpo_kA parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newMotionMagicExpo_kA Parameter to modify
     * \returns Itself
     */
    MotionMagicConfigs& WithMotionMagicExpo_kA(double newMotionMagicExpo_kA)
    {
        MotionMagicExpo_kA = std::move(newMotionMagicExpo_kA);
        return *this;
    }

    

    std::string ToString() const override
    {
        std::stringstream ss;
        ss << "{" << std::endl;
        ss << "Config Group: MotionMagic" << std::endl;
        ss << "Name: \"MotionMagicCruiseVelocity\" Value: \"" << MotionMagicCruiseVelocity << "rps\"" << std::endl;
        ss << "Name: \"MotionMagicAcceleration\" Value: \"" << MotionMagicAcceleration << "rot per sec²\"" << std::endl;
        ss << "Name: \"MotionMagicJerk\" Value: \"" << MotionMagicJerk << "rot per sec³\"" << std::endl;
        ss << "Name: \"MotionMagicExpo_kV\" Value: \"" << MotionMagicExpo_kV << "V/rps\"" << std::endl;
        ss << "Name: \"MotionMagicExpo_kA\" Value: \"" << MotionMagicExpo_kA << "V/rps²\"" << std::endl;
        ss << "}" << std::endl;
        return ss.str();
    }

    std::string Serialize() const override
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Config_MotionMagicCruiseVelocity, MotionMagicCruiseVelocity, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Config_MotionMagicAcceleration, MotionMagicAcceleration, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Config_MotionMagicJerk, MotionMagicJerk, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Config_MotionMagicExpo_kV, MotionMagicExpo_kV, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Config_MotionMagicExpo_kA, MotionMagicExpo_kA, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return ss.str();
    }

    ctre::phoenix::StatusCode Deserialize(const std::string& to_deserialize) override
    {
        const char *string_c_str = to_deserialize.c_str();
        size_t string_length = to_deserialize.length();
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Config_MotionMagicCruiseVelocity, string_c_str, string_length, &MotionMagicCruiseVelocity);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Config_MotionMagicAcceleration, string_c_str, string_length, &MotionMagicAcceleration);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Config_MotionMagicJerk, string_c_str, string_length, &MotionMagicJerk);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Config_MotionMagicExpo_kV, string_c_str, string_length, &MotionMagicExpo_kV);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Config_MotionMagicExpo_kA, string_c_str, string_length, &MotionMagicExpo_kA);
        return 0;
    }
};


/**
 * \brief Custom Params.
 * 
 * \details Custom paramaters that have no real impact on controller.
 */
class CustomParamsConfigs : public ParentConfiguration
{
public:
    /**
     * \brief Custom parameter 0.  This is provided to allow
     * end-applications to store persistent information in the device.
     * 
     *   Minimum Value: -32768
     *   Maximum Value: 32767
     *   Default Value: 0
     *   Units: 
     */
    int CustomParam0 = 0;
    /**
     * \brief Custom parameter 1.  This is provided to allow
     * end-applications to store persistent information in the device.
     * 
     *   Minimum Value: -32768
     *   Maximum Value: 32767
     *   Default Value: 0
     *   Units: 
     */
    int CustomParam1 = 0;
    
    /**
     * \brief Modifies this configuration's CustomParam0 parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newCustomParam0 Parameter to modify
     * \returns Itself
     */
    CustomParamsConfigs& WithCustomParam0(int newCustomParam0)
    {
        CustomParam0 = std::move(newCustomParam0);
        return *this;
    }
    /**
     * \brief Modifies this configuration's CustomParam1 parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newCustomParam1 Parameter to modify
     * \returns Itself
     */
    CustomParamsConfigs& WithCustomParam1(int newCustomParam1)
    {
        CustomParam1 = std::move(newCustomParam1);
        return *this;
    }

    

    std::string ToString() const override
    {
        std::stringstream ss;
        ss << "{" << std::endl;
        ss << "Config Group: CustomParams" << std::endl;
        ss << "Name: \"CustomParam0\" Value: \"" << CustomParam0 << "\"" << std::endl;
        ss << "Name: \"CustomParam1\" Value: \"" << CustomParam1 << "\"" << std::endl;
        ss << "}" << std::endl;
        return ss.str();
    }

    std::string Serialize() const override
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_int(ctre::phoenix6::spns::SpnValue::CustomParam0, CustomParam0, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_int(ctre::phoenix6::spns::SpnValue::CustomParam1, CustomParam1, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return ss.str();
    }

    ctre::phoenix::StatusCode Deserialize(const std::string& to_deserialize) override
    {
        const char *string_c_str = to_deserialize.c_str();
        size_t string_length = to_deserialize.length();
        c_ctre_phoenix6_deserialize_int(ctre::phoenix6::spns::SpnValue::CustomParam0, string_c_str, string_length, &CustomParam0);
        c_ctre_phoenix6_deserialize_int(ctre::phoenix6::spns::SpnValue::CustomParam1, string_c_str, string_length, &CustomParam1);
        return 0;
    }
};


/**
 * \brief Configs that affect general behavior during closed-looping.
 * 
 * \details Includes Continuous Wrap features.
 */
class ClosedLoopGeneralConfigs : public ParentConfiguration
{
public:
    /**
     * \brief Wrap position error within [-0.5,+0.5) mechanism rotations. 
     * Typically used for continuous position closed-loops like swerve
     * azimuth.
     * 
     * \details This uses the mechanism rotation value. If there is a gear
     * ratio between the sensor and the mechanism, make sure to apply a
     * SensorToMechanismRatio so the closed loop operates on the full
     * rotation.
     * 
     *   Default Value: False
     */
    bool ContinuousWrap = false;


    

    std::string ToString() const override
    {
        std::stringstream ss;
        ss << "{" << std::endl;
        ss << "Config Group: ClosedLoopGeneral" << std::endl;
        ss << "Name: \"ContinuousWrap\" Value: \"" << ContinuousWrap << "\"" << std::endl;
        ss << "}" << std::endl;
        return ss.str();
    }

    std::string Serialize() const override
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_bool(ctre::phoenix6::spns::SpnValue::Config_ContinuousWrap, ContinuousWrap, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return ss.str();
    }

    ctre::phoenix::StatusCode Deserialize(const std::string& to_deserialize) override
    {
        const char *string_c_str = to_deserialize.c_str();
        size_t string_length = to_deserialize.length();
        c_ctre_phoenix6_deserialize_bool(ctre::phoenix6::spns::SpnValue::Config_ContinuousWrap, string_c_str, string_length, &ContinuousWrap);
        return 0;
    }
};


/**
 * \brief Gains for the specified slot.
 * 
 * \details If this slot is selected, these gains are used in closed
 *          loop control requests.
 */
class Slot0Configs : public ParentConfiguration
{
public:
    /**
     * \brief Proportional Gain
     * 
     * \details The units for this gain is dependent on the control mode.
     * Since this gain is multiplied by error in the input, the units
     * should be defined as units of output per unit of input error. For
     * example, when controlling velocity using a duty cycle closed loop,
     * the units for the proportional gain will be duty cycle per rps of
     * error, or 1/rps.
     * 
     *   Minimum Value: 0
     *   Maximum Value: 3.4e+38
     *   Default Value: 0
     *   Units: 
     */
    double kP = 0;
    /**
     * \brief Integral Gain
     * 
     * \details The units for this gain is dependent on the control mode.
     * Since this gain is multiplied by error in the input integrated over
     * time (in units of seconds), the units should be defined as units of
     * output per unit of integrated input error. For example, when
     * controlling velocity using a duty cycle closed loop, integrating
     * velocity over time results in rps * s = rotations. Therefore, the
     * units for the integral gain will be duty cycle per rotation of
     * accumulated error, or 1/rot.
     * 
     *   Minimum Value: 0
     *   Maximum Value: 3.4e+38
     *   Default Value: 0
     *   Units: 
     */
    double kI = 0;
    /**
     * \brief Derivative Gain
     * 
     * \details The units for this gain is dependent on the control mode.
     * Since this gain is multiplied by the derivative of error in the
     * input with respect to time (in units of seconds), the units should
     * be defined as units of output per unit of the differentiated input
     * error. For example, when controlling velocity using a duty cycle
     * closed loop, the derivative of velocity with respect to time is
     * rps/s, which is acceleration. Therefore, the units for the
     * derivative gain will be duty cycle per unit of acceleration error,
     * or 1/(rps/s).
     * 
     *   Minimum Value: 0
     *   Maximum Value: 3.4e+38
     *   Default Value: 0
     *   Units: 
     */
    double kD = 0;
    /**
     * \brief Static Feedforward Gain
     * 
     * \details This is added to the closed loop output. The unit for this
     * constant is dependent on the control mode, typically fractional
     * duty cycle, voltage, or torque current. The sign is typically
     * determined by reference velocity when using position and velocity
     * closed loop modes. However, when using position closed loop with
     * zero velocity reference (no motion profiling), application can
     * instead use the position closed loop error by setting the Static
     * Feedforward Sign configuration parameter.  In which case, we
     * recommend the minimal amount of kS, otherwise the motor output may
     * dither when closed loop error is near zero.
     * 
     *   Minimum Value: -512
     *   Maximum Value: 511
     *   Default Value: 0
     *   Units: 
     */
    double kS = 0;
    /**
     * \brief Velocity Feedforward Gain
     * 
     * \details The units for this gain is dependent on the control mode.
     * Since this gain is multiplied by the requested velocity, the units
     * should be defined as units of output per unit of requested input
     * velocity. For example, when controlling velocity using a duty cycle
     * closed loop, the units for the velocity feedfoward gain will be
     * duty cycle per requested rps, or 1/rps.
     * 
     *   Minimum Value: 0
     *   Maximum Value: 3.4e+38
     *   Default Value: 0
     *   Units: 
     */
    double kV = 0;
    /**
     * \brief Acceleration Feedforward Gain
     * 
     * \details The units for this gain is dependent on the control mode.
     * Since this gain is multiplied by the requested acceleration, the
     * units should be defined as units of output per unit of requested
     * input acceleration. For example, when controlling velocity using a
     * duty cycle closed loop, the units for the acceleration feedfoward
     * gain will be duty cycle per requested rps/s, or 1/(rps/s).
     * 
     *   Minimum Value: 0
     *   Maximum Value: 3.4e+38
     *   Default Value: 0
     *   Units: 
     */
    double kA = 0;
    /**
     * \brief Gravity Feedforward Gain
     * 
     * \details This is added to the closed loop output. The sign is
     * determined by the type of gravity feedforward. The unit for this
     * constant is dependent on the control mode, typically fractional
     * duty cycle, voltage, or torque current.
     * 
     *   Minimum Value: -512
     *   Maximum Value: 511
     *   Default Value: 0
     *   Units: 
     */
    double kG = 0;
    /**
     * \brief Gravity Feedforward Type
     * 
     * \details This determines the type of the gravity feedforward.
     * Choose Elevator_Static for systems where the gravity feedforward is
     * constant, such as an elevator. The gravity feedforward output will
     * always have the same sign. Choose Arm_Cosine for systems where the
     * gravity feedforward is dependent on the angular position of the
     * mechanism, such as an arm. The gravity feedforward output will vary
     * depending on the mechanism angular position. Note that the sensor
     * offset and ratios must be configured so that the sensor reports a
     * position of 0 when the mechanism is horizonal (parallel to the
     * ground), and the reported sensor position is 1:1 with the
     * mechanism.
     * 
     */
    signals::GravityTypeValue GravityType = signals::GravityTypeValue::Elevator_Static;
    /**
     * \brief Static Feedforward Sign during position closed loop
     * 
     * \details This determines the sign of the applied kS during position
     * closed-loop modes. The default behavior uses the velocity
     * feedforward sign. This works well with position closed loop when
     * velocity reference is specified (motion profiling). However, when
     * using position closed loop with zero velocity reference (no motion
     * profiling), the application may want to apply static feedforward
     * based on the closed loop error sign instead. In which case, we
     * recommend the minimal amount of kS, otherwise the motor output may
     * dither when closed loop error is near zero.
     * 
     */
    signals::StaticFeedforwardSignValue StaticFeedforwardSign = signals::StaticFeedforwardSignValue::UseVelocitySign;
    
    /**
     * \brief Modifies this configuration's kP parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newKP Parameter to modify
     * \returns Itself
     */
    Slot0Configs& WithKP(double newKP)
    {
        kP = std::move(newKP);
        return *this;
    }
    /**
     * \brief Modifies this configuration's kI parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newKI Parameter to modify
     * \returns Itself
     */
    Slot0Configs& WithKI(double newKI)
    {
        kI = std::move(newKI);
        return *this;
    }
    /**
     * \brief Modifies this configuration's kD parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newKD Parameter to modify
     * \returns Itself
     */
    Slot0Configs& WithKD(double newKD)
    {
        kD = std::move(newKD);
        return *this;
    }
    /**
     * \brief Modifies this configuration's kS parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newKS Parameter to modify
     * \returns Itself
     */
    Slot0Configs& WithKS(double newKS)
    {
        kS = std::move(newKS);
        return *this;
    }
    /**
     * \brief Modifies this configuration's kV parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newKV Parameter to modify
     * \returns Itself
     */
    Slot0Configs& WithKV(double newKV)
    {
        kV = std::move(newKV);
        return *this;
    }
    /**
     * \brief Modifies this configuration's kA parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newKA Parameter to modify
     * \returns Itself
     */
    Slot0Configs& WithKA(double newKA)
    {
        kA = std::move(newKA);
        return *this;
    }
    /**
     * \brief Modifies this configuration's kG parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newKG Parameter to modify
     * \returns Itself
     */
    Slot0Configs& WithKG(double newKG)
    {
        kG = std::move(newKG);
        return *this;
    }
    /**
     * \brief Modifies this configuration's GravityType parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newGravityType Parameter to modify
     * \returns Itself
     */
    Slot0Configs& WithGravityType(signals::GravityTypeValue newGravityType)
    {
        GravityType = std::move(newGravityType);
        return *this;
    }
    /**
     * \brief Modifies this configuration's StaticFeedforwardSign parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newStaticFeedforwardSign Parameter to modify
     * \returns Itself
     */
    Slot0Configs& WithStaticFeedforwardSign(signals::StaticFeedforwardSignValue newStaticFeedforwardSign)
    {
        StaticFeedforwardSign = std::move(newStaticFeedforwardSign);
        return *this;
    }

    static Slot0Configs From(const SlotConfigs& value);

    std::string ToString() const override
    {
        std::stringstream ss;
        ss << "{" << std::endl;
        ss << "Config Group: Slot0" << std::endl;
        ss << "Name: \"kP\" Value: \"" << kP << "\"" << std::endl;
        ss << "Name: \"kI\" Value: \"" << kI << "\"" << std::endl;
        ss << "Name: \"kD\" Value: \"" << kD << "\"" << std::endl;
        ss << "Name: \"kS\" Value: \"" << kS << "\"" << std::endl;
        ss << "Name: \"kV\" Value: \"" << kV << "\"" << std::endl;
        ss << "Name: \"kA\" Value: \"" << kA << "\"" << std::endl;
        ss << "Name: \"kG\" Value: \"" << kG << "\"" << std::endl;
        ss << "Name: \"GravityType\" Value: \"" << GravityType << "\"" << std::endl;
        ss << "Name: \"StaticFeedforwardSign\" Value: \"" << StaticFeedforwardSign << "\"" << std::endl;
        ss << "}" << std::endl;
        return ss.str();
    }

    std::string Serialize() const override
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Slot0_kP, kP, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Slot0_kI, kI, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Slot0_kD, kD, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Slot0_kS, kS, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Slot0_kV, kV, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Slot0_kA, kA, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Slot0_kG, kG, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_int(ctre::phoenix6::spns::SpnValue::Slot0_kG_Type, GravityType.value, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_int(ctre::phoenix6::spns::SpnValue::Slot0_kS_Sign, StaticFeedforwardSign.value, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return ss.str();
    }

    ctre::phoenix::StatusCode Deserialize(const std::string& to_deserialize) override
    {
        const char *string_c_str = to_deserialize.c_str();
        size_t string_length = to_deserialize.length();
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Slot0_kP, string_c_str, string_length, &kP);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Slot0_kI, string_c_str, string_length, &kI);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Slot0_kD, string_c_str, string_length, &kD);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Slot0_kS, string_c_str, string_length, &kS);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Slot0_kV, string_c_str, string_length, &kV);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Slot0_kA, string_c_str, string_length, &kA);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Slot0_kG, string_c_str, string_length, &kG);
        c_ctre_phoenix6_deserialize_int(ctre::phoenix6::spns::SpnValue::Slot0_kG_Type, string_c_str, string_length, &GravityType.value);
        c_ctre_phoenix6_deserialize_int(ctre::phoenix6::spns::SpnValue::Slot0_kS_Sign, string_c_str, string_length, &StaticFeedforwardSign.value);
        return 0;
    }
};


/**
 * \brief Gains for the specified slot.
 * 
 * \details If this slot is selected, these gains are used in closed
 *          loop control requests.
 */
class Slot1Configs : public ParentConfiguration
{
public:
    /**
     * \brief Proportional Gain
     * 
     * \details The units for this gain is dependent on the control mode.
     * Since this gain is multiplied by error in the input, the units
     * should be defined as units of output per unit of input error. For
     * example, when controlling velocity using a duty cycle closed loop,
     * the units for the proportional gain will be duty cycle per rps, or
     * 1/rps.
     * 
     *   Minimum Value: 0
     *   Maximum Value: 3.4e+38
     *   Default Value: 0
     *   Units: 
     */
    double kP = 0;
    /**
     * \brief Integral Gain
     * 
     * \details The units for this gain is dependent on the control mode.
     * Since this gain is multiplied by error in the input integrated over
     * time (in units of seconds), the units should be defined as units of
     * output per unit of integrated input error. For example, when
     * controlling velocity using a duty cycle closed loop, integrating
     * velocity over time results in rps * s = rotations. Therefore, the
     * units for the integral gain will be duty cycle per rotation of
     * accumulated error, or 1/rot.
     * 
     *   Minimum Value: 0
     *   Maximum Value: 3.4e+38
     *   Default Value: 0
     *   Units: 
     */
    double kI = 0;
    /**
     * \brief Derivative Gain
     * 
     * \details The units for this gain is dependent on the control mode.
     * Since this gain is multiplied by the derivative of error in the
     * input with respect to time (in units of seconds), the units should
     * be defined as units of output per unit of the differentiated input
     * error. For example, when controlling velocity using a duty cycle
     * closed loop, the derivative of velocity with respect to time is
     * rps/s, which is acceleration. Therefore, the units for the
     * derivative gain will be duty cycle per unit of acceleration error,
     * or 1/(rps/s).
     * 
     *   Minimum Value: 0
     *   Maximum Value: 3.4e+38
     *   Default Value: 0
     *   Units: 
     */
    double kD = 0;
    /**
     * \brief Static Feedforward Gain
     * 
     * \details This is added to the closed loop output. The unit for this
     * constant is dependent on the control mode, typically fractional
     * duty cycle, voltage, or torque current. The sign is typically
     * determined by reference velocity when using position and velocity
     * closed loop modes. However, when using position closed loop with
     * zero velocity reference (no motion profiling), application can
     * instead use the position closed loop error by setting the Static
     * Feedforward Sign configuration parameter.  In which case, we
     * recommend the minimal amount of kS, otherwise the motor output may
     * dither when closed loop error is near zero.
     * 
     *   Minimum Value: -512
     *   Maximum Value: 511
     *   Default Value: 0
     *   Units: 
     */
    double kS = 0;
    /**
     * \brief Velocity Feedforward Gain
     * 
     * \details The units for this gain is dependent on the control mode.
     * Since this gain is multiplied by the requested velocity, the units
     * should be defined as units of output per unit of requested input
     * velocity. For example, when controlling velocity using a duty cycle
     * closed loop, the units for the velocity feedfoward gain will be
     * duty cycle per requested rps, or 1/rps.
     * 
     *   Minimum Value: 0
     *   Maximum Value: 3.4e+38
     *   Default Value: 0
     *   Units: 
     */
    double kV = 0;
    /**
     * \brief Acceleration Feedforward Gain
     * 
     * \details The units for this gain is dependent on the control mode.
     * Since this gain is multiplied by the requested acceleration, the
     * units should be defined as units of output per unit of requested
     * input acceleration. For example, when controlling velocity using a
     * duty cycle closed loop, the units for the acceleration feedfoward
     * gain will be duty cycle per requested rps/s, or 1/(rps/s).
     * 
     *   Minimum Value: 0
     *   Maximum Value: 3.4e+38
     *   Default Value: 0
     *   Units: 
     */
    double kA = 0;
    /**
     * \brief Gravity Feedforward Gain
     * 
     * \details This is added to the closed loop output. The sign is
     * determined by the type of gravity feedforward. The unit for this
     * constant is dependent on the control mode, typically fractional
     * duty cycle, voltage, or torque current.
     * 
     *   Minimum Value: -512
     *   Maximum Value: 511
     *   Default Value: 0
     *   Units: 
     */
    double kG = 0;
    /**
     * \brief Gravity Feedforward Type
     * 
     * \details This determines the type of the gravity feedforward.
     * Choose Elevator_Static for systems where the gravity feedforward is
     * constant, such as an elevator. The gravity feedforward output will
     * always be positive. Choose Arm_Cosine for systems where the gravity
     * feedforward is dependent on the angular position of the mechanism,
     * such as an arm. The gravity feedforward output will vary depending
     * on the mechanism angular position. Note that the sensor offset and
     * ratios must be configured so that the sensor position is 0 when the
     * mechanism is horizonal, and one rotation of the mechanism
     * corresponds to one rotation of the sensor position.
     * 
     */
    signals::GravityTypeValue GravityType = signals::GravityTypeValue::Elevator_Static;
    /**
     * \brief Static Feedforward Sign during position closed loop
     * 
     * \details This determines the sign of the applied kS during position
     * closed-loop modes. The default behavior uses the velocity
     * feedforward sign. This works well with position closed loop when
     * velocity reference is specified (motion profiling). However, when
     * using position closed loop with zero velocity reference (no motion
     * profiling), the application may want to apply static feedforward
     * based on the closed loop error sign instead. In which case, we
     * recommend the minimal amount of kS, otherwise the motor output may
     * dither when closed loop error is near zero.
     * 
     */
    signals::StaticFeedforwardSignValue StaticFeedforwardSign = signals::StaticFeedforwardSignValue::UseVelocitySign;
    
    /**
     * \brief Modifies this configuration's kP parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newKP Parameter to modify
     * \returns Itself
     */
    Slot1Configs& WithKP(double newKP)
    {
        kP = std::move(newKP);
        return *this;
    }
    /**
     * \brief Modifies this configuration's kI parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newKI Parameter to modify
     * \returns Itself
     */
    Slot1Configs& WithKI(double newKI)
    {
        kI = std::move(newKI);
        return *this;
    }
    /**
     * \brief Modifies this configuration's kD parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newKD Parameter to modify
     * \returns Itself
     */
    Slot1Configs& WithKD(double newKD)
    {
        kD = std::move(newKD);
        return *this;
    }
    /**
     * \brief Modifies this configuration's kS parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newKS Parameter to modify
     * \returns Itself
     */
    Slot1Configs& WithKS(double newKS)
    {
        kS = std::move(newKS);
        return *this;
    }
    /**
     * \brief Modifies this configuration's kV parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newKV Parameter to modify
     * \returns Itself
     */
    Slot1Configs& WithKV(double newKV)
    {
        kV = std::move(newKV);
        return *this;
    }
    /**
     * \brief Modifies this configuration's kA parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newKA Parameter to modify
     * \returns Itself
     */
    Slot1Configs& WithKA(double newKA)
    {
        kA = std::move(newKA);
        return *this;
    }
    /**
     * \brief Modifies this configuration's kG parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newKG Parameter to modify
     * \returns Itself
     */
    Slot1Configs& WithKG(double newKG)
    {
        kG = std::move(newKG);
        return *this;
    }
    /**
     * \brief Modifies this configuration's GravityType parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newGravityType Parameter to modify
     * \returns Itself
     */
    Slot1Configs& WithGravityType(signals::GravityTypeValue newGravityType)
    {
        GravityType = std::move(newGravityType);
        return *this;
    }
    /**
     * \brief Modifies this configuration's StaticFeedforwardSign parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newStaticFeedforwardSign Parameter to modify
     * \returns Itself
     */
    Slot1Configs& WithStaticFeedforwardSign(signals::StaticFeedforwardSignValue newStaticFeedforwardSign)
    {
        StaticFeedforwardSign = std::move(newStaticFeedforwardSign);
        return *this;
    }

    static Slot1Configs From(const SlotConfigs& value);

    std::string ToString() const override
    {
        std::stringstream ss;
        ss << "{" << std::endl;
        ss << "Config Group: Slot1" << std::endl;
        ss << "Name: \"kP\" Value: \"" << kP << "\"" << std::endl;
        ss << "Name: \"kI\" Value: \"" << kI << "\"" << std::endl;
        ss << "Name: \"kD\" Value: \"" << kD << "\"" << std::endl;
        ss << "Name: \"kS\" Value: \"" << kS << "\"" << std::endl;
        ss << "Name: \"kV\" Value: \"" << kV << "\"" << std::endl;
        ss << "Name: \"kA\" Value: \"" << kA << "\"" << std::endl;
        ss << "Name: \"kG\" Value: \"" << kG << "\"" << std::endl;
        ss << "Name: \"GravityType\" Value: \"" << GravityType << "\"" << std::endl;
        ss << "Name: \"StaticFeedforwardSign\" Value: \"" << StaticFeedforwardSign << "\"" << std::endl;
        ss << "}" << std::endl;
        return ss.str();
    }

    std::string Serialize() const override
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Slot1_kP, kP, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Slot1_kI, kI, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Slot1_kD, kD, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Slot1_kS, kS, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Slot1_kV, kV, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Slot1_kA, kA, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Slot1_kG, kG, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_int(ctre::phoenix6::spns::SpnValue::Slot1_kG_Type, GravityType.value, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_int(ctre::phoenix6::spns::SpnValue::Slot1_kS_Sign, StaticFeedforwardSign.value, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return ss.str();
    }

    ctre::phoenix::StatusCode Deserialize(const std::string& to_deserialize) override
    {
        const char *string_c_str = to_deserialize.c_str();
        size_t string_length = to_deserialize.length();
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Slot1_kP, string_c_str, string_length, &kP);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Slot1_kI, string_c_str, string_length, &kI);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Slot1_kD, string_c_str, string_length, &kD);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Slot1_kS, string_c_str, string_length, &kS);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Slot1_kV, string_c_str, string_length, &kV);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Slot1_kA, string_c_str, string_length, &kA);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Slot1_kG, string_c_str, string_length, &kG);
        c_ctre_phoenix6_deserialize_int(ctre::phoenix6::spns::SpnValue::Slot1_kG_Type, string_c_str, string_length, &GravityType.value);
        c_ctre_phoenix6_deserialize_int(ctre::phoenix6::spns::SpnValue::Slot1_kS_Sign, string_c_str, string_length, &StaticFeedforwardSign.value);
        return 0;
    }
};


/**
 * \brief Gains for the specified slot.
 * 
 * \details If this slot is selected, these gains are used in closed
 *          loop control requests.
 */
class Slot2Configs : public ParentConfiguration
{
public:
    /**
     * \brief Proportional Gain
     * 
     * \details The units for this gain is dependent on the control mode.
     * Since this gain is multiplied by error in the input, the units
     * should be defined as units of output per unit of input error. For
     * example, when controlling velocity using a duty cycle closed loop,
     * the units for the proportional gain will be duty cycle per rps, or
     * 1/rps.
     * 
     *   Minimum Value: 0
     *   Maximum Value: 3.4e+38
     *   Default Value: 0
     *   Units: 
     */
    double kP = 0;
    /**
     * \brief Integral Gain
     * 
     * \details The units for this gain is dependent on the control mode.
     * Since this gain is multiplied by error in the input integrated over
     * time (in units of seconds), the units should be defined as units of
     * output per unit of integrated input error. For example, when
     * controlling velocity using a duty cycle closed loop, integrating
     * velocity over time results in rps * s = rotations. Therefore, the
     * units for the integral gain will be duty cycle per rotation of
     * accumulated error, or 1/rot.
     * 
     *   Minimum Value: 0
     *   Maximum Value: 3.4e+38
     *   Default Value: 0
     *   Units: 
     */
    double kI = 0;
    /**
     * \brief Derivative Gain
     * 
     * \details The units for this gain is dependent on the control mode.
     * Since this gain is multiplied by the derivative of error in the
     * input with respect to time (in units of seconds), the units should
     * be defined as units of output per unit of the differentiated input
     * error. For example, when controlling velocity using a duty cycle
     * closed loop, the derivative of velocity with respect to time is
     * rps/s, which is acceleration. Therefore, the units for the
     * derivative gain will be duty cycle per unit of acceleration error,
     * or 1/(rps/s).
     * 
     *   Minimum Value: 0
     *   Maximum Value: 3.4e+38
     *   Default Value: 0
     *   Units: 
     */
    double kD = 0;
    /**
     * \brief Static Feedforward Gain
     * 
     * \details This is added to the closed loop output. The unit for this
     * constant is dependent on the control mode, typically fractional
     * duty cycle, voltage, or torque current. The sign is typically
     * determined by reference velocity when using position and velocity
     * closed loop modes. However, when using position closed loop with
     * zero velocity reference (no motion profiling), application can
     * instead use the position closed loop error by setting the Static
     * Feedforward Sign configuration parameter.  In which case, we
     * recommend the minimal amount of kS, otherwise the motor output may
     * dither when closed loop error is near zero.
     * 
     *   Minimum Value: -512
     *   Maximum Value: 511
     *   Default Value: 0
     *   Units: 
     */
    double kS = 0;
    /**
     * \brief Velocity Feedforward Gain
     * 
     * \details The units for this gain is dependent on the control mode.
     * Since this gain is multiplied by the requested velocity, the units
     * should be defined as units of output per unit of requested input
     * velocity. For example, when controlling velocity using a duty cycle
     * closed loop, the units for the velocity feedfoward gain will be
     * duty cycle per requested rps, or 1/rps.
     * 
     *   Minimum Value: 0
     *   Maximum Value: 3.4e+38
     *   Default Value: 0
     *   Units: 
     */
    double kV = 0;
    /**
     * \brief Acceleration Feedforward Gain
     * 
     * \details The units for this gain is dependent on the control mode.
     * Since this gain is multiplied by the requested acceleration, the
     * units should be defined as units of output per unit of requested
     * input acceleration. For example, when controlling velocity using a
     * duty cycle closed loop, the units for the acceleration feedfoward
     * gain will be duty cycle per requested rps/s, or 1/(rps/s).
     * 
     *   Minimum Value: 0
     *   Maximum Value: 3.4e+38
     *   Default Value: 0
     *   Units: 
     */
    double kA = 0;
    /**
     * \brief Gravity Feedforward Gain
     * 
     * \details This is added to the closed loop output. The sign is
     * determined by the type of gravity feedforward. The unit for this
     * constant is dependent on the control mode, typically fractional
     * duty cycle, voltage, or torque current.
     * 
     *   Minimum Value: -512
     *   Maximum Value: 511
     *   Default Value: 0
     *   Units: 
     */
    double kG = 0;
    /**
     * \brief Gravity Feedforward Type
     * 
     * \details This determines the type of the gravity feedforward.
     * Choose Elevator_Static for systems where the gravity feedforward is
     * constant, such as an elevator. The gravity feedforward output will
     * always be positive. Choose Arm_Cosine for systems where the gravity
     * feedforward is dependent on the angular position of the mechanism,
     * such as an arm. The gravity feedforward output will vary depending
     * on the mechanism angular position. Note that the sensor offset and
     * ratios must be configured so that the sensor position is 0 when the
     * mechanism is horizonal, and one rotation of the mechanism
     * corresponds to one rotation of the sensor position.
     * 
     */
    signals::GravityTypeValue GravityType = signals::GravityTypeValue::Elevator_Static;
    /**
     * \brief Static Feedforward Sign during position closed loop
     * 
     * \details This determines the sign of the applied kS during position
     * closed-loop modes. The default behavior uses the velocity
     * feedforward sign. This works well with position closed loop when
     * velocity reference is specified (motion profiling). However, when
     * using position closed loop with zero velocity reference (no motion
     * profiling), the application may want to apply static feedforward
     * based on the closed loop error sign instead. In which case, we
     * recommend the minimal amount of kS, otherwise the motor output may
     * dither when closed loop error is near zero.
     * 
     */
    signals::StaticFeedforwardSignValue StaticFeedforwardSign = signals::StaticFeedforwardSignValue::UseVelocitySign;
    
    /**
     * \brief Modifies this configuration's kP parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newKP Parameter to modify
     * \returns Itself
     */
    Slot2Configs& WithKP(double newKP)
    {
        kP = std::move(newKP);
        return *this;
    }
    /**
     * \brief Modifies this configuration's kI parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newKI Parameter to modify
     * \returns Itself
     */
    Slot2Configs& WithKI(double newKI)
    {
        kI = std::move(newKI);
        return *this;
    }
    /**
     * \brief Modifies this configuration's kD parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newKD Parameter to modify
     * \returns Itself
     */
    Slot2Configs& WithKD(double newKD)
    {
        kD = std::move(newKD);
        return *this;
    }
    /**
     * \brief Modifies this configuration's kS parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newKS Parameter to modify
     * \returns Itself
     */
    Slot2Configs& WithKS(double newKS)
    {
        kS = std::move(newKS);
        return *this;
    }
    /**
     * \brief Modifies this configuration's kV parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newKV Parameter to modify
     * \returns Itself
     */
    Slot2Configs& WithKV(double newKV)
    {
        kV = std::move(newKV);
        return *this;
    }
    /**
     * \brief Modifies this configuration's kA parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newKA Parameter to modify
     * \returns Itself
     */
    Slot2Configs& WithKA(double newKA)
    {
        kA = std::move(newKA);
        return *this;
    }
    /**
     * \brief Modifies this configuration's kG parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newKG Parameter to modify
     * \returns Itself
     */
    Slot2Configs& WithKG(double newKG)
    {
        kG = std::move(newKG);
        return *this;
    }
    /**
     * \brief Modifies this configuration's GravityType parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newGravityType Parameter to modify
     * \returns Itself
     */
    Slot2Configs& WithGravityType(signals::GravityTypeValue newGravityType)
    {
        GravityType = std::move(newGravityType);
        return *this;
    }
    /**
     * \brief Modifies this configuration's StaticFeedforwardSign parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newStaticFeedforwardSign Parameter to modify
     * \returns Itself
     */
    Slot2Configs& WithStaticFeedforwardSign(signals::StaticFeedforwardSignValue newStaticFeedforwardSign)
    {
        StaticFeedforwardSign = std::move(newStaticFeedforwardSign);
        return *this;
    }

    static Slot2Configs From(const SlotConfigs& value);

    std::string ToString() const override
    {
        std::stringstream ss;
        ss << "{" << std::endl;
        ss << "Config Group: Slot2" << std::endl;
        ss << "Name: \"kP\" Value: \"" << kP << "\"" << std::endl;
        ss << "Name: \"kI\" Value: \"" << kI << "\"" << std::endl;
        ss << "Name: \"kD\" Value: \"" << kD << "\"" << std::endl;
        ss << "Name: \"kS\" Value: \"" << kS << "\"" << std::endl;
        ss << "Name: \"kV\" Value: \"" << kV << "\"" << std::endl;
        ss << "Name: \"kA\" Value: \"" << kA << "\"" << std::endl;
        ss << "Name: \"kG\" Value: \"" << kG << "\"" << std::endl;
        ss << "Name: \"GravityType\" Value: \"" << GravityType << "\"" << std::endl;
        ss << "Name: \"StaticFeedforwardSign\" Value: \"" << StaticFeedforwardSign << "\"" << std::endl;
        ss << "}" << std::endl;
        return ss.str();
    }

    std::string Serialize() const override
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Slot2_kP, kP, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Slot2_kI, kI, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Slot2_kD, kD, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Slot2_kS, kS, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Slot2_kV, kV, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Slot2_kA, kA, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::Slot2_kG, kG, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_int(ctre::phoenix6::spns::SpnValue::Slot2_kG_Type, GravityType.value, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_int(ctre::phoenix6::spns::SpnValue::Slot2_kS_Sign, StaticFeedforwardSign.value, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return ss.str();
    }

    ctre::phoenix::StatusCode Deserialize(const std::string& to_deserialize) override
    {
        const char *string_c_str = to_deserialize.c_str();
        size_t string_length = to_deserialize.length();
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Slot2_kP, string_c_str, string_length, &kP);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Slot2_kI, string_c_str, string_length, &kI);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Slot2_kD, string_c_str, string_length, &kD);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Slot2_kS, string_c_str, string_length, &kS);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Slot2_kV, string_c_str, string_length, &kV);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Slot2_kA, string_c_str, string_length, &kA);
        c_ctre_phoenix6_deserialize_double(ctre::phoenix6::spns::SpnValue::Slot2_kG, string_c_str, string_length, &kG);
        c_ctre_phoenix6_deserialize_int(ctre::phoenix6::spns::SpnValue::Slot2_kG_Type, string_c_str, string_length, &GravityType.value);
        c_ctre_phoenix6_deserialize_int(ctre::phoenix6::spns::SpnValue::Slot2_kS_Sign, string_c_str, string_length, &StaticFeedforwardSign.value);
        return 0;
    }
};

/**
 * \brief Gains for the specified slot.
 * 
 * \details If this slot is selected, these gains are used in closed
 *          loop control requests.
 */
class SlotConfigs : public ParentConfiguration
{
    struct SlotSpns
    {
        int kPSpn;
        int kISpn;
        int kDSpn;
        int kSSpn;
        int kVSpn;
        int kASpn;
        int kGSpn;
        int GravityTypeSpn;
        int StaticFeedforwardSignSpn;
    };

    std::map<int, SlotSpns> genericMap{
        {0, SlotSpns{
            ctre::phoenix6::spns::SpnValue::Slot0_kP,
            ctre::phoenix6::spns::SpnValue::Slot0_kI,
            ctre::phoenix6::spns::SpnValue::Slot0_kD,
            ctre::phoenix6::spns::SpnValue::Slot0_kS,
            ctre::phoenix6::spns::SpnValue::Slot0_kV,
            ctre::phoenix6::spns::SpnValue::Slot0_kA,
            ctre::phoenix6::spns::SpnValue::Slot0_kG,
            ctre::phoenix6::spns::SpnValue::Slot0_kG_Type,
            ctre::phoenix6::spns::SpnValue::Slot0_kS_Sign,
        }},
        
        {1, SlotSpns{
            ctre::phoenix6::spns::SpnValue::Slot1_kP,
            ctre::phoenix6::spns::SpnValue::Slot1_kI,
            ctre::phoenix6::spns::SpnValue::Slot1_kD,
            ctre::phoenix6::spns::SpnValue::Slot1_kS,
            ctre::phoenix6::spns::SpnValue::Slot1_kV,
            ctre::phoenix6::spns::SpnValue::Slot1_kA,
            ctre::phoenix6::spns::SpnValue::Slot1_kG,
            ctre::phoenix6::spns::SpnValue::Slot1_kG_Type,
            ctre::phoenix6::spns::SpnValue::Slot1_kS_Sign,
        }},
        
        {2, SlotSpns{
            ctre::phoenix6::spns::SpnValue::Slot2_kP,
            ctre::phoenix6::spns::SpnValue::Slot2_kI,
            ctre::phoenix6::spns::SpnValue::Slot2_kD,
            ctre::phoenix6::spns::SpnValue::Slot2_kS,
            ctre::phoenix6::spns::SpnValue::Slot2_kV,
            ctre::phoenix6::spns::SpnValue::Slot2_kA,
            ctre::phoenix6::spns::SpnValue::Slot2_kG,
            ctre::phoenix6::spns::SpnValue::Slot2_kG_Type,
            ctre::phoenix6::spns::SpnValue::Slot2_kS_Sign,
        }},
        
    };

public:
    /**
     * \brief Proportional Gain
     * 
     * \details The units for this gain is dependent on the control mode.
     * Since this gain is multiplied by error in the input, the units
     * should be defined as units of output per unit of input error. For
     * example, when controlling velocity using a duty cycle closed loop,
     * the units for the proportional gain will be duty cycle per rps of
     * error, or 1/rps.
     * 
     *   Minimum Value: 0
     *   Maximum Value: 3.4e+38
     *   Default Value: 0
     *   Units: 
     */
    double kP = 0;
    /**
     * \brief Integral Gain
     * 
     * \details The units for this gain is dependent on the control mode.
     * Since this gain is multiplied by error in the input integrated over
     * time (in units of seconds), the units should be defined as units of
     * output per unit of integrated input error. For example, when
     * controlling velocity using a duty cycle closed loop, integrating
     * velocity over time results in rps * s = rotations. Therefore, the
     * units for the integral gain will be duty cycle per rotation of
     * accumulated error, or 1/rot.
     * 
     *   Minimum Value: 0
     *   Maximum Value: 3.4e+38
     *   Default Value: 0
     *   Units: 
     */
    double kI = 0;
    /**
     * \brief Derivative Gain
     * 
     * \details The units for this gain is dependent on the control mode.
     * Since this gain is multiplied by the derivative of error in the
     * input with respect to time (in units of seconds), the units should
     * be defined as units of output per unit of the differentiated input
     * error. For example, when controlling velocity using a duty cycle
     * closed loop, the derivative of velocity with respect to time is
     * rps/s, which is acceleration. Therefore, the units for the
     * derivative gain will be duty cycle per unit of acceleration error,
     * or 1/(rps/s).
     * 
     *   Minimum Value: 0
     *   Maximum Value: 3.4e+38
     *   Default Value: 0
     *   Units: 
     */
    double kD = 0;
    /**
     * \brief Static Feedforward Gain
     * 
     * \details This is added to the closed loop output. The unit for this
     * constant is dependent on the control mode, typically fractional
     * duty cycle, voltage, or torque current. The sign is typically
     * determined by reference velocity when using position and velocity
     * closed loop modes. However, when using position closed loop with
     * zero velocity reference (no motion profiling), application can
     * instead use the position closed loop error by setting the Static
     * Feedforward Sign configuration parameter.  In which case, we
     * recommend the minimal amount of kS, otherwise the motor output may
     * dither when closed loop error is near zero.
     * 
     *   Minimum Value: -512
     *   Maximum Value: 511
     *   Default Value: 0
     *   Units: 
     */
    double kS = 0;
    /**
     * \brief Velocity Feedforward Gain
     * 
     * \details The units for this gain is dependent on the control mode.
     * Since this gain is multiplied by the requested velocity, the units
     * should be defined as units of output per unit of requested input
     * velocity. For example, when controlling velocity using a duty cycle
     * closed loop, the units for the velocity feedfoward gain will be
     * duty cycle per requested rps, or 1/rps.
     * 
     *   Minimum Value: 0
     *   Maximum Value: 3.4e+38
     *   Default Value: 0
     *   Units: 
     */
    double kV = 0;
    /**
     * \brief Acceleration Feedforward Gain
     * 
     * \details The units for this gain is dependent on the control mode.
     * Since this gain is multiplied by the requested acceleration, the
     * units should be defined as units of output per unit of requested
     * input acceleration. For example, when controlling velocity using a
     * duty cycle closed loop, the units for the acceleration feedfoward
     * gain will be duty cycle per requested rps/s, or 1/(rps/s).
     * 
     *   Minimum Value: 0
     *   Maximum Value: 3.4e+38
     *   Default Value: 0
     *   Units: 
     */
    double kA = 0;
    /**
     * \brief Gravity Feedforward Gain
     * 
     * \details This is added to the closed loop output. The sign is
     * determined by the type of gravity feedforward. The unit for this
     * constant is dependent on the control mode, typically fractional
     * duty cycle, voltage, or torque current.
     * 
     *   Minimum Value: -512
     *   Maximum Value: 511
     *   Default Value: 0
     *   Units: 
     */
    double kG = 0;
    /**
     * \brief Gravity Feedforward Type
     * 
     * \details This determines the type of the gravity feedforward.
     * Choose Elevator_Static for systems where the gravity feedforward is
     * constant, such as an elevator. The gravity feedforward output will
     * always have the same sign. Choose Arm_Cosine for systems where the
     * gravity feedforward is dependent on the angular position of the
     * mechanism, such as an arm. The gravity feedforward output will vary
     * depending on the mechanism angular position. Note that the sensor
     * offset and ratios must be configured so that the sensor reports a
     * position of 0 when the mechanism is horizonal (parallel to the
     * ground), and the reported sensor position is 1:1 with the
     * mechanism.
     * 
     */
    signals::GravityTypeValue GravityType = signals::GravityTypeValue::Elevator_Static;
    /**
     * \brief Static Feedforward Sign during position closed loop
     * 
     * \details This determines the sign of the applied kS during position
     * closed-loop modes. The default behavior uses the velocity
     * feedforward sign. This works well with position closed loop when
     * velocity reference is specified (motion profiling). However, when
     * using position closed loop with zero velocity reference (no motion
     * profiling), the application may want to apply static feedforward
     * based on the closed loop error sign instead. In which case, we
     * recommend the minimal amount of kS, otherwise the motor output may
     * dither when closed loop error is near zero.
     * 
     */
    signals::StaticFeedforwardSignValue StaticFeedforwardSign = signals::StaticFeedforwardSignValue::UseVelocitySign;
    
    /**
     * \brief Modifies this configuration's kP parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newKP Parameter to modify
     * \returns Itself
     */
    SlotConfigs& WithKP(double newKP)
    {
        kP = std::move(newKP);
        return *this;
    }
    /**
     * \brief Modifies this configuration's kI parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newKI Parameter to modify
     * \returns Itself
     */
    SlotConfigs& WithKI(double newKI)
    {
        kI = std::move(newKI);
        return *this;
    }
    /**
     * \brief Modifies this configuration's kD parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newKD Parameter to modify
     * \returns Itself
     */
    SlotConfigs& WithKD(double newKD)
    {
        kD = std::move(newKD);
        return *this;
    }
    /**
     * \brief Modifies this configuration's kS parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newKS Parameter to modify
     * \returns Itself
     */
    SlotConfigs& WithKS(double newKS)
    {
        kS = std::move(newKS);
        return *this;
    }
    /**
     * \brief Modifies this configuration's kV parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newKV Parameter to modify
     * \returns Itself
     */
    SlotConfigs& WithKV(double newKV)
    {
        kV = std::move(newKV);
        return *this;
    }
    /**
     * \brief Modifies this configuration's kA parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newKA Parameter to modify
     * \returns Itself
     */
    SlotConfigs& WithKA(double newKA)
    {
        kA = std::move(newKA);
        return *this;
    }
    /**
     * \brief Modifies this configuration's kG parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newKG Parameter to modify
     * \returns Itself
     */
    SlotConfigs& WithKG(double newKG)
    {
        kG = std::move(newKG);
        return *this;
    }
    /**
     * \brief Modifies this configuration's GravityType parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newGravityType Parameter to modify
     * \returns Itself
     */
    SlotConfigs& WithGravityType(signals::GravityTypeValue newGravityType)
    {
        GravityType = std::move(newGravityType);
        return *this;
    }
    /**
     * \brief Modifies this configuration's StaticFeedforwardSign parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newStaticFeedforwardSign Parameter to modify
     * \returns Itself
     */
    SlotConfigs& WithStaticFeedforwardSign(signals::StaticFeedforwardSignValue newStaticFeedforwardSign)
    {
        StaticFeedforwardSign = std::move(newStaticFeedforwardSign);
        return *this;
    }


    /**
     * \brief Chooses which slot these configs are for.
     */
    int SlotNumber = 0;

    static SlotConfigs From(const Slot0Configs& value);
    static SlotConfigs From(const Slot1Configs& value);
    static SlotConfigs From(const Slot2Configs& value);

    std::string ToString() const
    {
        std::stringstream ss;
        ss << "{" << std::endl;
        ss << "Config Group: Slot" << std::endl;
        ss << "Name: \"kP\" Value: \"" << kP << "\"" << std::endl;
        ss << "Name: \"kI\" Value: \"" << kI << "\"" << std::endl;
        ss << "Name: \"kD\" Value: \"" << kD << "\"" << std::endl;
        ss << "Name: \"kS\" Value: \"" << kS << "\"" << std::endl;
        ss << "Name: \"kV\" Value: \"" << kV << "\"" << std::endl;
        ss << "Name: \"kA\" Value: \"" << kA << "\"" << std::endl;
        ss << "Name: \"kG\" Value: \"" << kG << "\"" << std::endl;
        ss << "Name: \"GravityType\" Value: \"" << GravityType << "\"" << std::endl;
        ss << "Name: \"StaticFeedforwardSign\" Value: \"" << StaticFeedforwardSign << "\"" << std::endl;
        ss << "}" << std::endl;
        return ss.str();
    }

    std::string Serialize() const
    {
        std::stringstream ss;
        SlotSpns currentSpns = genericMap.at(SlotNumber);
        char *ref;
        c_ctre_phoenix6_serialize_double(currentSpns.kPSpn, kP, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(currentSpns.kISpn, kI, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(currentSpns.kDSpn, kD, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(currentSpns.kSSpn, kS, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(currentSpns.kVSpn, kV, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(currentSpns.kASpn, kA, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_double(currentSpns.kGSpn, kG, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_int(currentSpns.GravityTypeSpn, GravityType.value, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        c_ctre_phoenix6_serialize_int(currentSpns.StaticFeedforwardSignSpn, StaticFeedforwardSign.value, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return ss.str();
    }

    ctre::phoenix::StatusCode Deserialize(const std::string& to_deserialize)
    {
        const char *string_c_str = to_deserialize.c_str();
        size_t string_length = to_deserialize.length();
        SlotSpns currentSpns = genericMap.at(SlotNumber);
        c_ctre_phoenix6_deserialize_double(currentSpns.kPSpn, string_c_str, string_length, &kP);
        c_ctre_phoenix6_deserialize_double(currentSpns.kISpn, string_c_str, string_length, &kI);
        c_ctre_phoenix6_deserialize_double(currentSpns.kDSpn, string_c_str, string_length, &kD);
        c_ctre_phoenix6_deserialize_double(currentSpns.kSSpn, string_c_str, string_length, &kS);
        c_ctre_phoenix6_deserialize_double(currentSpns.kVSpn, string_c_str, string_length, &kV);
        c_ctre_phoenix6_deserialize_double(currentSpns.kASpn, string_c_str, string_length, &kA);
        c_ctre_phoenix6_deserialize_double(currentSpns.kGSpn, string_c_str, string_length, &kG);
        c_ctre_phoenix6_deserialize_int(currentSpns.GravityTypeSpn, string_c_str, string_length, &GravityType.value);
        c_ctre_phoenix6_deserialize_int(currentSpns.StaticFeedforwardSignSpn, string_c_str, string_length, &StaticFeedforwardSign.value);
        return 0;
    }
};


}
}
}
