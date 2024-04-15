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

#include "ctre/phoenix6/sim/CANcoderSimState.hpp"
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/dimensionless.h>
#include <units/voltage.h>

namespace ctre {
namespace phoenix6 {

namespace hardware {
namespace core {
    class CoreCANcoder;
}
}

namespace configs {

/**
 * Class for CANcoder, a CAN based magnetic encoder that provides absolute and
 * relative position along with filtered velocity.
 *
 * This handles the configurations for the hardware#CANcoder
 */
class CANcoderConfiguration : public ParentConfiguration
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
     * \brief Configs that affect the magnet sensor and how to interpret
     *        it.
     * 
     * \details Includes sensor range and other configs related to sensor.
     */
    MagnetSensorConfigs MagnetSensor;
    
    /**
     * \brief Modifies this configuration's MagnetSensor parameter and returns itself for
     *        method-chaining and easier to use config API.
     * \param newMagnetSensor Parameter to modify
     * \returns Itself
     */
    CANcoderConfiguration& WithMagnetSensor(MagnetSensorConfigs newMagnetSensor)
    {
        MagnetSensor = std::move(newMagnetSensor);
        return *this;
    }

    /**
     * \brief Get the string representation of this configuration
     */
    std::string ToString() const
    {
        std::stringstream ss;
        ss << MagnetSensor.ToString();
        return ss.str();
    }

    /**
     * \brief Get the serialized form of this configuration
     */
    std::string Serialize() const
    {
        std::stringstream ss;
        ss << MagnetSensor.Serialize();
        return ss.str();
    }

    /**
     * \brief Take a string and deserialize it to this configuration
     */
    ctre::phoenix::StatusCode Deserialize(const std::string& to_deserialize)
    {
        ctre::phoenix::StatusCode err = ctre::phoenix::StatusCode::OK;
        err = MagnetSensor.Deserialize(to_deserialize);
        return err;
    }
};

/**
 * Class for CANcoder, a CAN based magnetic encoder that provides absolute and
 * relative position along with filtered velocity.
 *
 * This handles the configurations for the hardware#CANcoder
 */
class CANcoderConfigurator : public ParentConfigurator
{
    CANcoderConfigurator (hardware::DeviceIdentifier id):
        ParentConfigurator{std::move(id)}
    {}

    friend hardware::core::CoreCANcoder;
public:

    /**
     * Delete the copy constructor, we can only pass by reference
     */
    CANcoderConfigurator(const CANcoderConfigurator&) = delete;

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
    ctre::phoenix::StatusCode Refresh(CANcoderConfiguration& configs) const
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
    ctre::phoenix::StatusCode Refresh(CANcoderConfiguration& configs, units::time::second_t timeoutSeconds) const
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
    ctre::phoenix::StatusCode Apply(const CANcoderConfiguration& configs)
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
    ctre::phoenix::StatusCode Apply(const CANcoderConfiguration& configs, units::time::second_t timeoutSeconds)
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
    ctre::phoenix::StatusCode Refresh(MagnetSensorConfigs& configs) const
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
    ctre::phoenix::StatusCode Refresh(MagnetSensorConfigs& configs, units::time::second_t timeoutSeconds) const
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
    ctre::phoenix::StatusCode Apply(const MagnetSensorConfigs& configs)
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
    ctre::phoenix::StatusCode Apply(const MagnetSensorConfigs& configs, units::time::second_t timeoutSeconds)
    {
        return SetConfigsPrivate(configs.Serialize(), timeoutSeconds, false, false);
    }

    
    /**
     * \brief Sets the current position of the device.
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
     * \brief Sets the current position of the device.
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
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::CANCoder_SetSensorPosition, newValue.to<double>(), &ref); if (ref != nullptr) { ss << ref; free(ref); }
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
     * \brief Clear sticky fault: The magnet distance is not correct or
     * magnet is missing
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
    ctre::phoenix::StatusCode ClearStickyFault_BadMagnet()
    {
        return ClearStickyFault_BadMagnet(DefaultTimeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: The magnet distance is not correct or
     * magnet is missing
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
    ctre::phoenix::StatusCode ClearStickyFault_BadMagnet(units::time::second_t timeoutSeconds)
    {
        std::stringstream ss;
        char *ref;
        c_ctre_phoenix6_serialize_double(ctre::phoenix6::spns::SpnValue::ClearStickyFault_CANCODER_BadMagnet, 0, &ref); if (ref != nullptr) { ss << ref; free(ref); }
        return SetConfigsPrivate(ss.str(), timeoutSeconds, false, true);
    }
};

}

namespace hardware {
namespace core {

/**
 * Class for CANcoder, a CAN based magnetic encoder that provides absolute and
 * relative position along with filtered velocity.
 */
class CoreCANcoder : public ParentDevice
{
private:
    configs::CANcoderConfigurator _configs;

    
public:
    /**
     * Constructs a new CANcoder object.
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
    CoreCANcoder(int deviceId, std::string canbus = "");

    CoreCANcoder(CoreCANcoder const &) = delete;
    CoreCANcoder &operator=(CoreCANcoder const &) = delete;

    /**
     * \brief Gets the configurator for this CANcoder
     *
     * \details Gets the configurator for this CANcoder
     *
     * \returns Configurator for this CANcoder
     */
    configs::CANcoderConfigurator &GetConfigurator()
    {
        return _configs;
    }

    /**
     * \brief Gets the configurator for this CANcoder
     *
     * \details Gets the configurator for this CANcoder
     *
     * \returns Configurator for this CANcoder
     */
    configs::CANcoderConfigurator const &GetConfigurator() const
    {
        return _configs;
    }


private:
    std::unique_ptr<sim::CANcoderSimState> _simState{};
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
    sim::CANcoderSimState &GetSimState()
    {
        if (_simState == nullptr)
            _simState = std::make_unique<sim::CANcoderSimState>(*this);
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
     * \brief Velocity of the device.
     * 
     *   Minimum Value: -512.0
     *   Maximum Value: 511.998046875
     *   Default Value: 0
     *   Units: rotations per second
     * 
     * Default Rates:
     *   CAN 2.0: 100.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns Velocity Status Signal Object
     */
    StatusSignal<units::angular_velocity::turns_per_second_t> &GetVelocity();
        
    /**
     * \brief Position of the device. This is initialized to the absolute
     * position on boot.
     * 
     *   Minimum Value: -16384.0
     *   Maximum Value: 16383.999755859375
     *   Default Value: 0
     *   Units: rotations
     * 
     * Default Rates:
     *   CAN 2.0: 100.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns Position Status Signal Object
     */
    StatusSignal<units::angle::turn_t> &GetPosition();
        
    /**
     * \brief Absolute Position of the device. The possible range is
     * documented below; however, the exact expected range is determined
     * by the AbsoluteSensorRange. This position is only affected by the
     * MagnetSensor configs.
     * 
     *   Minimum Value: -0.5
     *   Maximum Value: 0.999755859375
     *   Default Value: 0
     *   Units: rotations
     * 
     * Default Rates:
     *   CAN 2.0: 100.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns AbsolutePosition Status Signal Object
     */
    StatusSignal<units::angle::turn_t> &GetAbsolutePosition();
        
    /**
     * \brief The unfiltered velocity reported by CANcoder.
     * 
     * \details This is the unfiltered velocity reported by CANcoder. This
     * signal does not use the fusing algorithm.
     * 
     *   Minimum Value: -8000.0
     *   Maximum Value: 7999.755859375
     *   Default Value: 0
     *   Units: rotations per second
     * 
     * Default Rates:
     *   CAN 2.0: 4.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns UnfilteredVelocity Status Signal Object
     */
    StatusSignal<units::angular_velocity::turns_per_second_t> &GetUnfilteredVelocity();
        
    /**
     * \brief The relative position reported by the CANcoder since boot.
     * 
     * \details This is the total displacement reported by CANcoder since
     * power up. This signal is relative and is not influenced by the
     * fusing algorithm.
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
     * \returns PositionSinceBoot Status Signal Object
     */
    StatusSignal<units::angle::turn_t> &GetPositionSinceBoot();
        
    /**
     * \brief Measured supply voltage to the CANcoder.
     * 
     *   Minimum Value: 4
     *   Maximum Value: 16.75
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
     * \brief Magnet health as measured by CANcoder.
     * 
     * \details Magnet health as measured by CANcoder. Red indicates too
     * close or too far, Orange is adequate but with reduced accuracy,
     * green is ideal. Invalid means the accuracy cannot be determined.
     * 
     * 
     * Default Rates:
     *   CAN 2.0: 4.0 Hz
     *   CAN FD: 100.0 Hz (TimeSynced with Pro)
     * 
     * \returns MagnetHealth Status Signal Object
     */
    StatusSignal<signals::MagnetHealthValue> &GetMagnetHealth();
        
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
     * \brief The magnet distance is not correct or magnet is missing
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns Fault_BadMagnet Status Signal Object
     */
    StatusSignal<bool> &GetFault_BadMagnet();
        
    /**
     * \brief The magnet distance is not correct or magnet is missing
     * 
     *   Default Value: False
     * 
     * Default Rates:
     *   CAN: 4.0 Hz
     * 
     * \returns StickyFault_BadMagnet Status Signal Object
     */
    StatusSignal<bool> &GetStickyFault_BadMagnet();

    

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
     * \brief Sets the current position of the device.
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
     * \brief Sets the current position of the device.
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
     * \brief Clear sticky fault: The magnet distance is not correct or
     * magnet is missing
     * 
     * \param timeoutSeconds Maximum time to wait up to in seconds.
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_BadMagnet(units::time::second_t timeoutSeconds)
    {
        return GetConfigurator().ClearStickyFault_BadMagnet(timeoutSeconds);
    }
    /**
     * \brief Clear sticky fault: The magnet distance is not correct or
     * magnet is missing
     * 
     * This will wait up to 0.050 seconds (50ms) by default.
     * 
     * \returns StatusCode of the set command
     */
    ctre::phoenix::StatusCode ClearStickyFault_BadMagnet()
    {
        return ClearStickyFault_BadMagnet(0.050_s);
    }
};

}
}

}
}

