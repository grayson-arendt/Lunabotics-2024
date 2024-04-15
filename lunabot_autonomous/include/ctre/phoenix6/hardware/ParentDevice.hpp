/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
#pragma once

#include "ctre/phoenix6/configs/Configs.hpp"
#include "ctre/phoenix6/configs/Configurator.hpp"
#include "ctre/phoenix6/controls/ControlRequest.hpp"
#include "ctre/phoenix6/hardware/DeviceIdentifier.hpp"
#include "ctre/phoenix6/signals/SpnEnums.hpp"
#include "ctre/phoenix6/spns/SpnValue.hpp"
#include "ctre/phoenix6/StatusSignal.hpp"
#include <map>
#include <memory>
#include <mutex>
#include <units/dimensionless.h>

namespace ctre {
namespace phoenix6 {
namespace hardware {

    /**
     * Parent class for all devices
     */
    class ParentDevice
    {
    protected:
        static inline controls::EmptyControl _emptyControl{};

        DeviceIdentifier deviceIdentifier;

        /**
         * \brief Type trait to verify that all types passed in are subclasses of ParentDevice.
         */
        template <typename... Devices>
        struct is_all_device :
            std::conjunction<std::is_base_of<ParentDevice, std::remove_reference_t<Devices>>...>
        {};

        /**
         * \brief Whether all types passed in are subclasses of ParentDevice.
         */
        template <typename... Devices>
        static constexpr bool is_all_device_v = is_all_device<Devices...>::value;

    private:
        std::map<uint32_t, std::unique_ptr<BaseStatusSignal>> _signalValues;
        std::recursive_mutex _signalValuesLck;
        /**
         * Use a shared pointer so users that access the control request via #GetAppliedControl has a copy
         * of the pointer without risk of it becoming a dangling pointer due to parallel operations
         */
        std::shared_ptr<controls::ControlRequest> _controlReq = std::make_shared<controls::EmptyControl>();
        std::mutex _controlReqLck;

        double _creationTime = GetCurrentTimeSeconds();

        bool _isInitialized = false;
        ctre::phoenix::StatusCode _versionStatus{ctre::phoenix::StatusCode::CouldNotRetrieveV6Firmware};
        double _timeToRefreshVersion = GetCurrentTimeSeconds();

        StatusSignal<int> &_compliancy{LookupStatusSignal<int>(ctre::phoenix6::spns::SpnValue::Compliancy_Version, "Compliancy", false)};
        StatusSignal<int> &_resetSignal{LookupStatusSignal<int>(ctre::phoenix6::spns::SpnValue::Startup_ResetFlags, "ResetFlags", false)};

        void ReportIfTooOld();

        template <typename T>
        StatusSignal<T> &LookupCommon(uint16_t spn, uint16_t mapper_iter, std::function<std::map<int, StatusSignal<T>>()> map_filler, std::string signalName, bool reportOnConstruction)
        {
            BaseStatusSignal *toFind;
            {
                /* lock access to the map */
                std::lock_guard<std::recursive_mutex> lock{_signalValuesLck};

                const uint32_t totalHash = spn | ((uint32_t)mapper_iter << 16);
                /* lookup and return if found */
                auto iter = _signalValues.find(totalHash);
                if (iter != _signalValues.end())
                {
                    /* Found it, toFind is now the found StatusSignal */
                    toFind = iter->second.get();
                    /* since we didn't construct, report errors */
                    reportOnConstruction = true;
                }
                else
                {
                    /* insert into map */
                    /* Mapper_iter is 0 when using straight SPNs, otherwise it's nonzero for switchable SPNs */
                    if (mapper_iter != 0)
                    {
                        /* Switchable spn, so generate the map to switch with */
                        _signalValues.emplace(totalHash, std::unique_ptr<StatusSignal<T>>{new StatusSignal<T>{
                                                            deviceIdentifier, spn, [this]()
                                                            { ReportIfTooOld(); },
                                                            map_filler, std::move(signalName)}});
                    }
                    else
                    {
                        /* Non-switchable spn, so just add the SPN plain */
                        _signalValues.emplace(totalHash, std::unique_ptr<StatusSignal<T>>{new StatusSignal<T>{
                                                            deviceIdentifier, spn, [this]()
                                                            { ReportIfTooOld(); },
                                                            std::move(signalName)}});
                    }

                    /* look up and return */
                    iter = _signalValues.find(totalHash);
                    toFind = iter->second.get();
                }
            }

            /* Now cast it up to the StatusSignal */
            StatusSignal<T> *ret = dynamic_cast<StatusSignal<T> *>(toFind);
            /* If ret is null, that means the cast failed. Otherwise we can return it */
            if (ret == nullptr)
            {
                /* Cast failed, let user know this doesn't exist */
                static StatusSignal<T> failure{ctre::phoenix::StatusCode::InvalidParamValue};
                return failure;
            }
            else
            {
                /* Good cast, refresh it and return this now */
                ret->Refresh(reportOnConstruction);
                return *ret;
            }
        }

    public:
        ParentDevice(int deviceID, std::string model, std::string canbus) :
            deviceIdentifier{DeviceIdentifier{deviceID, std::move(model), std::move(canbus)}}
        {
            /* This needs to be set to true after everything else has already been initialized */
            _isInitialized = true;
        }

        virtual ~ParentDevice() = default;

        /**
         * \returns The device ID of this device [0,62].
         */
        int GetDeviceID() const
        {
            return deviceIdentifier.deviceID;
        }

        /**
         * \returns Name of the network this device is on.
         */
        const std::string &GetNetwork() const
        {
            return deviceIdentifier.network;
        }

        /**
         * \brief Gets a number unique for this device's hardware type and ID.
         * This number is not unique across networks.
         *
         * \details This can be used to easily reference hardware devices on
         * the same network in collections such as maps.
         *
         * \returns Hash of this device.
         */
        uint64_t GetDeviceHash() const
        {
            return deviceIdentifier.deviceHash;
        }

        /**
         * \brief Get the latest applied control.
         *        Caller can cast this to the derived class if they know its type. Otherwise,
         *        use controls#ControlRequest#GetControlInfo to get info out of it.
         *
         * \details This returns a shared pointer to avoid becoming a dangling pointer
         *          due to parallel operations changing the underlying data. Make sure
         *          to save the shared_ptr to a variable before chaining function calls,
         *          otherwise the data may be freed early.
         *
         * \returns Latest applied control
         */
        std::shared_ptr<const controls::ControlRequest> GetAppliedControl() const
        {
            return _controlReq;
        }

        /**
         * \brief Get the latest applied control.
         *        Caller can cast this to the derived class if they know its type. Otherwise,
         *        use controls#ControlRequest#GetControlInfo to get info out of it.
         *
         * \details This returns a shared pointer to avoid becoming a dangling pointer
         *          due to parallel operations changing the underlying data. Make sure
         *          to save the shared_ptr to a variable before chaining function calls,
         *          otherwise the data may be freed early.
         *
         * \returns Latest applied control
         */
        std::shared_ptr<controls::ControlRequest> GetAppliedControl()
        {
            return _controlReq;
        }

        /**
         * \returns true if device has reset since the previous call of this routine.
         */
        bool HasResetOccurred()
        {
            return _resetSignal.Refresh(false).HasUpdated();
        }

        /**
         * \returns a function that checks for device resets.
         */
        std::function<bool()> GetResetOccurredChecker() const
        {
            return [resetSignal=_resetSignal]() mutable {
                return resetSignal.Refresh(false).HasUpdated();
            };
        }

        /**
         * \brief This is a reserved routine for internal testing.  Use the other get routines to retrieve signal values.
         *
         * \param signal Signal to get.
         * \return StatusSignalValue holding value
         */
        StatusSignal<double> & GetGenericSignal(uint32_t signal)
        {
            return LookupStatusSignal<double>((uint16_t)signal, "Generic", true);
        }

        /**
         * \brief Optimizes the device's bus utilization by reducing the update frequencies of its status signals.
         *
         * All status signals that have not been explicitly given an update frequency using
         * BaseStatusSignal#SetUpdateFrequency will be disabled. Note that if other status
         * signals in the same status frame have been given an update frequency, the update
         * frequency will be honored for the entire frame.
         *
         * This function only needs to be called once on this device in the robot program. Additionally, this
         * method does not necessarily need to be called after setting the update frequencies of other signals.
         *
         * To restore the default status update frequencies, remove this method call, redeploy the robot
         * application, and power-cycle the devices on the bus. Alternatively, the user can override
         * individual status update frequencies using BaseStatusSignal#SetUpdateFrequency.
         *
         * \param timeoutSeconds Maximum amount of time to wait for each status frame when performing the action
         * \return Status code of the first failed update frequency set call, or OK if all succeeded
         */
        ctre::phoenix::StatusCode OptimizeBusUtilization(units::time::second_t timeoutSeconds = 50_ms);

        /**
         * \brief Optimizes the bus utilization of the provided devices by reducing the update
         * frequencies of their status signals.
         *
         * All status signals that have not been explicitly given an update frequency using
         * BaseStatusSignal#SetUpdateFrequency will be disabled. Note that if other status
         * signals in the same status frame have been given an update frequency, the update
         * frequency will be honored for the entire frame.
         *
         * This function only needs to be called once in the robot program for the provided devices.
         * Additionally, this method does not necessarily need to be called after setting the update
         * frequencies of other signals.
         *
         * To restore the default status update frequencies, remove this method call, redeploy the robot
         * application, and power-cycle the devices on the bus. Alternatively, the user can override
         * individual status update frequencies using BaseStatusSignal#SetUpdateFrequency.
         *
         * This will wait up to 0.050 seconds (50ms) for each status frame.
         *
         * \param devices Devices for which to optimize bus utilization, passed as a comma-separated list of device references.
         * \return Status code of the first failed optimize call, or OK if all succeeded
         */
        template <typename... Devices, typename = std::enable_if_t<is_all_device_v<Devices...>>>
        static ctre::phoenix::StatusCode OptimizeBusUtilizationForAll(Devices &... devices)
        {
            return OptimizeBusUtilizationForAll(std::array<ParentDevice *, sizeof...(Devices)>{(&devices)...});
        }

        /**
         * \brief Optimizes the bus utilization of the provided devices by reducing the update
         * frequencies of their status signals.
         *
         * All status signals that have not been explicitly given an update frequency using
         * BaseStatusSignal#SetUpdateFrequency will be disabled. Note that if other status
         * signals in the same status frame have been given an update frequency, the update
         * frequency will be honored for the entire frame.
         *
         * This function only needs to be called once in the robot program for the provided devices.
         * Additionally, this method does not necessarily need to be called after setting the update
         * frequencies of other signals.
         *
         * To restore the default status update frequencies, remove this method call, redeploy the robot
         * application, and power-cycle the devices on the bus. Alternatively, the user can override
         * individual status update frequencies using BaseStatusSignal#SetUpdateFrequency.
         *
         * This will wait up to 0.050 seconds (50ms) for each status frame.
         *
         * \param devices Devices for which to optimize bus utilization, passed as a vector or initializer list of device addresses.
         * \return Status code of the first failed optimize call, or OK if all succeeded
         */
        static ctre::phoenix::StatusCode OptimizeBusUtilizationForAll(const std::vector<ParentDevice *> &devices)
        {
            ctre::phoenix::StatusCode retval = ctre::phoenix::StatusCode::OK;
            for (auto device : devices) {
                const auto err = device->OptimizeBusUtilization();
                if (retval.IsOK()) {
                    retval = err;
                }
            }
            return retval;
        }

        /**
         * \brief Optimizes the bus utilization of the provided devices by reducing the update
         * frequencies of their status signals.
         *
         * All status signals that have not been explicitly given an update frequency using
         * BaseStatusSignal#SetUpdateFrequency will be disabled. Note that if other status
         * signals in the same status frame have been given an update frequency, the update
         * frequency will be honored for the entire frame.
         *
         * This function only needs to be called once in the robot program for the provided devices.
         * Additionally, this method does not necessarily need to be called after setting the update
         * frequencies of other signals.
         *
         * To restore the default status update frequencies, remove this method call, redeploy the robot
         * application, and power-cycle the devices on the bus. Alternatively, the user can override
         * individual status update frequencies using BaseStatusSignal#SetUpdateFrequency.
         *
         * This will wait up to 0.050 seconds (50ms) for each status frame.
         *
         * \param devices Devices for which to optimize bus utilization, passed as an array of device addresses.
         * \return Status code of the first failed optimize call, or OK if all succeeded
         */
        template <size_t N>
        static ctre::phoenix::StatusCode OptimizeBusUtilizationForAll(const std::array<ParentDevice *, N> &devices)
        {
            ctre::phoenix::StatusCode retval = ctre::phoenix::StatusCode::OK;
            for (auto device : devices) {
                const auto err = device->OptimizeBusUtilization();
                if (retval.IsOK()) {
                    retval = err;
                }
            }
            return retval;
        }

    protected:
        virtual ctre::phoenix::StatusCode SetControlPrivate(controls::ControlRequest &request);

        template <typename T>
        StatusSignal<T> &LookupStatusSignal(uint16_t spn, std::string signalName, bool reportOnConstruction)
        {
            std::function<std::map<int, StatusSignal<T>>()> emptyMapFiller = []
            {
                return std::map<int, StatusSignal<T>>{};
            };
            return LookupCommon<T>(spn, 0, emptyMapFiller, std::move(signalName), reportOnConstruction);
        }

        template <typename T>
        StatusSignal<T> &LookupStatusSignal(uint16_t spn, uint16_t mapper_iter, std::function<std::map<int, StatusSignal<T>>()> map_filler, std::string signalName, bool reportOnConstruction)
        {
            return LookupCommon<T>(spn, mapper_iter, map_filler, std::move(signalName), reportOnConstruction);
        }

        /** Returns a unitless version of the StatusSignal by value. Do not store the result in a reference. */
        template <typename T, typename U>
        StatusSignal<T> LookupDimensionlessStatusSignal(uint16_t spn, std::string signalName)
        {
            return StatusSignal<T>{LookupStatusSignal<U>(spn, std::move(signalName), true)};
        }
    };

}
}
}
