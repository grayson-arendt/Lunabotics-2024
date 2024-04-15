/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
#pragma once

#include "ctre/phoenix/StatusCodes.h"
#include "ctre/phoenix6/signals/SpnEnums.hpp"
#include "ctre/phoenix6/sim/ChassisReference.hpp"
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>

namespace ctre {
namespace phoenix6 {

namespace hardware {
namespace core {
	/* forward proto */
	class CoreCANcoder;
}
}

namespace sim {

	/**
	 * \brief Class to control the state of a simulated hardware#CANcoder.
	 */
	class CANcoderSimState
	{
	private:
		int _id;

	public:
		/**
		 * \brief The orientation of the CANcoder relative to the robot chassis.
		 *
		 * This value should not be changed based on the CANcoder invert.
		 * Rather, this value should be changed when the mechanical linkage
		 * between the CANcoder and the robot changes.
		 */
		ChassisReference Orientation;

		/**
		 * \brief Creates an object to control the state of the given hardware#CANcoder.
		 *
		 * \details This constructor defaults to a counter-clockwise positive orientation
		 *          relative to the robot chassis. Note the recommended method of accessing
		 *          simulation features is to use hardware#CANcoder#GetSimState.
		 *
		 * \param device
		 *        Device to which this simulation state is attached
		 */
		CANcoderSimState(hardware::core::CoreCANcoder const &device) :
			CANcoderSimState{device, ChassisReference::CounterClockwise_Positive}
		{}
		/**
		 * \brief Creates an object to control the state of the given hardware#CANcoder.
		 *
		 * \details Note the recommended method of accessing simulation features is to
		 *          use hardware#CANcoder#GetSimState.
		 *
		 * \param device
		 *        Device to which this simulation state is attached
		 * \param orientation
		 *        Orientation of the device relative to the robot chassis
		 */
		CANcoderSimState(hardware::core::CoreCANcoder const &device, ChassisReference orientation);
		/* disallow copy, allow move */
		CANcoderSimState(CANcoderSimState const &) = delete;
		CANcoderSimState(CANcoderSimState &&) = default;
		CANcoderSimState &operator=(CANcoderSimState const &) = delete;
		CANcoderSimState &operator=(CANcoderSimState &&) = default;

		/**
		 * \brief Sets the simulated supply voltage of the CANcoder.
		 *
		 * \details The minimum allowed supply voltage is 4 V - values below this
		 * will be promoted to 4 V.
		 *
		 * \param volts
		 *        The supply voltage in Volts
		 * \returns Status code
		 */
		ctre::phoenix::StatusCode SetSupplyVoltage(units::voltage::volt_t volts);
		/**
		 * \brief Sets the simulated raw position of the CANcoder.
		 *
		 * Inputs to this function over time should be continuous, as user calls of hardware#CANcoder#SetPosition will be accounted for in the callee.
		 *
		 * \details The CANcoder integrates this to calculate the true reported position.
		 *
		 * When using the WPI Sim GUI, you will notice a readonly `position` and settable `rawPositionInput`.
		 * The readonly signal is the emulated position which will match self-test in Tuner and the hardware API.
		 * Changes to `rawPositionInput` will be integrated into the emulated position.
		 * This way a simulator can modify the position without overriding hardware API calls for home-ing the sensor.
		 *
		 * \param rotations
		 *        The raw position in rotations
		 * \returns Status code
		 */
		ctre::phoenix::StatusCode SetRawPosition(units::angle::turn_t rotations);
		/**
		 * \brief Adds to the simulated position of the CANcoder.
		 *
		 * \param dRotations
		 *        The change in position in rotations
		 * \returns Status code
		 */
		ctre::phoenix::StatusCode AddPosition(units::angle::turn_t dRotations);
		/**
		 * \brief Sets the simulated velocity of the CANcoder.
		 *
		 * \param rps
		 *        The new velocity in rotations per second
		 * \returns Status code
		 */
		ctre::phoenix::StatusCode SetVelocity(units::angular_velocity::turns_per_second_t rps);
		/**
		 * \brief Sets the simulated magnet health of the CANcoder.
		 *
		 * \param value
		 *        The magnet health to simulate. This directly correlates to the
		 *        red/green/orange state of the simulated LED.
		 * \returns Status code
		 */
		ctre::phoenix::StatusCode SetMagnetHealth(ctre::phoenix6::signals::MagnetHealthValue value);
	};

}

}
}
