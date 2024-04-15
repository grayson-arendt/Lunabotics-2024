/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
#pragma once

#include "ctre/phoenix/StatusCodes.h"
#include "ctre/phoenix6/sim/ChassisReference.hpp"
#include <units/angle.h>
#include <units/voltage.h>

namespace ctre {
namespace phoenix6 {

namespace hardware {
namespace core {
	/* forward proto */
	class CorePigeon2;
}
}

namespace sim {

	/**
	 * \brief Class to control the state of a simulated hardware#Pigeon2.
	 */
	class Pigeon2SimState
	{
	private:
		int _id;

	public:
		/**
		 * \brief Creates an object to control the state of the given hardware#Pigeon2.
		 *
		 * \details Note the recommended method of accessing simulation features is to
		 *          use hardware#Pigeon2#GetSimState.
		 *
		 * \param device
		 *        Device to which this simulation state is attached
		 */
		Pigeon2SimState(hardware::core::CorePigeon2 const &device);
		/* disallow copy, allow move */
		Pigeon2SimState(Pigeon2SimState const &) = delete;
		Pigeon2SimState(Pigeon2SimState &&) = default;
		Pigeon2SimState &operator=(Pigeon2SimState const &) = delete;
		Pigeon2SimState &operator=(Pigeon2SimState &&) = default;

		/**
		 * \brief Sets the simulated supply voltage of the Pigeon2.
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
		 * \brief Sets the simulated raw yaw of the Pigeon2.
		 *
		 * Inputs to this function over time should be continuous, as user calls of hardware#Pigeon2#SetYaw will be accounted for in the callee.
		 *
		 * \details The Pigeon2 integrates this to calculate the true reported yaw.
		 *
		 * When using the WPI Sim GUI, you will notice a readonly `yaw` and settable `rawYawInput`.
		 * The readonly signal is the emulated yaw which will match self-test in Tuner and the hardware API.
		 * Changes to `rawYawInput` will be integrated into the emulated yaw.
		 * This way a simulator can modify the yaw without overriding hardware API calls for home-ing the sensor.
		 *
		 * \param deg
		 *        The yaw in degrees
		 * \returns Status code
		 */
		ctre::phoenix::StatusCode SetRawYaw(units::angle::degree_t deg);
		/**
		 * \brief Adds to the simulated yaw of the Pigeon2.
		 *
		 * \param dDeg
		 *        The change in yaw in degrees
		 * \returns Status code
		 */
		ctre::phoenix::StatusCode AddYaw(units::angle::degree_t dDeg);
		/**
		 * \brief Sets the simulated pitch of the Pigeon2.
		 *
		 * \param deg
		 *        The pitch in degrees
		 * \returns Status code
		 */
		ctre::phoenix::StatusCode SetPitch(units::angle::degree_t deg);
		/**
		 * \brief Sets the simulated roll of the Pigeon2.
		 *
		 * \param deg
		 *        The roll in degrees
		 * \returns Status code
		 */
		ctre::phoenix::StatusCode SetRoll(units::angle::degree_t deg);
	};

}

}
}
