/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
#pragma once

namespace ctre {
namespace phoenix6 {
namespace sim {

	/**
	 * Represents the orientation of a device relative to the robot chassis.
	 */
	enum class ChassisReference {
		/** The device should read a counter-clockwise rotation as positive motion. */
		CounterClockwise_Positive = 1,
		/** The device should read a clockwise rotation as positive motion. */
		Clockwise_Positive = -1
	};

}
}
}
