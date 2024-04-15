/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
#pragma once

namespace ctre {
namespace phoenix6 {

/**
 * \brief Possible states of a mechanism.
 */
enum class MechanismState {
    /**
     * \brief The mechanism is running normally.
     */
    OK,
    /**
     * \brief The mechanism is temporarily disabled due to an issue.
     */
    Disabled,
    /**
     * \brief The mechanism is disabled and requires user action.
     */
    RequiresUserAction,
};

}
}
