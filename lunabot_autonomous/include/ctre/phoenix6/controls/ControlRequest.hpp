/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
#pragma once

#include "ctre/phoenix6/networking/interfaces/Control_Interface.h"
#include <map>
#include <memory>
#include <sstream>
#include <string>

namespace ctre {
namespace phoenix6 {

namespace hardware {
    class ParentDevice;
}

namespace controls {

    /**
     * \brief Abstract Control Request class that other control requests extend for use.
     */
    class ControlRequest
    {
        friend hardware::ParentDevice;
        virtual ctre::phoenix::StatusCode SendRequest(const char *network, uint32_t deviceHash, bool cancelOtherRequests, std::shared_ptr<ControlRequest> &req) = 0;

    protected:
        std::string name;

        /* don't allow copy and move of the base class, only derived classes */
        ControlRequest(ControlRequest const &) = default;
        ControlRequest(ControlRequest &&) = default;
        ControlRequest &operator=(ControlRequest const &) = default;
        ControlRequest &operator=(ControlRequest &&) = default;

    public:
        /**
         * \brief Constructs a new Control Request with the given name.
         *
         * \param name Name of the control request
         */
        ControlRequest(std::string name) : name{std::move(name)}
        {
        }

        std::string const &GetName() const
        {
            return name;
        }

        /**
         * \brief Gets information about this control request.
         *
         * \returns Map of control parameter names and corresponding applied values
         */
        virtual std::map<std::string, std::string> GetControlInfo() const = 0;

        virtual ~ControlRequest() = default;
        virtual std::string ToString() const = 0;
    };

    /**
     * \brief Generic Empty Control class used to do nothing.
     */
    class EmptyControl : public ControlRequest
    {
        ctre::phoenix::StatusCode SendRequest(const char *network, uint32_t deviceHash, bool cancelOtherRequests, std::shared_ptr<ControlRequest> &req) override
        {
            if (req.get() != this)
            {
                auto const reqCast = dynamic_cast<EmptyControl *>(req.get());
                if (reqCast != nullptr)
                {
                    *reqCast = *this;
                }
                else
                {
                    req = std::make_shared<EmptyControl>(*this);
                }
            }

            return c_ctre_phoenix6_RequestControlEmpty(network, deviceHash, 0, cancelOtherRequests);
        }

    public:
        /**
         * \brief Constructs an empty control request.
         */
        EmptyControl() : ControlRequest{"EmptyControl"}
        {
        }

        /**
         * \brief Gets information about this control request.
         *
         * \returns Map of control parameter names and corresponding applied values
         */
        std::map<std::string, std::string> GetControlInfo() const override
        {
            std::map<std::string, std::string> controlInfo;
            controlInfo["Name"] = GetName();
            return controlInfo;
        }

        std::string ToString() const override
        {
            std::stringstream ss;
            ss << "class: EmptyControl" << std::endl;
            return ss.str();
        }
    };

}

}
}
