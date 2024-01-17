#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#define Phoenix_No_WPI
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.hpp"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

std::string interface = "can0";
TalonFX left_wheel_motor(2, interface);
//TalonFX right_wheel_motor(3);

/*
Modified by: Grayson Arendt

Sample code for PID adjusted for ROS2 usage.
This uses a DualShock4 controller
*/

class MotorPID : public rclcpp::Node
{
public:
    MotorPID() : Node("motor_pid")
    {        
        left_wheel_motor.ConfigFactoryDefault();

        left_wheel_motor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);

        /* Set relevant frame periods to be at least as fast as periodic rate */
        left_wheel_motor.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
        left_wheel_motor.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);

        /* Set the peak and nominal outputs */
        left_wheel_motor.ConfigNominalOutputForward(0, 10);
        left_wheel_motor.ConfigNominalOutputReverse(0, 10);
        left_wheel_motor.ConfigPeakOutputForward(1, 10);
        left_wheel_motor.ConfigPeakOutputReverse(-1, 10);

        /* Set Motion Magic gains in slot0 - see documentation */
        left_wheel_motor.SelectProfileSlot(0, 0);
        left_wheel_motor.Config_kF(0, 0.1097, 10);
        left_wheel_motor.Config_kP(0, 0.8, 10);
        left_wheel_motor.Config_kI(0, 0.0, 10);
        left_wheel_motor.Config_kD(0, 0.0, 10);

        /* Set acceleration and vcruise velocity - see documentation */
        left_wheel_motor.ConfigMotionCruiseVelocity(10000, 10);
        left_wheel_motor.ConfigMotionAcceleration(10000, 10);

        /* Zero the sensor */
        left_wheel_motor.SetSelectedSensorPosition(0, 0, 10);

        controller_subscriber = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10,
        std::bind(&MotorPID::callbackMotors, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), " Motor control started. ");
    }


private:

    void callbackMotors(const sensor_msgs::msg::Joy::SharedPtr controller_input)
    {   

        c_FeedEnable(5000);

        /* Get gamepad axis - forward stick is positive */
        double leftYstick = controller_input->axes[1];
        double RightYstick = controller_input->axes[3];

        //RCLCPP_INFO(this->get_logger(), "Left stick: %f Right stick: %f", leftYstick, RightYstick);
        
        if (fabs(leftYstick) < 0.10) { leftYstick = 0;} /* deadband 10% */
        if (fabs(RightYstick) < 0.10) { RightYstick = 0;} /* deadband 10% */

        RCLCPP_INFO(this->get_logger(), "Velocity: %f", left_wheel_motor.GetSelectedSensorVelocity(0));
      
        // press circle button[1]
        if(controller_input->buttons[1])
        {
            /* Zero the sensor */
            left_wheel_motor.SetSelectedSensorPosition(0, 0, 10);
        }

        /**
         * Perform Motion Magic when Button 1 is held,
         * else run Percent Output, which can be used to confirm hardware setup.
         */

        // press square buttons[3]
        if (controller_input->buttons[3]) {
            /* Motion Magic */ 
        
            /*2048 ticks/rev * 10 Rotations in either direction */
            double targetPos = RightYstick * 2048 * 10.0;
            left_wheel_motor.Set(ControlMode::MotionMagic, targetPos);

            /* Append more signals to print when in speed mode */
            //sb << "\terr:" << left_wheel_motor.GetClosedLoopError(0);
            //sb << "\ttrg:" << targetPos;
        } 

        // press triangle buttons[2]
        else if (controller_input->buttons[2])
        {
            /* Increase smoothing */
            ++_smoothing;
            if(_smoothing > 8) _smoothing = 8;
            RCLCPP_INFO(this->get_logger(), "Smoothing: %d", _smoothing);
            left_wheel_motor.ConfigMotionSCurveStrength(_smoothing, 0);
        }

        // press x buttons[0] 
        else if (controller_input->buttons[0])
        {
            /* Decreasing smoothing */
            --_smoothing;
            if(_smoothing < 0) _smoothing = 0;
            RCLCPP_INFO(this->get_logger(), "Smoothing: %d", _smoothing);
            left_wheel_motor.ConfigMotionSCurveStrength(_smoothing, 0);
        }

        else {
            /* Percent Output */
            RCLCPP_INFO(this->get_logger(), "Left stick: %f Right stick: %f", leftYstick, RightYstick);
        
            left_wheel_motor.Set(ControlMode::PercentOutput, leftYstick);

            double motorOutput = left_wheel_motor.GetMotorOutputPercent(); 
            RCLCPP_INFO(this->get_logger(), "Percent Output: %f", motorOutput);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr controller_subscriber;
    int _smoothing;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorPID>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}