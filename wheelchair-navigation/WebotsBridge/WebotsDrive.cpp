#include "WebotsDrive.hpp"

#include <webots/motor.h>

#include <algorithm>
#include <cmath>
#include <iostream>

WebotsDrive& WebotsDrive::getInstance()
{
    static WebotsDrive instance;
    return instance;
}

void WebotsDrive::init(WbDeviceTag leftMotor,
                       WbDeviceTag rightMotor,
                       double wheelRadius,
                       double wheelBase,
                       double maxWheelVelocity)
{
    leftMotor_ = leftMotor;
    rightMotor_ = rightMotor;
    wheelRadius_ = wheelRadius;
    wheelBase_ = wheelBase;
    maxWheelVelocity_ = maxWheelVelocity;

    if (leftMotor_ == 0 || rightMotor_ == 0) {
        std::cerr << "[WebotsDrive] Cannot init: motor tag is 0\n";
        return;
    }

    // מצב velocity control
    wb_motor_set_position(leftMotor_, INFINITY);
    wb_motor_set_position(rightMotor_, INFINITY);

    wb_motor_set_velocity(leftMotor_, 0.0);
    wb_motor_set_velocity(rightMotor_, 0.0);

    std::cout << "[WebotsDrive] Initialized with C API\n";
}

void WebotsDrive::send(float linearSpeed, float angularSpeed)
{
    if (leftMotor_ == 0 || rightMotor_ == 0) {
        return;
    }

    double leftVelocity =
        (static_cast<double>(linearSpeed) -
         static_cast<double>(angularSpeed) * wheelBase_ / 2.0) / wheelRadius_;

    double rightVelocity =
        (static_cast<double>(linearSpeed) +
         static_cast<double>(angularSpeed) * wheelBase_ / 2.0) / wheelRadius_;

    leftVelocity = std::clamp(leftVelocity, -maxWheelVelocity_, maxWheelVelocity_);
    rightVelocity = std::clamp(rightVelocity, -maxWheelVelocity_, maxWheelVelocity_);

    std::cout << "[WebotsDrive] send() -> linear=" << linearSpeed << " angular=" << angularSpeed
              << " => leftVel=" << leftVelocity << " rightVel=" << rightVelocity << std::endl;

    wb_motor_set_velocity(leftMotor_, leftVelocity);
    wb_motor_set_velocity(rightMotor_, rightVelocity);
}

void WebotsDrive::stop()
{
    if (leftMotor_ != 0) {
        wb_motor_set_velocity(leftMotor_, 0.0);
    }

    if (rightMotor_ != 0) {
        wb_motor_set_velocity(rightMotor_, 0.0);
    }
}