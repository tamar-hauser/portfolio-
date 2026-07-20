#pragma once

#include <webots/types.h>

class WebotsDrive {
public:
    static WebotsDrive& getInstance();

    void init(WbDeviceTag leftMotor,
              WbDeviceTag rightMotor,
              double wheelRadius,
              double wheelBase,
              double maxWheelVelocity);

    void send(float linearSpeed, float angularSpeed);
    void stop();

private:
    WebotsDrive() = default;

    WbDeviceTag leftMotor_ = 0;
    WbDeviceTag rightMotor_ = 0;

    double wheelRadius_ = 0.22;
    double wheelBase_ = 0.86;
    double maxWheelVelocity_ = 12.0;
};