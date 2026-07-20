#pragma once
#include <string>
#include <iostream>
#include "Constants.hpp"
#include "SensorObject.hpp"
#include <string_view>

struct GpsObject : public SensorObject {
    // מיקום גיאומטרי (מחושב מתוך BasicLocationData)
    double x_local;       // מיקום X במטרים (StateX)
    double y_local;       // מיקום Y במטרים (StateY)
    double z_local;       // גובה (StateZ)
    double speed;         // מהירות קווית (StateVx)
    double heading;       // כיוון במעלות/רדיאנים (StateYaw)

    GpsObject()
    : SensorObject(Config::DimWheelchairStateVector, Config::MeasurementSizeGps, SensorType::Gps,false,false,false), // <-- הפסיק התווסף כאן בסוף השורה
      x_local(0.0),
      y_local(0.0),
      z_local(0.0),
      speed(0.0),
      heading(0.0)
    {}
};