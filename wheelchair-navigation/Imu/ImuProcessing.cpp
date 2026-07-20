#include "ImuProcessing.hpp"
#include "ImuData.hpp"
#include "ImuObject.hpp"
#include <cmath>
#include <algorithm> // נוסף כדי לפתור את בעיית std::clamp

ImuObject ImuProcessing::process(ImuMeasurement& input, double tss) {
    this->ts = tss;
    

    return createObject(input);
}

ImuObject ImuProcessing::createObject(ImuMeasurement& raw) {
    ImuObject IO;
    IO.timestamp = this->ts;
    
    // תיקון הגישה למשתנים מתוך ה-struct:
    IO.Ax    = raw.raw_accel_x / 1000.0;
    IO.Vyaw  = raw.raw_gyro_z  / 1000.0;
    IO.Pitch = raw.raw_mag_y   / 1000.0;
    IO.Yaw   = raw.raw_mag_z   / 1000.0;
    
    /// חישוב האמינות של החיישן
    double accel_noise = std::abs(IO.Ax);
    double conf_accel = std::max(0.1, 1.0 - (accel_noise / 10.0));
    
    double gyro_noise = std::abs(IO.Vyaw);
    double conf_gyro = std::max(0.1, 1.0 - (gyro_noise / 2.0));
    
    double pitch_noise = std::abs(IO.Pitch);
    double conf_pitch = std::max(0.1, 1.0 - (pitch_noise / 1.5));

    IO.confidence = conf_accel * conf_gyro * conf_pitch;

    // הגנה סופית על טווח האמינות
    IO.confidence = std::clamp(IO.confidence, 0.1, 1.0);
    ImuData data ;
    data.process(IO);
    return IO;
}