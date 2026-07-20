#pragma once

template <typename T>
struct SensorMeasurement {
    double timestamp = 0.0; 
    T data;              
};