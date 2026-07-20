#pragma once
#include "SensorFusionM.hpp"
#include "SensorFusionObject.hpp"
#include "Radar/RadarObject.hpp"
#include "HungarianAlgorithm.hpp"
#include "Constants.hpp"
#include "KalmanFilter/ekf.hpp"


class SensorFusionRadar : public SensorFusionM<RadarObject,SensorFusionObject>
{
public:
     ~SensorFusionRadar() = default;

    // מקבלת חבילת נתונים ומחזירה וקטור של אובייקטים מאוחדים
    
protected:
     HungarianAlgorithm HA;
     void updateObject(RadarObject& ro, SensorFusionObject& sensor_object)override;
     SensorFusionObject createNewObject(RadarObject& RO) override;
};