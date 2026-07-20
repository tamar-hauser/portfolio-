#pragma once
#include "Thread\SensorProducerManager.hpp"
#include <vector>
#include "SensorObject.hpp" // הבסיס של כל האובייקטים
#include "SensorProcessing.hpp" // הבסיס של כל האובייקטים

class ImuProcessing :public SensorProcessing<ImuMeasurement, ImuObject> {
public:
    ImuProcessing() = default; 
    

    // הפונקציה שמנהלת את הזרם: מקבלת מידע גולמי ומחזירה רשימת אובייקטים מעובדים
 ImuObject process(ImuMeasurement& input,double ts) override;

protected:
   ImuObject createObject(ImuMeasurement& raw);
};