#pragma once
#include <vector>
#include "SensorProcessing.hpp" // תיקון שם הקובץ והסרת שגיאת הכתיב
#include "GpsObject.hpp"
#include "GPSLocation.hpp" // תיקון: הוספת ה-Include כדי שהקומפיילר יזהה את הטיפוס
class GpsProcessing : public SensorProcessing<std::stringstream, GpsObject> {
public:
    GpsProcessing() = default; 

    GpsObject process(std::stringstream& input,double ts) override;

protected:
    GpsObject createObject(GPSLocation& raw);
private:
    GPSLocation m_parser; // <-- התיקון: המפענח נשמר ברמת המחלקה ולא מתאפס!
};