#pragma once
#include <vector>
#include "RadarObject.hpp" // הבסיס של כל האובייקטים
#include "SensorProcessing.hpp" // הבסיס של כל האובייקטים
#include "SensorConfig.hpp"
#include "Thread\SensorProducerManager.hpp"

// תוקן: RadarMeasurement במקום RadarMeasurement
class RadarProcessing : public SensorProcessing< std::vector<RadarMeasurement>, std::vector<RadarObject> > {
public:
    RadarProcessing() = default; 

    std::vector<RadarObject> process(std::vector<RadarMeasurement>& input, double ts) override;

protected:
    std::vector<RadarMeasurement> Clean(std::vector<RadarMeasurement> r);
    RadarObject createObject(RadarMeasurement& r);
};