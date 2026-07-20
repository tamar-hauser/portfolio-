#pragma once
#include <vector>
#include "EncoderObject.hpp"
#include "Thread\SensorProducerManager.hpp"
#include "SensorProcessing.hpp"

class EncoderProcessing : public SensorProcessing<EncoderMeasurement, EncoderObject> {
public:
    EncoderProcessing() = default;
    EncoderObject process(EncoderMeasurement& input, double ts) override;

private:
    EncoderObject createObject(EncoderMeasurement& raw);
};