#pragma once
#include <Eigen/Dense>
#include "SensorData.hpp"
#include "EncoderObject.hpp"
class EncoderData : public SensorData<EncoderObject> {
public:
    EncoderData() = default;
    void process(EncoderObject& EO) override;
private:
    void buildZ(EncoderObject& IO) override; 
    void buildH(EncoderObject& IO) override;
    void buildR(EncoderObject& IO) override;
};