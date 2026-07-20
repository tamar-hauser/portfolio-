#pragma once
#include "ImuObject.hpp"
#include "SensorData.hpp"

class ImuData : public SensorData<ImuObject> {
public:
  ImuData() = default;  
  void process(ImuObject& IO) override;
   
private:  
    void buildZ(ImuObject& IO) override; 
    void buildH(ImuObject& IO) override;
    void buildR(ImuObject& IO)override;
     
};
