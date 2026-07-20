#pragma once 
#include <vector>
#include <memory> 
#include <Eigen/Dense>
template <typename OutputObject>
class SensorData {
public:
    virtual ~SensorData() {}

    // שימוש בוקטור של מצביעים חכמים במקום העברה לפי ייחוס (Reference)
    virtual void process(OutputObject& O) = 0;
    
protected:
    virtual void buildZ(OutputObject& o) = 0;
    virtual void buildR(OutputObject& o) = 0;
    virtual void buildH(OutputObject& O) = 0;
};