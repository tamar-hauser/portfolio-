#pragma once  
#include <vector>

template <typename RawInput, typename OutputType>
class SensorProcessing {
public:
    virtual ~SensorProcessing() = default;    
    virtual OutputType process(RawInput& input,double ts) = 0;
    virtual double getTs() const { return ts; }
    virtual void setTs(double newts)  { this->ts=newts; }
 
protected:
    double ts;

};