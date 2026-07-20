#include <vector>
// ודאי שאת מכלילה את הקבצים של האובייקטים ומחלקת האב
 #include "RadarObject.hpp" 
 #include "SensorData.hpp"

class RadarData : public SensorData<RadarObject> {
public:
    RadarData() = default;    
    
    void process(RadarObject& RD) override;

private:

    void buildZ(RadarObject& RD) override; 
    void buildR(RadarObject& RD) override;
    void buildH(RadarObject& RD) override; 

};
