#pragma once
#include "SensorFusionObject.hpp"
#include "SensorData.hpp"
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

// תיקון: שינוי ל-SensorfusionObject (f קטנה) כדי להתאים להגדרת ה-struct
class SensorFusionData : public SensorData<SensorFusionObject> {
public:
    SensorFusionData() = default;

    void process(SensorFusionObject& frame) override;

private:
   // תיקון קריטי: שינוי הטיפוס מ-SensorFusionData ל-cv::Mat כדי למנוע קריסת זיכרון/רקורסיית גודל
   cv::Mat currentFrame; 
   
   void buildZ(SensorFusionObject& SO) override; 
   void buildH(SensorFusionObject& SO) override;
   void buildR(SensorFusionObject& SO) override;
   void buildF(SensorFusionObject& SO);
};