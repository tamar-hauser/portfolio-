#include "SensorObject.hpp"

#include "CameraObject.hpp"
#include "CameraData.hpp"

void CameraData::process( CameraObject& CO) {
    // השלמת פרמטרים וכיול לפני הכנסה לקלמן
    buildZ(CO); // בונה את וקטור המדידה
    buildH(CO); // מחשבת את מטריצת המעבר
    buildR(CO); // קובעת את הרעש הדינמי
   
}

// void CameraData::fillParamForCompute(CameraObject& CO) {
//     // 1. טיפול בעיוותי עדשה (Distortion Correction) אם נדרש
//     // 2. תזוזת Bias - אם המצלמה ממוקמת 0.5 מטר קדימה ממרכז הכיסא:
//     // CO.position_3d.x() += 0.5;
// }

void CameraData::buildZ(CameraObject& CO) {
    CO.Z(0) = CO.position_3d.x();
    CO.Z(1) = CO.position_3d.y();
    CO.Z(2) = CO.position_3d.z();

}

void CameraData::buildH(CameraObject& CO) {
    CO.H.setZero();
    CO.H(0,0) = 1;
    CO.H(1,1) = 1;
    CO.H(2,2) = 1;
}


void CameraData::buildR(CameraObject& CO) {
    CO.R.setZero();

    double var = 0.1 / (CO.confidence + 0.001);

    CO.R(0,0) = var;
    CO.R(1,1) = var;
    CO.R(2,2) = var * 1.5;
}

