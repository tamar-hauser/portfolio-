#pragma once 
#include <Eigen/Dense>
#include <string>

namespace Config {
    // הוספת המש0'תנה שהיה חסר לבנאי של LidarObject
    constexpr int SensorFusionLidar = 4; // גודל וקטור המדידה של לידאר (x,y,z,yaw)
    constexpr double resolution = 0.10; // גודל וקטור המדידה של לידאר (x,y,z,yaw)
    constexpr double alpha = 0.15;//תיקון הווקטור מצב מהגרף פקטור
    constexpr int StateSizeObject = 8;          
    constexpr int DimWheelchairStateVector = 8; 

    constexpr int MeasurementSizeCamera = 3;    
    constexpr int MeasurementSizeLidar = 4;     
    constexpr int MeasurementSizeRadar = 6;     
    constexpr int MeasurementSizeGps = 5;
    constexpr int MeasurementSizeImu = 2; // Ax, Vyaw בלבד — Pitch/Yaw חסומים (מגנטומטר גולמי לא מכוייל)
    constexpr int MeasurementSizeEncoder = 2;
    constexpr int MeasurementSizeSensorObgect = 1;
    constexpr float MinIoUThreshold = 0.1f;
    constexpr float MaxVelDiff = 5.0f; // <--- הוסיפי את זה! (את יכולה לשנות את 5.0 לערך שמתאים לך)
    constexpr double ImuDt = 0.01;
    constexpr double EncedorDt = 0.02;

    ////הגדרות אינקודר
    constexpr double WHEEL_RADIUS = 0.34;
    constexpr double TRACK_WIDTH = 0.96;
    constexpr double TICKS_TO_RAD = 6.28318530718 / 2048.0; // 2π/2048 ≈ 0.00307 rad/tick
    ////משקולות לICP
    constexpr float W_STATIC     = 0.20f;
    constexpr float W_VETERAN    = 0.15f;
    constexpr float W_CONFIDENCE = 0.25f;
    constexpr float W_SIZE       = 0.20f; // משקל לגודל האובייקט
    constexpr float W_STABILITY  = 0.20f;
    static constexpr double CW_STATIC = 1.0; // הוסף את הערך המתאים
    static constexpr double EncoderDt = 0.01;
    /// תנאים מוקדמים לפתיחת צומת חדשה בגרף פקטור
    constexpr double DISTANCE_THRESHOLD = 0.50; // תזוזה של 50 ס"מ
    constexpr double ANGLE_THRESHOLD = 0.35;   // שינוי של 20 מעלות (ברדיאנים)
enum class StateIndicesObject {
        StateX = 0, StateY = 1, StateZ = 2,
        StateVx = 3, StateVy = 4, StateVz = 5,
        StateYaw = 6, StateYawRate = 7, DimState = 8 
};
enum class StateMembersRobot
{
    StateX = 0,
    StateY=1,
    StateZ=2,

    StatePitch=3,
    StateYaw=4,

    StateVx=5,
    StateVyaw=6,

    StateAx=7
};
    inline Eigen::Matrix3f getK() {
        Eigen::Matrix3f K;
        // 640x480, FOV_h=1.05rad: fx = (640/2)/tan(1.05/2) ≈ 559
        K << 559.0f, 0.0f, 320.0f,
             0.0f, 559.0f, 240.0f,
             0.0f,   0.0f,   1.0f;
        return K;
    }

    
    inline constexpr float camera_height = 0.80f;
    constexpr float MaxDimDev = 0.4f;            
    constexpr float DefaultVoxelSize = 0.1f;    

    inline float getChiSquareThreshold(int dof) {
        switch(dof) {
            case 2:  return 9.21f;
            case 3:  return 11.34f;
            case 4:  return 13.28f;
            case 8:  return 20.09f;
            default: return 15.0f;
        }
    }
}; 
