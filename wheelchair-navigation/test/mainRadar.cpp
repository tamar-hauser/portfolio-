#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include "Sensor/SensorManager.hpp"
#include "Sensor/SensorRing.hpp"
#include "Sensor/SensorMeasurement.hpp"
#include "Radar/RadarObject.hpp"
#include "Radar/RadarProcessing.hpp"
#include "Radar/RadarData.hpp"

// פונקציית עזר ליצירת נתונים גולמיים המותאמת ל-struct הקיים ב-SensorManager
std::vector<RadarMeasurement> generateRawRadarHardwareData() {
    std::vector<RadarMeasurement> frame_data;

    // אובייקט מדומה 1: מטרה במסלול תלת-ממדי
    RadarMeasurement target;
    target.id = 1;
    target.pos_x = 15.0f;
    target.pos_y = 2.5f;
    target.pos_z = 1.0f;  // הוספת מימד Z כפי שקיים כעת במבנה
    target.vel_x = 1.2f;
    target.vel_y = -0.3f;
    target.vel_z = 0.0f;
    // אובייקט מדומה 2: רעש/מדידה ריקה בראשית הצירים (תסונן על ידי ה-Clean)
    RadarMeasurement clutter;
    clutter.id = 2;
    clutter.pos_x = 0.0f;
    clutter.pos_y = 0.0f;
    clutter.pos_z = 0.0f;
    clutter.vel_x = 0.0f;
    clutter.vel_y = 0.0f;
    clutter.vel_z = 0.0f;
    frame_data.push_back(target);
    frame_data.push_back(clutter);

    return frame_data;
}

int main() {
    std::cout << "=== Initialize Radar Pipeline with SensorRing ===" << std::endl;

    // 1. השגת המופע היחיד של ה-SensorManager (Singleton)
    SensorManager& manager = SensorManager::getInstance();

    // 2. אתחול מחלקת העיבוד של הראדאר
    RadarProcessing radar_processor;
    
    // סימולציית ריצה על פני מספר פריימים
    double current_timestamp = 1000.0; 

    for (int i = 0; i < 3; ++i) {
        current_timestamp += 66.0; // קצב דגימה של בערך 15Hz
        std::cout << "\n----------------------------------------\n";
        std::cout << "FRAME #" << i + 1 << " | TS: " << current_timestamp << std::endl;

        // ==========================================
        // שלב 1: כניסה לרינג (הדמיית הגעת מידע מהחיישן)
        // ==========================================
        std::vector<RadarMeasurement> raw_hardware_data = generateRawRadarHardwareData();
        
        // דחיפת הנתונים לתוך ה-RadarRing המנוהל בתוך ה-SensorManager
        manager.addRadarUpdate(current_timestamp, raw_hardware_data);
        std::cout << "[SensorManager] Inserted raw data frame into radarRing." << std::endl;

        // ==========================================
        // שלב 2: יציאה מהרינג לעיבוד
        // ==========================================
        // שליפת המדידה האחרונה שנכנסה ל-Ring
        SensorMeasurement<std::vector<RadarMeasurement>> latest_measurement = manager.radarRing.getLast();
        
        // שליפת וקטור המדידות מתוך הרינג (שימוש בשמות השדות הנכונים dt ו-dataSensor)
        std::vector<RadarMeasurement>& radar_input = latest_measurement.dataSensor;
        radar_processor.setTs(latest_measurement.dt); // עדכון ה-timestamp במחלקת העיבוד

        std::cout << "[SensorRing] Successfully fetched frame from Ring buffer." << std::endl;

        // ==========================================
        // שלב 3: הרצת ה-Pipeline של הראדאר
        // ==========================================
        // א. שלב העיבוד: ניקוי רעשים ויצירת אובייקטים תלת-ממדיים מעובדים (RadarObject)
        std::vector<RadarObject> processed_objects = radar_processor.process(radar_input);
        std::cout << "[RadarProcessing] Processed objects remaining after filter: " << processed_objects.size() << std::endl;

        // ב. שלב הנתונים: שליחת כל אובייקט מעובד לקלמן דרך RadarData
        if (!processed_objects.empty()) {
            RadarData radar_data_handler;

            // הרצת ה-process של ה-Kalman Filter עבור האובייקטים
            for (auto& obj : processed_objects) {
                radar_data_handler.process(obj);
                
                // הדפסת המידע שהתקבל והתעדכן בקלמן
                std::cout << "   -> Object Validated!" << std::endl;
                std::cout << "      Position (X, Y, Z): (" << obj.position.x() << ", " << obj.position.y() << ", " << obj.position.z() << ")" << std::endl;
                std::cout << "      RCS (Dynamic): " << obj.rcs << " | Confidence: " << obj.confidence << std::endl;
                std::cout << "      Measurement Vector Z matrix rows: " << obj.Z.rows() << std::endl;
            }
        } else {
            std::cout << "[Warning] No objects passed the radar clean criteria." << std::endl;
        }

        // השהייה סמלית בין הפריימים
        std::this_thread::sleep_for(std::chrono::milliseconds(66));
    }

    std::cout << "\n=== Pipeline Execution Finished ===" << std::endl;
    return 0;
}