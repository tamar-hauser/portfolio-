#include <iostream>
#include <string>
#include <sstream>  // תוקן: נוסף עבור std::stringstream
#include "Gps/GpsProcessing.hpp" 
#include "Gps/GpsObject.hpp"

#include <iostream>
#include <string>
#include <sstream>  // תוקן: נוסף עבור std::stringstream
#include "Gps/GpsProcessing.hpp" 
#include "Gps/GpsObject.hpp"

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "Sensor/SensorConfig.hpp"
#include "Camera/CameraProcessing.hpp"
#include "Camera/CameraData.hpp"
#include "Camera/CameraObject.hpp"

int main() {
    std::string config_path = "C:/Users/User/Desktop/TrackObject/SensorConfig.yaml";

    std::cout << "Loading sensor configurations from YAML..." << std::endl;
    
    if (!ConfigManager::load(config_path)) {
        std::cerr << "CRITICAL ERROR: Failed to load configuration file!" << std::endl;
        return -1; // עוצרים את התוכנית אם אין כיול לחיישנים
    }

    std::cout << "Configuration loaded successfully!" << std::endl;  
    
    std::cout << "==================================================\n";
    std::cout << "        Starting Camera Pipeline Test Only        \n";
    std::cout << "==================================================\n\n";

    cv::Mat dummyFrame = cv::Mat::zeros(480, 640, CV_8UC3);
    if (dummyFrame.empty())
{
    std::cout << "dummyFrame is empty!" << std::endl;
}
    // לולאת ה-try-catch נועדה לתפוס שגיאות של OpenCV (למשל, אם קובץ ה-ONNX של YOLO חסר)
    try {
        // 2. יצירת אובייקטים של מחלקות העיבוד והנתונים שלך
        double ts=111.0;
        CameraProcessing camProcessor;
        cv::Mat frame = cv::imread("IMG_7257.jpg");

        if (frame.empty())
        {
            std::cout << "Failed to load image!" << std::endl;
            return -1;
        }
        else
        {
            std::cout << "Image loaded successfully!" << std::endl;
        }
        std::cout << "[MAIN] Step 1: Sending frame to CameraProcessing (YOLO DNN)...\n";
        
        // 3. שלב א': הפעלת ה-Pipeline הראשי של ה-Processing
        // הפונקציה מפעילה את הרשת הנוירונית ומחזירה וקטור של אובייקטים מעובדים (CameraObject)
        std::vector<CameraObject> detectedObjects = camProcessor.process(frame);

        std::cout << "[MAIN] Step 1 Completed. Detected " << detectedObjects.size() << " objects.\n";

        // 4. שלב ב': מעבר על האובייקטים שזוהו והזרמתם לתוך ה-Data (הכנה לקלמן)
        if (detectedObjects.empty()) {
            std::cout << "\n[!] Notice: No objects were detected in the dummy frame.\n";
            std::cout << "    (This is expected since the input image is completely black).\n";
        } else {
            for (size_t i = 0; i < detectedObjects.size(); ++i) {
                std::cout << "\n----------------------------------------\n";
                std::cout << "Processing Camera Object #" << i << "\n";
                std::cout << "----------------------------------------\n";
                
               
                // הדפסת הנתונים שחולצו ובנו לתוך האובייקט
                std::cout << "Object Label:    " << detectedObjects[i].type_label << "\n";
                std::cout << "2D Centroid Pixel: (" << detectedObjects[i].position_2d.x << ", " << detectedObjects[i].position_2d.y << ")\n";
                std::cout << "Confidence Score:  " << detectedObjects[i].confidence << "\n";
                std::cout << "3D Physical Sizes: L=" << detectedObjects[i].length 
                          << ", W=" << detectedObjects[i].width 
                          << ", H=" << detectedObjects[i].height << "\n";
                              
            }
        }

    } catch (const cv::Exception& e) {
        // שגיאה נפוצה מאוד: OpenCV יזרוק שגיאה אם הוא לא ימצא את הקובץ 'yolov8n.onnx' 
        std::cerr << "\n[!] OpenCV DNN Error: " << e.what() << "\n";
        std::cerr << "--> CRITICAL TIP: Ensure the file 'yolov8n.onnx' is located in the exact same folder as your executable (.exe)!\n";
    } catch (const std::exception& e) {
        std::cerr << "\n[!] Standard C++ Error: " << e.what() << "\n";
    }

    std::cout << "\n==================================================\n";
    std::cout << "              Camera Test Completed               \n";
    std::cout << "==================================================\n";

    return 0;
}