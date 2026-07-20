#include "Sensor\SensorManager.hpp"
#include "Sensor\SensorConfig.hpp"
#include "Sensor\SensorRing.hpp"
#include "Lidar\LidarProcessing.hpp"
#include "main\createCloude.hpp"

int main() {
    TestUtils::generateLidarTestData();
    if (!ConfigManager::load("SensorConfig.yaml")) {
        return -1;
    }

    // יוצרים ישר shared_ptr כדי שיתאים ל-SensorManager ול-loadPCDFile
    auto cloud_front = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    auto cloud_right = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    auto cloud_left = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    // טוענים את המידע ישירות לתוך העננים הללו
    if (pcl::io::loadPCDFile("lidar_front_test.pcd", *cloud_front) == -1 ||
        pcl::io::loadPCDFile("lidar_right_test.pcd", *cloud_right) == -1 ||
        pcl::io::loadPCDFile("lidar_left_test.pcd", *cloud_left) == -1) {
        std::cerr << "Error: Failed to load PCD files!" << std::endl;
        return -1;
    }
    
    auto& manager = SensorManager::getInstance();
    LidarProcessing processor;

    double ts = 1000.0; 
    // עכשיו את מכניסה את העננים שמכילים את המידע האמיתי שנקרא מהקובץ!
    manager.addLidarUpdate(ts, cloud_front); 
    manager.addLidarUpdate(ts, cloud_right);
    manager.addLidarUpdate(ts, cloud_left);
    processor.setTs(ts);

    try {
        // שליפה של ה-3 האחרונים
        auto measurements = manager.lidarRing.popLastN(3);

        // יצירת וקטור של העננים בלבד עבור המעבד
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> raw_clouds;
        for (auto& meas : measurements) {
            raw_clouds.push_back(meas.dataSensor);
        }

        // עכשיו הוקטור raw_clouds יכיל מידע אמיתי, והשגיאה והקריסה ייפתרו!
        std::vector<LidarObject> detected_objects = processor.process(raw_clouds);

        std::cout << "Detected " << detected_objects.size() << " objects from 3 lidars." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error in processing: " << e.what() << std::endl;
    }

    return 0;
}