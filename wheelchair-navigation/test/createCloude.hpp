#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>

namespace TestUtils {
    inline void generateLidarTestData() {
        // מיקום אובייקט דמיוני: 5 מטר קדימה מהרובוט
        float objX = 5.0f, objY = 0.0f, objZ = 0.0f;

        // 1. ענן לידאר קדמי (Front)
        pcl::PointCloud<pcl::PointXYZI> front_cloud;
        for (float y = -0.5; y <= 0.5; y += 0.05) {
            for (float z = -0.2; z <= 0.5; z += 0.05) {
                pcl::PointXYZI p;
                p.x = objX; p.y = y; p.z = z; p.intensity = 200;
                front_cloud.push_back(p);
            }
        }
        pcl::io::savePCDFileBinary("lidar_front_test.pcd", front_cloud);

        // 2. ענן לידאר ימני (Right) - רואה דופן ימנית
        pcl::PointCloud<pcl::PointXYZI> right_cloud;
        for (float x = objX; x <= objX + 1.0; x += 0.05) {
            for (float z = -0.2; z <= 0.5; z += 0.05) {
                pcl::PointXYZI p;
                p.x = x; p.y = -0.5; p.z = z; p.intensity = 150;
                right_cloud.push_back(p);
            }
        }
        pcl::io::savePCDFileBinary("lidar_right_test.pcd", right_cloud);

        // 3. ענן לידאר שמאלי (Left) - רואה דופן שמאלית
        pcl::PointCloud<pcl::PointXYZI> left_cloud;
        for (float x = objX; x <= objX + 1.0; x += 0.05) {
            for (float z = -0.2; z <= 0.5; z += 0.05) {
                pcl::PointXYZI p;
                p.x = x; p.y = 0.5; p.z = z; p.intensity = 150;
                left_cloud.push_back(p);
            }
        }
        pcl::io::savePCDFileBinary("lidar_left_test.pcd", left_cloud);

        std::cout << "Successfully generated 3 test PCD files." << std::endl;
    }
}