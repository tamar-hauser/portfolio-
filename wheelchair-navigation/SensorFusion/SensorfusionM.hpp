#pragma once
#include <vector>
#include <memory>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <type_traits>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "SensorFrame.hpp"
#include "SensorConfig.hpp"
#include "Constants.hpp"
#include "HungarianAlgorithm.hpp"

template <typename RawInput, typename OutputObject>

class SensorFusionM {
public:
    virtual ~SensorFusionM() = default;

    std::mutex mtx; 
     void process(std::vector<RawInput>& inputs, std::vector<OutputObject>& tracks)
    {
        bool has_inputs = !inputs.empty();
        bool has_tracks = !tracks.empty();

        if (has_inputs && has_tracks) {
            UnitingBoth(inputs, tracks);
        } 
        else if (has_tracks) {
            processUnitingObjectOnly(tracks); 
        } 
        else if (has_inputs) {
            processSensoreObjectOnly(inputs, tracks);
        }
    }

protected:
    HungarianAlgorithm HA;

     void processUnitingObjectOnly(std::vector<OutputObject>& tracks)
    {
        if (tracks.empty()) return;
        for (auto& obj : tracks) {
            increaseUncertainty(obj); 
        }
    }

     void processSensoreObjectOnly(std::vector<RawInput>& inputs, std::vector<OutputObject>& tracks)
    {
        if (inputs.empty()) return;
        for (auto& sensor_obj : inputs) {
            OutputObject new_Object = createNewObject(sensor_obj);
            std::lock_guard<std::mutex> lock(mtx);
            tracks.push_back(new_Object);
        }
    }

     void UnitingBoth(std::vector<RawInput>& detections, std::vector<OutputObject>& tracks)
    {
        const double INF_COST = 1e5;
        int numSensors = detections.size();
        int numObject = tracks.size();

        if constexpr (std::is_same_v<RawInput, SensorFusionObject>) {
            std::cout << "[UNIT] AssocMatrix start sensors=" << numSensors << " tracks=" << numObject << std::endl;
        }
        std::vector<std::vector<double>> AssociationMatrix = CreateAssociationMatrix(detections, tracks);
        if constexpr (std::is_same_v<RawInput, SensorFusionObject>) {
            std::cout << "[UNIT] AssocMatrix done, HA.Solve start" << std::endl;
        }

        std::vector<int> assignment;
        HA.Solve(AssociationMatrix, assignment);
        if constexpr (std::is_same_v<RawInput, SensorFusionObject>) {
            std::cout << "[UNIT] HA.Solve done, update loop start" << std::endl;
        }

        std::vector<bool> sensor_matched(numSensors, false);
        std::vector<bool> object_matched(numObject, false);

        for (int i = 0; i < numSensors; ++i) {
            int j = assignment[i];
            if (j != -1 && j < numObject && AssociationMatrix[i][j] < INF_COST) {
                if constexpr (std::is_same_v<RawInput, SensorFusionObject>) {
                    std::cout << "[UNIT] updateObject i=" << i << " j=" << j << std::endl;
                }
                std::lock_guard<std::mutex> lock(mtx);
                updateObject(detections[i], tracks[j]);
                if constexpr (std::is_same_v<RawInput, SensorFusionObject>) {
                    std::cout << "[UNIT] updateObject done i=" << i << std::endl;
                }
                sensor_matched[i] = true;
                object_matched[j] = true;
            }
        }
        if constexpr (std::is_same_v<RawInput, SensorFusionObject>) {
            std::cout << "[UNIT] uncertainty loop start" << std::endl;
        }

        for (int j = 0; j < numObject; ++j) {
            if (!object_matched[j]) {
                std::lock_guard<std::mutex> lock(mtx);
                increaseUncertainty(tracks[j]);
            }
        }
        if constexpr (std::is_same_v<RawInput, SensorFusionObject>) {
            std::cout << "[UNIT] new-obj loop start" << std::endl;
        }

        for (int i = 0; i < numSensors; ++i) {
            if (!sensor_matched[i]) {
                if constexpr (std::is_same_v<RawInput, SensorFusionObject>) {
                    std::cout << "[UNIT] createNewObject i=" << i << std::endl;
                }
                OutputObject new_obj = createNewObject(detections[i]);
                if constexpr (std::is_same_v<RawInput, SensorFusionObject>) {
                    std::cout << "[UNIT] createNewObject done i=" << i << std::endl;
                }
                std::lock_guard<std::mutex> lock(mtx);
                tracks.push_back(new_obj);
            }
        }
        if constexpr (std::is_same_v<RawInput, SensorFusionObject>) {
            std::cout << "[UNIT] UnitingBoth done" << std::endl;
        }
    }
    virtual void updateObject(RawInput& sensor_object, OutputObject& Object)=0;
    virtual OutputObject createNewObject(RawInput& sensor_object)=0;
   virtual void increaseUncertainty(OutputObject& Object)
{
    Object.confidence *= 0.95f;

    if constexpr (
        std::is_same_v<OutputObject, SensorFusionObject> ||
        std::is_same_v<OutputObject, TrackedObject>)
    {
        Object.filter.P *= 1.05;
    }
}
 std::vector<std::vector<double>> CreateAssociationMatrix(std::vector<RawInput>& sensor_list, std::vector<OutputObject>& object_list)
    {
        int rows = sensor_list.size();
        int cols = object_list.size();

        if (rows == 0 || cols == 0) return std::vector<std::vector<double>>();

        std::vector<std::vector<double>> cost_matrix(rows, std::vector<double>(cols));

        for (int i = 0; i < rows; ++i) {
            auto& current_sensor = sensor_list[i];

            for (int j = 0; j < cols; ++j) {
                auto& current_object = object_list[j];

                double current_cost = calculate_cost(current_sensor, current_object);
                cost_matrix[i][j] = current_cost;
            }
        }

        return cost_matrix;
    }
     float calculate_cost(RawInput& sensor_in, OutputObject& object)
    {
        const float INF_COST = 1e6f;

        // 1. Semantic Gating
        if constexpr (std::is_same_v<RawInput, CameraObject>||std::is_same_v<RawInput, SensorFusionObject>)
        {
          if (sensor_in.has_camera) {
            if (sensor_in.confidence > 0.8f && object.confidence > 0.8f) {
                if (sensor_in.type_label != object.type_label) return INF_COST;
            }
          }
        }
        

        // 2. Position Distance
        Eigen::Vector3f sensor_pos;
        if constexpr (std::is_same_v<RawInput, SensorFusionObject>) {
            sensor_pos = sensor_in.filter.x.head<3>().cast<float>();
        } else {
            if (sensor_in.Z.rows() < 3) return INF_COST;
            sensor_pos = sensor_in.Z.head<3>().cast<float>();
        }
        Eigen::Vector3f obj_pos;

        if constexpr (std::is_same_v<OutputObject, SensorFusionObject> || std::is_same_v<OutputObject, TrackedObject>) {
            obj_pos = object.filter.x.head<3>().cast<float>();
        } else {
            obj_pos = object.Z.head<3>().cast<float>();
        }

        float dist = (sensor_pos - obj_pos).norm();
        if (dist > 4.0f) return INF_COST;

        // 3. Dimensions Gating
        if constexpr (!std::is_same_v<RawInput, RadarObject>) {
            float max_dev = 0.6f;
            if constexpr (std::is_same_v<RawInput, LidarObject>) { max_dev = 0.3f; }

            float size_diff = (std::abs(sensor_in.length - object.length) / std::max(object.length, 0.1f) +
                               std::abs(sensor_in.width - object.width) / std::max(object.width, 0.1f) +
                               std::abs(sensor_in.height - object.height) / std::max(object.height, 0.1f)) / 3.0f;
            if (size_diff > max_dev) return INF_COST;
        }

        // 4. Mahalanobis
        double d2 = calculateMahalanobis(sensor_in, object);
        float threshold = Config::getChiSquareThreshold(sensor_in.Z.rows());
        if (d2 > threshold) return INF_COST;

        float final_cost = static_cast<float>(d2) / threshold;
        float weight_sum = 1.0f;

        // 5. IoU (Camera-Lidar)
        if constexpr (std::is_same_v<RawInput, CameraObject>||std::is_same_v<RawInput, SensorFusionObject>) {
        if (object.has_lidar) {
            float iou = calculateIoU(sensor_in, object);
            if (iou < Config::MinIoUThreshold && d2 > (threshold * 0.5f)) return INF_COST;
            final_cost += (1.0f - iou); 
            weight_sum += 1.0f;
          }
         } 
        else if constexpr (std::is_same_v<RawInput, LidarObject>||std::is_same_v<RawInput, SensorFusionObject>) {
            if (object.has_camera) {
            float iou = calculateIoU(sensor_in, object);
            if (iou < Config::MinIoUThreshold && d2 > (threshold * 0.5f)) return INF_COST;
            final_cost += (1.0f - iou); 
            weight_sum += 1.0f;
            }
        }

        // 6. Confidence & Radar
        float joint_conf = sensor_in.confidence * object.confidence;
        final_cost += (1.0f - joint_conf) * 0.5f;
        weight_sum += 0.5f;
        float obj_speed = object.velocity().norm();
        float sensor_speed;
        if constexpr (std::is_same_v<RawInput, SensorFusionObject>) {
            sensor_speed = sensor_in.filter.x.template segment<3>(3).cast<float>().norm();
        } else {
            sensor_speed = (sensor_in.Z.rows() >= 3) ? sensor_in.Z.head<3>().cast<float>().norm() : 0.0f;
        }
        float vel_diff = std::abs(sensor_speed - obj_speed);

        final_cost += (vel_diff / Config::MaxVelDiff); 
        weight_sum += 1.0f;

        return final_cost / weight_sum;
    }
     float calculaterect(const cv::Rect& rectA, const cv::Rect& rectB)
    {
        cv::Rect interRect = rectA & rectB;
        float interArea = interRect.area();

        if (interArea <= 0) {
            return 0.0f;
        }

        float areaA = rectA.area();
        float areaB = rectB.area();
        float unionArea = areaA + areaB - interArea;

        return interArea / unionArea;

    }
     float calculateAlpha( const RawInput& sensor,const OutputObject& Object)
    {
        double delta_t = sensor.timestamp - Object.timestamp;

        const float lambda = 2.0f;

        float exponent = -1.0f * (lambda * static_cast<float>(delta_t) * sensor.confidence);

        float alpha = 1.0f - std::exp(exponent);

        return std::clamp(alpha, 0.05f, 0.95f);
    } 
     double calculateMahalanobis(RawInput& sensor_in, OutputObject& object)
    {
        int dof = sensor_in.Z.rows();
        if (dof <= 0 ||
            dof > sensor_in.H.rows() ||
            Config::StateSizeObject > sensor_in.H.cols() ||
            dof > sensor_in.R.rows() ||
            dof > sensor_in.R.cols())
        {
            return std::numeric_limits<double>::max();
        }
        Eigen::VectorXd z = sensor_in.Z.cast<double>();
        Eigen::MatrixXd H = sensor_in.H.block(0, 0, dof, Config::StateSizeObject).cast<double>();
        Eigen::MatrixXd R_temp = sensor_in.R.block(0, 0, dof, dof).cast<double>();
        Eigen::MatrixXd S = H * object.filter.P * H.transpose() + R_temp;
        Eigen::VectorXd y = z - (H * object.filter.x);

        double d2 = y.transpose() * S.ldlt().solve(y);
        return d2;

    }
     double calculateIoU(RawInput& sensor_in, OutputObject& object)
    {
        cv::Rect cam_rect;
        pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud = nullptr;
        float iou = 0.0f;

        if constexpr (std::is_same_v<RawInput, CameraObject>) {
            if (object.has_lidar) {
                cam_rect = sensor_in.bounding_box;
                lidar_cloud = object.cloud;
            }
        } 
        else if constexpr (std::is_same_v<RawInput, LidarObject>) {
            if (object.has_camera) {
                cam_rect = object.bounding_box;
                lidar_cloud = sensor_in.cloud;
            }
        } 
        else if constexpr (std::is_same_v<RawInput, SensorFusionObject>) {
            if (object.has_lidar && sensor_in.has_camera) {
                cam_rect = sensor_in.bounding_box;
                lidar_cloud = object.cloud;
            }
            else if (sensor_in.has_lidar && object.has_camera) { 
                cam_rect = object.bounding_box;
                lidar_cloud = sensor_in.cloud;
            }
        }

        if (cam_rect.empty()) return 0.0;

        if (lidar_cloud && !lidar_cloud->empty()) {
            std::cout << "[IoU] calling projectLidar cloud_size=" << lidar_cloud->size()
                      << " cam_rect=" << cam_rect.x << "," << cam_rect.y
                      << "," << cam_rect.width << "x" << cam_rect.height << std::endl;
            cv::Rect proj = projectLidarCloudObject(lidar_cloud);
            std::cout << "[IoU] projectLidar done rect=" << proj.x << "," << proj.y
                      << " " << proj.width << "x" << proj.height << std::endl;
            iou = calculaterect(proj, cam_rect);
        }

        return static_cast<double>(iou);
    }   
    virtual cv::Rect projectLidarCloudObject(const pcl::PointCloud<pcl::PointXYZI>::Ptr& objectCloud)
    {
    if (!objectCloud || objectCloud->empty()) {
        return cv::Rect();
    }

    auto lidarConfig = ConfigManager::getLidar("front_lidar");
    auto cameraConfig = ConfigManager::getCamera("rgb_camera");

    Eigen::Matrix3f R_lidar2cam = cameraConfig.rotation.transpose() * lidarConfig.rotation;
    Eigen::Vector3f T_lidar2cam = cameraConfig.rotation.transpose() * (lidarConfig.translation - cameraConfig.translation);

    std::vector<cv::Point3f> points3D;
    points3D.reserve(objectCloud->size());

    for (const auto& pcl_point : objectCloud->points) {
        Eigen::Vector3f pt_lidar(pcl_point.x, pcl_point.y, pcl_point.z);
        Eigen::Vector3f pt_camera = R_lidar2cam * pt_lidar + T_lidar2cam;

        if (pt_camera.z() > 0.1f) {
            points3D.push_back(cv::Point3f(pcl_point.x, pcl_point.y, pcl_point.z));
        }
    }

    if (points3D.empty()) {
        return cv::Rect();
    }

    cv::Mat R_cv, tvec;
    cv::eigen2cv(R_lidar2cam, R_cv);
    cv::eigen2cv(T_lidar2cam, tvec);

    cv::Mat rvec;
    cv::Rodrigues(R_cv, rvec);

    Eigen::Matrix3f K_eigen = Config::getK();
    cv::Mat cameraMatrix;
    cv::eigen2cv(K_eigen, cameraMatrix);

    cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_32F);

    std::vector<cv::Point2f> points2D;
    std::cout << "[PROJ] cv::projectPoints start pts=" << points3D.size() << std::endl;
    cv::projectPoints(points3D, rvec, tvec, cameraMatrix, distCoeffs, points2D);
    std::cout << "[PROJ] cv::projectPoints done pts2D=" << points2D.size() << std::endl;

    std::cout << "[PROJ] cv::boundingRect start" << std::endl;
    cv::Rect result = cv::boundingRect(points2D);
    std::cout << "[PROJ] cv::boundingRect done rect=" << result.x << "," << result.y
              << " " << result.width << "x" << result.height << std::endl;
    return result;
    }
};