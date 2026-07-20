#include <vector>
#include <set>
#include <filesystem>
#include "SensorObject.hpp"
#include "CameraProcessing.hpp"
#include "CameraObject.hpp"
#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include "CameraData.hpp"

#include "Constants.hpp"
#include "SensorConfig.hpp"
#include "CameraProcessing.hpp"
#include <opencv2/dnn.hpp>
#include <iostream>
CameraProcessing::CameraProcessing()
{
    const std::string customModel = "C:/Users/User/Desktop/TrackObject/models/yolov8s_webots.onnx";
    if (std::filesystem::exists(customModel)) {
        modelPath = customModel;
        usingCustomModel = true;
        std::cout << "[CAMERA][YOLO] מודל מאומן על Webots נטען: " << modelPath << "\n";
    } else {
        std::cout << "[CAMERA][YOLO] מודל מאומן לא נמצא — שימוש ב-yolov8n בסיסי\n";
    }

    net = cv::dnn::readNetFromONNX(modelPath);
    if (net.empty()) {
        std::cerr << "[CameraProcessing][ERROR] Failed to load model: " << modelPath << std::endl;
    } else {
        std::cout << "[CAMERA][YOLO] model loaded successfully: " << modelPath << std::endl;
    }
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    K = Config::getK();
}
std::vector<CameraObject> CameraProcessing::process(cv::Mat& input,double tss) {
    this->ts=tss;
    std::vector<CameraObject> myCamera;
    if (input.empty()) {
        std::cerr << "Error: Input frame to process is empty!" << std::endl;
        return myCamera;
    }

    this->currentFrame = input.clone();

    {
        static int ds_saved = -1; 
        static const std::string DS_DIR =
            "C:/Users/User/Desktop/TrackObject/dataset_v2/raw_frames";

        if (ds_saved == -1) {
            std::filesystem::create_directories(DS_DIR);
            ds_saved = 0;
            for (auto& entry : std::filesystem::directory_iterator(DS_DIR))
                if (entry.path().extension() == ".jpg") ds_saved++;
            std::cout << "[DATASET] מתחיל מ-frame " << ds_saved << "\n";
        }

        if (ds_saved < 200) {
            char dspath[300];
            snprintf(dspath, sizeof(dspath),
                     "%s/frame_%05d.jpg", DS_DIR.c_str(), ++ds_saved);
            cv::imwrite(dspath, input);
            std::cout << "[DATASET] נשמר " << ds_saved << "/200\n";
            if (ds_saved >= 200)
                std::cout << "[DATASET] הושלם — 200 frames!\n";
        }
    }

    std::vector<DetectedItem> raw_detections = yolo(input);
    for (const auto& detection : raw_detections) {
        CameraObject co = createObject(detection);
        myCamera.push_back(co);
    }

    return myCamera;
}

std::vector<DetectedItem> CameraProcessing::yolo(const cv::Mat& input) {
    std::vector<DetectedItem> results; 
    
    
    if (input.empty()) {
        std::cerr << "Error: Input image is empty!" << std::endl;
        return results; 
    }

    if (net.empty()) {
        std::cerr << "[YOLO][ERROR] network not loaded; skipping detection." << std::endl;
        return results;
    }

    static int yolo_call_count = 0;
    ++yolo_call_count;
    bool should_log = (yolo_call_count <= 3) || (yolo_call_count % 50 == 0);

    if (should_log)
        std::cout << "[CAMERA][YOLO] input image size="
                  << input.cols << "x" << input.rows
                  << " channels=" << input.channels()
                  << " empty=" << input.empty() << std::endl;

    cv::Mat blob;
    cv::dnn::blobFromImage(input, blob, 1.0 / 255.0, cv::Size(640, 640), cv::Scalar(), true, false);

    if (should_log)
        std::cout << "[CAMERA][YOLO] blob dims=" << blob.dims
                  << " [" << (blob.dims > 0 ? blob.size[0] : -1)
                  << "," << (blob.dims > 1 ? blob.size[1] : -1)
                  << "," << (blob.dims > 2 ? blob.size[2] : -1)
                  << "," << (blob.dims > 3 ? blob.size[3] : -1) << "]" << std::endl;

    net.setInput(blob);

    std::vector<cv::Mat> outputs;
    net.forward(outputs, net.getUnconnectedOutLayersNames());

    if (should_log) {
        std::cout << "[CAMERA][YOLO] outputs count=" << outputs.size() << std::endl;
        for (size_t oi = 0; oi < outputs.size(); ++oi)
            std::cout << "[CAMERA][YOLO] output[" << oi << "] dims=" << outputs[oi].dims
                      << " [" << (outputs[oi].dims > 0 ? outputs[oi].size[0] : -1)
                      << "," << (outputs[oi].dims > 1 ? outputs[oi].size[1] : -1)
                      << "," << (outputs[oi].dims > 2 ? outputs[oi].size[2] : -1) << "]" << std::endl;
    }
    if (outputs.empty()) {
        std::cerr << "YOLO outputs are empty!" << std::endl;
        return results;
    }
    
    cv::Mat detMat = outputs[0];
    if (detMat.dims == 3) {
        detMat = detMat.reshape(1, detMat.size[1]); 
        cv::transpose(detMat, detMat);              
    }

    std::vector<std::string> classNames;
    std::set<int>            relevantClasses;
    float                    detConf;

    if (usingCustomModel) {
        classNames      = {"person", "car", "bus", "motorcycle", "bicycle", "traffic light", "stop sign"};
        relevantClasses = {0, 1, 2, 3, 4, 5, 6};
        detConf         = 0.5f;
    } else {
        classNames = {
            "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
            "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
            "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
            "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
            "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
            "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
            "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake",
            "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop",
            "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
            "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
        };
        relevantClasses = {0, 1, 2, 3, 5, 7, 9, 11};
        detConf         = 0.01f;
    }

    std::vector<cv::Rect> boxes;
    std::vector<float>    confidences;
    std::vector<int>      classIds;
    for (int i = 0; i < detMat.rows; ++i) {
        const float* data = detMat.ptr<float>(i);
        cv::Mat scores(1, detMat.cols - 4, CV_32F, const_cast<float*>(data + 4));
        cv::Point classIdPoint;
        double maxScore;
        cv::minMaxLoc(scores, 0, &maxScore, 0, &classIdPoint);
        if (maxScore > detConf && relevantClasses.count(classIdPoint.x) > 0) {
            float cx = data[0] * (input.cols / 640.0f);
            float cy = data[1] * (input.rows / 640.0f);
            float w  = data[2] * (input.cols / 640.0f);
            float h  = data[3] * (input.rows / 640.0f);

            if (should_log)
                std::cout << "[CAMERA][YOLO][RAW] classId=" << classIdPoint.x
                          << " confidence=" << maxScore
                          << " box=(" << cx << "," << cy << "," << w << "," << h << ")"
                          << std::endl;

            int left = static_cast<int>(cx - w / 2.0f);
            int top  = static_cast<int>(cy - h / 2.0f);
            if (w < 10 || h < 10) continue;    
            if (w > input.cols * 0.95f) continue;     
            if (h > input.rows * 0.95f) continue;
            boxes.push_back(cv::Rect(left, top, static_cast<int>(w), static_cast<int>(h)));
            confidences.push_back(static_cast<float>(maxScore));
            classIds.push_back(classIdPoint.x);
        }
    }

    if (should_log)
        std::cout << "[CAMERA][YOLO] detections after confidence filter=" << boxes.size() << std::endl;

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, detConf, 0.5f, indices);

    if (should_log)
        std::cout << "[CAMERA][YOLO] detections after NMS=" << indices.size() << std::endl;
    for (int idx : indices) {
         DetectedItem item;
         item.box = boxes[idx];
         item.confidence = confidences[idx];
         item.label = classNames[classIds[idx]];
         results.push_back(item);
    }

    if (should_log)
        std::cout << "[CAMERA][YOLO] final camera objects=" << results.size() << std::endl;

    return results;
}

CameraObject CameraProcessing::createObject(const DetectedItem& raw_detection) {
    CameraObject co;  
    co.type_label = raw_detection.label; 
    co.timestamp = this->ts;
    float centroid_x = raw_detection.box.x + (raw_detection.box.width / 2.0f);
    float centroid_y = raw_detection.box.y + (raw_detection.box.height / 2.0f);
    co.position_2d = cv::Point2f(centroid_x, centroid_y); 
    double detection_noise = 1.0 - raw_detection.confidence;
    co.confidence = std::max(0.1, 1.0 - (detection_noise / 1.5)); 
    co.width  = calculatePhysicalWidth(raw_detection.box);  
    co.height = calculatePhysicalHeight(raw_detection.box); 
    co.length = calculatePhysicalLength(co.width, co.height);
    cv::Point3f p3d = transformTo3D(co.position_2d, raw_detection.box);
    if (p3d.x < -9000.0f) {
        co.isValid = false;
        return co;
    }
    co.position_3d.x() = p3d.x;
    co.position_3d.y() = p3d.y;
    co.position_3d.z() = p3d.z;
    co.isValid = (co.confidence > 0.4);
    co.bounding_box = raw_detection.box;
    if (co.type_label == "traffic light") {
    co.traffic_light_color = detect(currentFrame, co.bounding_box, static_cast<float>(co.confidence));
    }
    CameraData cd;
    cd.process(co);
        return co;
}

float CameraProcessing::getDepthZ(const cv::Rect& box) {
    float fx = K(0, 0);
    float fy = K(1, 1);
    float cx = K(0, 2);
    float cy = K(1, 2);
    float pixel_bottom_x = static_cast<float>(box.x) + static_cast<float>(box.width) / 2.0f;
    float pixel_bottom_y = static_cast<float>(box.y + box.height);
    Eigen::Vector3f ray_cam(
        (pixel_bottom_x - cx) / fx,
        (pixel_bottom_y - cy) / fy,
        1.0f
    );

    auto config = ConfigManager::getCamera("rgb_camera"); 
    Eigen::Matrix3f R = config.rotation; 
    Eigen::Vector3f ray_world = R * ray_cam;
    if (ray_world.y() < 0.01f) {
        return -1.0f;
    }

    float Z_camera = Config::camera_height / ray_world.y();

    if (Z_camera > 50.0f) return -1.0f;
    return Z_camera;
}


float CameraProcessing::calculatePhysicalHeight(const cv::Rect& box) {
    float Z = getDepthZ(box);
    if (Z < 0) return 1.7f;

    float fy = K(1, 1);
    return (box.height * Z) / fy;
}

float CameraProcessing::calculatePhysicalWidth(const cv::Rect& box) {
    float Z = getDepthZ(box);
    if (Z < 0) return 1.8f; 

    float fx = K(0, 0);
    return (box.width * Z) / fx;
}
float CameraProcessing::calculatePhysicalLength(float physical_width, float physical_height) {
    if (physical_width <= 0 || physical_height <= 0) {
        return 2.0f; 
    }
    float aspect_ratio = physical_width / physical_height;
    float multiplier = aspect_ratio * 2.0f;
    if (multiplier < 0.5f) multiplier = 0.5f;
    if (multiplier > 3.0f) multiplier = 3.0f;
    float physical_length = physical_width * multiplier;

    return physical_length;
}    
cv::Point3f CameraProcessing::transformTo3D(cv::Point2f centroid, const cv::Rect& box) {
    float Z_cam = getDepthZ(box);
    if (Z_cam < 0) return cv::Point3f(-9999.0f, -9999.0f, -9999.0f);
    float fx = K(0, 0);
    float fy = K(1, 1);
    float cx = K(0, 2);
    float cy = K(1, 2);
    float pixel_center_x = box.x + (box.width / 2.0f);
    float pixel_center_y = box.y + (box.height / 2.0f);
    float x_norm = (pixel_center_x - cx) / fx;
    float y_norm = (pixel_center_y - cy) / fy;
    Eigen::Vector3f p_camera(
        x_norm * Z_cam,
        y_norm * Z_cam,
        Z_cam
    );
    auto config = ConfigManager::getCamera("rgb_camera"); 
    Eigen::Affine3f transform = Eigen::Translation3f(config.translation) * config.rotation;
    Eigen::Vector3f p_transformed = transform * p_camera;
    return cv::Point3f(p_transformed.x(), p_transformed.y(), p_transformed.z());
}

bool CameraProcessing::isBright(const cv::Rect& roi,
                                     const cv::Rect& safe_box) const
{
    int cx = roi.x - safe_box.x + roi.width  / 4;
    int cy = roi.y - safe_box.y + roi.height / 4;

    cv::Rect center_roi(cx, cy,
                        std::max(1, roi.width  / 2),
                        std::max(1, roi.height / 2));

    center_roi &= cv::Rect(0, 0, m_hsv.cols, m_hsv.rows);
    if (center_roi.empty()) return false;

    std::vector<cv::Mat> channels;
    cv::split(m_hsv(center_roi), channels);
    return cv::mean(channels[2])[0] > BRIGHTNESS_THRESHOLD;
}

int CameraProcessing::countColor(const cv::Rect& roi,
                                      const cv::Rect& safe_box,
                                      const cv::Scalar& lower,  const cv::Scalar& upper,
                                      const cv::Scalar& lower2, const cv::Scalar& upper2,
                                      float confidence)
{
    if (!isBright(roi, safe_box)) return 0;

    cv::Rect local_roi = {
        roi.x - safe_box.x,
        roi.y - safe_box.y,
        roi.width,
        roi.height
    };
    local_roi &= cv::Rect(0, 0, m_hsv.cols, m_hsv.rows);
    if (local_roi.empty()) return 0;

    cv::inRange(m_hsv(local_roi), lower, upper, m_mask);
    if (lower2[0] >= 0) {
        cv::inRange(m_hsv(local_roi), lower2, upper2, m_mask2);
        m_mask |= m_mask2;
    }

    cv::morphologyEx(m_mask, m_mask, cv::MORPH_OPEN, MORPH_KERNEL);
    return cv::countNonZero(m_mask);
}

std::string CameraProcessing::detect(const cv::Mat& frame,
                                          const cv::Rect& box,
                                          float           confidence)
{
    confidence = std::clamp(confidence, 0.1f, 1.0f);

    cv::Rect safe_box = box & cv::Rect(0, 0, frame.cols, frame.rows);
    if (safe_box.empty() || safe_box.area() < 10)
        return "unknown";

    cv::GaussianBlur(frame(safe_box), m_blurred, cv::Size(3, 3), 0);
    cv::cvtColor(m_blurred, m_hsv, cv::COLOR_BGR2HSV);

    auto threshold = [&](const cv::Rect& roi) -> int {
        return std::max(3, static_cast<int>((roi.area() / 25.0f) * confidence));
    };

    std::vector<ColorResult> results;

    if (safe_box.height >= MIN_HEIGHT_FOR_SPLIT)
    {
        int third_h = safe_box.height / 3;

        cv::Rect top_roi = { safe_box.x, safe_box.y,               safe_box.width, third_h };
        cv::Rect mid_roi = { safe_box.x, safe_box.y + third_h,     safe_box.width, third_h };
        cv::Rect bot_roi = { safe_box.x, safe_box.y + 2 * third_h, safe_box.width,
                             safe_box.height - 2 * third_h };

        results = {
            { "red",    countColor(top_roi, safe_box, RED_L1, RED_U1, RED_L2, RED_U2, confidence), threshold(top_roi) },
            { "yellow", countColor(mid_roi, safe_box, YEL_L,  YEL_U,  {-1,-1,-1}, {-1,-1,-1}, confidence), threshold(mid_roi) },
            { "green",  countColor(bot_roi, safe_box, GRN_L,  GRN_U,  {-1,-1,-1}, {-1,-1,-1}, confidence), threshold(bot_roi) }
        };
    }
    else
    {
        results = {
            { "red",    countColor(safe_box, safe_box, RED_L1, RED_U1, RED_L2, RED_U2, confidence), threshold(safe_box) },
            { "yellow", countColor(safe_box, safe_box, YEL_L,  YEL_U,  {-1,-1,-1}, {-1,-1,-1}, confidence), threshold(safe_box) },
            { "green",  countColor(safe_box, safe_box, GRN_L,  GRN_U,  {-1,-1,-1}, {-1,-1,-1}, confidence), threshold(safe_box) }
        };
    }

    ColorResult* dominant  = nullptr;
    ColorResult* runner_up = nullptr;

    for (auto& r : results)
        if (r.count > r.threshold) {
            if (!dominant || r.count > dominant->count) {
                runner_up = dominant;
                dominant  = &r;
            } else if (!runner_up || r.count > runner_up->count) {
                runner_up = &r;
            }
        }

    if (dominant && runner_up &&
        dominant->count < static_cast<int>(runner_up->count * MIN_DOMINANCE_RATIO))
        return "unknown";

    return dominant ? dominant->name : "unknown";
}