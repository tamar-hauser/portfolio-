#pragma once

#include <vector>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#include "CameraObject.hpp"
#include "SensorObject.hpp"
#include "CameraData.hpp"
#include "SensorProcessing.hpp"

struct DetectedItem {
    std::string label;     
    float confidence;     
    cv::Rect box;         
};

class CameraProcessing : public SensorProcessing<cv::Mat, std::vector<CameraObject>> {
public:
    CameraProcessing(); 
    std::vector<CameraObject> process(cv::Mat& input, double ts) override;

protected:
    std::vector<DetectedItem> yolo(const cv::Mat& input);
    CameraObject createObject(const DetectedItem& raw_detection);  
    float calculatePhysicalLength(float physical_width, float physical_height);
    float calculatePhysicalWidth(const cv::Rect& box);
    float calculatePhysicalHeight(const cv::Rect& box);
    cv::Point3f transformTo3D(cv::Point2f centroid, const cv::Rect& box);
    float getDepthZ(const cv::Rect& box);
    std::string detect(const cv::Mat& frame,const cv::Rect& box,float confidence = 1.0f);
private:
    cv::Mat currentFrame;
    Eigen::Matrix3f K;
    std::string modelPath     = "C:/Users/User/Desktop/TrackObject/build/Debug/yolov8n.onnx";
    bool        usingCustomModel = false;
    cv::dnn::Net net;
    const cv::Mat MORPH_KERNEL =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));

    const cv::Scalar RED_L1{0,   60, 60}, RED_U1{15,  255, 255};
    const cv::Scalar RED_L2{150, 60, 60}, RED_U2{180, 255, 255};
    const cv::Scalar YEL_L {15,  60, 60}, YEL_U {45,  255, 255};
    const cv::Scalar GRN_L {35,  60, 60}, GRN_U {100, 255, 255};

    static constexpr int   MIN_HEIGHT_FOR_SPLIT = 30;
    static constexpr float MIN_DOMINANCE_RATIO  = 1.5f;
    static constexpr float BRIGHTNESS_THRESHOLD = 100.0f;

    // מטריצות ממוחזרות — חבר של הקלאס, לא גלובלי
    cv::Mat m_blurred, m_hsv, m_mask, m_mask2;

    struct ColorResult { std::string name; int count; int threshold; };

    bool isBright  (const cv::Rect& roi, const cv::Rect& safe_box) const;
    int  countColor(const cv::Rect& roi, const cv::Rect& safe_box,
                    const cv::Scalar& lower,  const cv::Scalar& upper,
                    const cv::Scalar& lower2, const cv::Scalar& upper2,
                    float confidence);
};