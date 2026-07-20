#pragma once
#include <iostream>
#include <string>
#include <vector>
#include "SensorList.hpp"
#include <opencv2/opencv.hpp>
#include "CameraObject.hpp"

struct CameraList: public SensorList<CameraObject> {
   cv::Mat img;
};
