#include <iostream>
#include <vector>
#include <memory>
#include <thread>
#include <chrono>
#include <mutex>
#include <stdexcept>
#include <string>

#include "SensorConsumer.hpp"
#include "SensorProducerManager.hpp"
#include "SensorFrameManager.hpp"
#include "StatePriorityQueue.hpp"
#include "Constants.hpp"

#include "Imu\ImuProcessing.hpp"
#include "Encoder\EncoderProcessing.hpp"
#include "Lidar\LidarProcessing.hpp"
#include "Camera\CameraProcessing.hpp"
#include "Radar\RadarProcessing.hpp"
#include "Gps\GpsProcessing.hpp"

// ==========================================
// אבחון בלבד: עקבת עדכון EKF מלאה — state לפני, z, H*x, residual, אינדקסי H
// שאינם אפס, state אחרי, וה-timestamp של המשימה (לקישור לבעיית ה-dt).
// לא משנה שום לוגיקה — רק קריאה והדפסה.
// ==========================================
template <int N>
static void logEkfUpdateDebug(const char* task_type,
                               double task_ts,
                               const Eigen::VectorXd& state_before,
                               const Eigen::Matrix<double, N, 1>& Z,
                               const Eigen::Matrix<double, N, 1>& Hx,
                               const Eigen::Matrix<double, N, Config::DimWheelchairStateVector>& H,
                               const Eigen::VectorXd& state_after)
{
    static int log_count = 0;
    if ((++log_count % 10) != 1) return;

    std::vector<int> nonzero_idx;
    for (int c = 0; c < H.cols(); ++c) {
        if ((H.col(c).array() != 0.0).any()) nonzero_idx.push_back(c);
    }

    const Eigen::Matrix<double, N, 1> residual = Z - Hx;

    std::cout << "[EKF_UPDATE_DEBUG] task_type=" << task_type
              << " task_ts=" << task_ts
              << " state_before=[" << state_before.transpose() << "]"
              << " z=[" << Z.transpose() << "]"
              << " Hx=[" << Hx.transpose() << "]"
              << " residual=[" << residual.transpose() << "]"
              << " H_nonzero_idx=[";
    for (size_t i = 0; i < nonzero_idx.size(); ++i) {
        std::cout << nonzero_idx[i];
        if (i + 1 < nonzero_idx.size()) std::cout << ",";
    }
    std::cout << "]"
              << " state_after=[" << state_after.transpose() << "]"
              << std::endl;
}

void SensorConsumer::init() {
    std::cout << "[INIT] SensorConsumer::init() starting 6 consumers" << std::endl;
    worker_manager.startWorker([this]() { this->ConsumerImu(); });
    worker_manager.startWorker([this]() { this->ConsumerLidar(); });
    worker_manager.startWorker([this]() { this->ConsumerCamera(); });
    worker_manager.startWorker([this]() { this->ConsumerRadar(); });
    worker_manager.startWorker([this]() { this->ConsumerGps(); });
    worker_manager.startWorker([this]() { this->ConsumerEncoder(); });
    std::cout << "[INIT] SensorConsumer::init() all consumers started" << std::endl;
}

void SensorConsumer::stop() {
    std::cout << "[THREAD] SensorConsumer::stop() called" << std::endl;
    worker_manager.stop();
    std::cout << "[CAMERA][VIDEO] " << frameIndex << " frames saved to yolo_frames/" << std::endl;
    std::cout << "[THREAD] SensorConsumer::stop() complete" << std::endl;
}

// ==========================================
// Consumer IMU
// ==========================================
void SensorConsumer::ConsumerImu() {
    static int imu_count = 0;
    if (++imu_count % 50 == 0)
        std::cout << "[SENSOR] ConsumerImu running (iter=" << imu_count << ")" << std::endl;

    auto& producer_manager = SensorProducerManager::getInstance();
    auto& frame_manager = SensorFrameManager::getInstance();
    auto& state_queue = StatePriorityQueue::getInstance();
    static thread_local ImuProcessing imu;
    try {
        auto last_measurements = producer_manager.imuRing.popLastNWithTimeout(1, std::chrono::milliseconds(5));
        if (last_measurements.empty()) return;

        auto raw_data = last_measurements.back().data;
        double current_timestamp = last_measurements.back().timestamp;
        if (current_timestamp < 0.0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            return;
        }

        if (current_timestamp < 0.0) {
            std::cout << "[Consumer][IMU] invalid timestamp, skipping: " << current_timestamp << std::endl;
            return;
        }

        ImuObject processed_object = imu.process(raw_data,current_timestamp);

        std::shared_ptr<FramePointers> matched_frame = frame_manager.getOrCreateFrame(current_timestamp);

        if (matched_frame) {
            std::lock_guard<std::mutex> lock(matched_frame->frame_mutex);
            matched_frame->imu->list.push_back(processed_object);
            int sz = static_cast<int>(matched_frame->imu->list.size());
            if (sz == 1 || sz % 10 == 0) {
                long long bkt = static_cast<long long>(std::floor(current_timestamp / 0.5));
            }
        }

        if (current_timestamp < 0.0) {
            std::cout << "[Consumer][IMU] invalid timestamp, skipping: " << current_timestamp << std::endl;
            return;
        }

        StateUpdateTask task;
        task.priority = UpdatePriority::HIGH;
        task.timestamp = current_timestamp;
        task.source = "IMU";

        task.execute = [processed_object, current_timestamp](EKF<Config::DimWheelchairStateVector>& ekf)
        {
            auto x_current = ekf.getState();

            Eigen::Matrix<double, Config::MeasurementSizeImu, Config::DimWheelchairStateVector> H =
                processed_object.H.cast<double>();

            Eigen::Matrix<double, Config::MeasurementSizeImu, 1> Z =
                processed_object.Z.cast<double>();

            Eigen::Matrix<double, Config::MeasurementSizeImu, Config::MeasurementSizeImu> R =
                processed_object.R.cast<double>();

            // Validation
            const int z_rows = Z.rows();
            const int h_rows = H.rows();
            const int h_cols = H.cols();
            const int r_rows = R.rows();
            const int state_rows = ekf.getState().rows();

            if (!(z_rows == h_rows && r_rows == z_rows && h_cols == state_rows && state_rows == Config::DimWheelchairStateVector)) {
                std::cout << "[EKF][IMU] Invalid dims Z/H/R/state: Z=" << z_rows << " H=(" << h_rows << "," << h_cols << ") R=" << r_rows << " state=" << state_rows << std::endl;
                return;
            }

            Eigen::Matrix<double, Config::MeasurementSizeImu, 1> Z_pred =
                H * x_current;

            ekf.update<Config::MeasurementSizeImu>(
                Z,
                H,
                R,
                Z_pred
            );

            logEkfUpdateDebug<Config::MeasurementSizeImu>("IMU", current_timestamp, x_current, Z, Z_pred, H, ekf.getState());
        };

        state_queue.push(std::move(task));

    } catch (const std::exception& e) {
        std::cerr << "[ERROR] ConsumerImu crashed: " << e.what() << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

// ==========================================
// Consumer Encoder
// ==========================================
void SensorConsumer::ConsumerEncoder() {
    static int enc_count = 0;
    if (++enc_count % 50 == 0)
        std::cout << "[SENSOR] ConsumerEncoder running (iter=" << enc_count << ")" << std::endl;

    auto& producer_manager = SensorProducerManager::getInstance();
    auto& frame_manager = SensorFrameManager::getInstance();
    auto& state_queue = StatePriorityQueue::getInstance();
    static thread_local EncoderProcessing encoder;
    try {
        auto last_measurements = producer_manager.encoderRing.popLastNWithTimeout(1, std::chrono::milliseconds(5));
        if (last_measurements.empty()) return;

        auto raw_data = last_measurements.back().data;
        double current_timestamp = last_measurements.back().timestamp;

        if (current_timestamp < 0.0) {
            std::cout << "[Consumer][ENCODER] invalid timestamp, skipping: " << current_timestamp << std::endl;
            return;
        }

        EncoderObject processed_object = encoder.process(raw_data, current_timestamp);

        std::shared_ptr<FramePointers> matched_frame = frame_manager.getOrCreateFrame(current_timestamp);

        if (matched_frame) {
            std::lock_guard<std::mutex> lock(matched_frame->frame_mutex);
            matched_frame->encoder->list.push_back(processed_object);
            int sz = static_cast<int>(matched_frame->encoder->list.size());
            if (sz == 1 || sz % 10 == 0) {
                long long bkt = static_cast<long long>(std::floor(current_timestamp / 0.5));
            }
        }

        StateUpdateTask task;
        task.priority = UpdatePriority::MEDIUM;
        task.timestamp = current_timestamp;
        task.source = "ENCODER";
        task.execute = [processed_object, current_timestamp](EKF<Config::DimWheelchairStateVector>& ekf)
        {
            auto x_current = ekf.getState();

            Eigen::Matrix<double, Config::MeasurementSizeEncoder, Config::DimWheelchairStateVector> H =
                processed_object.H.cast<double>();

            Eigen::Matrix<double, Config::MeasurementSizeEncoder, 1> Z =
                processed_object.Z.cast<double>();

            Eigen::Matrix<double, Config::MeasurementSizeEncoder, Config::MeasurementSizeEncoder> R =
                processed_object.R.cast<double>();

            // Validation
            const int z_rows = Z.rows();
            const int h_rows = H.rows();
            const int h_cols = H.cols();
            const int r_rows = R.rows();
            const int state_rows = ekf.getState().rows();

            if (!(z_rows == h_rows && r_rows == z_rows && h_cols == state_rows && state_rows == Config::DimWheelchairStateVector)) {
                std::cout << "[EKF][ENCODER] Invalid dims Z/H/R/state: Z=" << z_rows << " H=(" << h_rows << "," << h_cols << ") R=" << r_rows << " state=" << state_rows << std::endl;
                return;
            }

            Eigen::Matrix<double, Config::MeasurementSizeEncoder, 1> Z_pred =
                H * x_current;

            {
                static int enc_r_log_count = 0;
                if ((++enc_r_log_count % 10) == 1) {
                    Eigen::Matrix<double, Config::MeasurementSizeEncoder, 1> residual_dbg = Z - Z_pred;
                    std::cout << "[ENCODER_R_DEBUG]"
                              << " confidence=" << processed_object.confidence
                              << " R00=" << R(0, 0)
                              << " R11=" << R(1, 1)
                              << " z=[" << Z.transpose() << "]"
                              << " Hx=[" << Z_pred.transpose() << "]"
                              << " residual=[" << residual_dbg.transpose() << "]"
                              << std::endl;
                }
            }

            ekf.update<Config::MeasurementSizeEncoder>(
                Z,
                H,
                R,
                Z_pred
            );

            logEkfUpdateDebug<Config::MeasurementSizeEncoder>("ENCODER", current_timestamp, x_current, Z, Z_pred, H, ekf.getState());
        };

        state_queue.push(std::move(task));

    } catch (const std::exception& e) {
        std::cerr << "[ERROR] ConsumerEncoder crashed: " << e.what() << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

// ==========================================
// Consumer Lidar
// ==========================================
void SensorConsumer::ConsumerLidar() {
    static int lidar_count = 0;
    if (++lidar_count % 50 == 0)
        std::cout << "[SENSOR] ConsumerLidar running (iter=" << lidar_count << ")" << std::endl;

    auto& producer_manager = SensorProducerManager::getInstance();
    auto& frame_manager = SensorFrameManager::getInstance();
    static thread_local LidarProcessing lidar;

    try {
        auto last_measurements = producer_manager.lidarRing.popLastNWithTimeout(3, std::chrono::milliseconds(10));
        if (last_measurements.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            return;
        }

        double current_timestamp = last_measurements.back().timestamp;

        if (lidar_count % 50 == 0)
            std::cout << "[SENSOR] ConsumerLidar received " << last_measurements.size()
                      << " clouds ts=" << current_timestamp << std::endl;

        if (current_timestamp < 0.0) {
            std::cerr << "[WARN] ConsumerLidar: invalid timestamp=" << current_timestamp << ", skipping" << std::endl;
            return;
        }

        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> raw_clouds;
        for(const auto& m : last_measurements) {
            raw_clouds.push_back(m.data);
        }

        size_t total_cloud_pts = 0;
        for (const auto& c : raw_clouds) if (c) total_cloud_pts += c->size();

        std::vector<LidarObject> processed_objects = lidar.process(raw_clouds,current_timestamp);
        auto lidar_list = std::make_shared<LidarList>();
        for (const auto& obj : processed_objects) {
            lidar_list->list.push_back(obj);
        }
        auto gc = lidar.getGlobalCloud();
        if (gc && !gc->empty())
            lidar_list->global_cloud = gc;

        std::shared_ptr<FramePointers> matched_frame = frame_manager.getOrCreateFrame(current_timestamp);

        if (matched_frame) {
            std::lock_guard<std::mutex> lock(matched_frame->frame_mutex);
            matched_frame->lidar = lidar_list;
            long long bkt = static_cast<long long>(std::floor(current_timestamp / 0.5));
            std::cout << "[FRAME][ADD] sensor=LIDAR bucket=" << bkt
                      << " ts=" << current_timestamp
                      << " lidar_objects=" << lidar_list->list.size()
                      << " cloud_points=" << total_cloud_pts << std::endl;
        }

    } catch (const std::exception& e) {
        std::cerr << "[ERROR] ConsumerLidar crashed: " << e.what() << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

// ==========================================
// Consumer Camera
// ==========================================
void SensorConsumer::ConsumerCamera() {
    static int cam_count = 0;
    if (++cam_count % 50 == 0)
        std::cout << "[SENSOR] ConsumerCamera running (iter=" << cam_count << ")" << std::endl;

    auto& producer_manager = SensorProducerManager::getInstance();
    auto& frame_manager = SensorFrameManager::getInstance();
    static thread_local CameraProcessing camera;

    try
    {
        auto last_measurements = producer_manager.cameraRing.popLastNWithTimeout(1, std::chrono::milliseconds(10));
        if (last_measurements.empty()) return;
        auto measurement = last_measurements.back();

        double current_timestamp = measurement.timestamp;
        if (current_timestamp < 0.0) return;
        std::cout << "[Consumer][CAMERA] processing frame t=" << current_timestamp << ", calling detector..." << std::endl;

        cv::Mat raw_frame = measurement.data;

        std::vector<CameraObject> processed_objects = camera.process(raw_frame, current_timestamp);

        // --- YOLO Frame Recording (JPEG per frame — reliable even on abrupt shutdown) ---
        if (!raw_frame.empty()) {
            cv::Mat annotated = raw_frame.clone();

            for (const auto& obj : processed_objects) {
                cv::Scalar color(0, 220, 0);
                if (obj.type_label == "person")
                    color = cv::Scalar(255, 100, 0);
                else if (obj.type_label == "car" || obj.type_label == "truck" || obj.type_label == "bus")
                    color = cv::Scalar(0, 60, 255);
                else if (obj.type_label == "traffic light")
                    color = cv::Scalar(0, 230, 230);

                cv::rectangle(annotated, obj.bounding_box, color, 2);

                std::string lbl = obj.type_label;
                if (obj.type_label == "traffic light" &&
                    !obj.traffic_light_color.empty() &&
                    obj.traffic_light_color != "unknown")
                    lbl += " [" + obj.traffic_light_color + "]";
                char conf_buf[32];
                snprintf(conf_buf, sizeof(conf_buf), " %.0f%%", obj.confidence * 100.0f);
                lbl += conf_buf;

                int baseline = 0;
                cv::Size ts = cv::getTextSize(lbl, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
                int ty = std::max(obj.bounding_box.y - 5, ts.height + 5);
                cv::rectangle(annotated,
                              cv::Point(obj.bounding_box.x, ty - ts.height - 2),
                              cv::Point(obj.bounding_box.x + ts.width, ty + baseline),
                              color, cv::FILLED);
                cv::putText(annotated, lbl,
                            cv::Point(obj.bounding_box.x, ty),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);

                char dist_buf[48];
                snprintf(dist_buf, sizeof(dist_buf), "(%.1fm, %.1fm)",
                         obj.position_3d.x(), obj.position_3d.z());
                cv::putText(annotated, dist_buf,
                            cv::Point(obj.bounding_box.x,
                                      obj.bounding_box.y + obj.bounding_box.height + 14),
                            cv::FONT_HERSHEY_SIMPLEX, 0.38, color, 1);
            }

            char hud[64];
            snprintf(hud, sizeof(hud), "Frame %-4d  t=%.1fs  objects:%d",
                     frameIndex, current_timestamp, (int)processed_objects.size());
            cv::Size hudSize = cv::getTextSize(hud, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, nullptr);
            cv::rectangle(annotated, cv::Point(0,0),
                          cv::Point(hudSize.width + 16, hudSize.height + 12),
                          cv::Scalar(0,0,0), cv::FILLED);
            cv::putText(annotated, hud, cv::Point(8, hudSize.height + 6),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255,255,255), 2);

            char path[256];
            snprintf(path, sizeof(path),
                     "C:/Users/User/Desktop/TrackObject/yolo_frames/frame_%05d.jpg",
                     frameIndex);
            cv::imwrite(path, annotated, {cv::IMWRITE_JPEG_QUALITY, 92});
            frameIndex++;
        }
        // --- End Frame Recording ---

        auto camera_list = std::make_shared<CameraList>();

        for (const auto& obj : processed_objects)
        {
            camera_list->list.push_back(obj);
        }
        std::cout << "[Consumer][CAMERA] detected " << processed_objects.size() << " objects t=" << current_timestamp << std::endl;
        std::cout << "[OBSTACLE_SENSOR_DEBUG] camera_objects=" << processed_objects.size() << std::endl;

        double frame_ts = current_timestamp;
        auto matched_frame = frame_manager.getOrCreateFrame(frame_ts);
        if (matched_frame) {
            bool already_processed;
            {
                std::lock_guard<std::mutex> check(matched_frame->frame_mutex);
                already_processed = matched_frame->is_processed;
            }
            if (already_processed) {
                frame_ts = current_timestamp + 0.5;
                matched_frame = frame_manager.getOrCreateFrame(frame_ts);
            }
        }
        if (matched_frame)
        {
            std::lock_guard<std::mutex> lock(matched_frame->frame_mutex);
            matched_frame->camera = camera_list;
            long long bkt = static_cast<long long>(std::floor(frame_ts / 0.5));
            std::cout << "[FRAME][ADD] sensor=CAMERA bucket=" << bkt
                      << " ts=" << frame_ts
                      << " camera_objects=" << camera_list->list.size() << std::endl;
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "[ERROR] ConsumerCamera crashed: " << e.what() << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

// ==========================================
// Consumer Radar
// ==========================================
void SensorConsumer::ConsumerRadar() {
    static int radar_count = 0;
    if (++radar_count % 50 == 0)
        std::cout << "[SENSOR] ConsumerRadar running (iter=" << radar_count << ")" << std::endl;

    auto& producer_manager = SensorProducerManager::getInstance();
    auto& frame_manager = SensorFrameManager::getInstance();
    static thread_local RadarProcessing radar;

    try {
        auto last_measurements = producer_manager.radarRing.popLastNWithTimeout(1, std::chrono::milliseconds(10));
        if (last_measurements.empty()) return;

        double current_timestamp = last_measurements.back().timestamp;
        if (current_timestamp < 0.0) return;
        std::cout << "[Consumer][RADAR] processing t=" << current_timestamp << std::endl;
        auto raw_data = last_measurements.back().data;

        std::vector<RadarObject> processed_objects = radar.process(raw_data,current_timestamp);
        std::cout << "[OBSTACLE_SENSOR_DEBUG] radar_objects=" << processed_objects.size() << std::endl;

        auto radar_list = std::make_shared<RadarList>();
        for (const auto& obj : processed_objects) {
            radar_list->list.push_back(obj);
        }

        std::shared_ptr<FramePointers> matched_frame = frame_manager.getOrCreateFrame(current_timestamp);

        if (matched_frame) {
            std::lock_guard<std::mutex> lock(matched_frame->frame_mutex);
            matched_frame->radar = radar_list;
            long long bkt = static_cast<long long>(std::floor(current_timestamp / 0.5));
            std::cout << "[FRAME][ADD] sensor=RADAR bucket=" << bkt
                      << " ts=" << current_timestamp
                      << " radar_objects=" << radar_list->list.size() << std::endl;
        }

    } catch (const std::exception& e) {
        std::cerr << "[ERROR] ConsumerRadar crashed: " << e.what() << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

// ==========================================
// Consumer GPS
// ==========================================
void SensorConsumer::ConsumerGps() {
    static int gps_count = 0;
    if (++gps_count % 50 == 0)
        std::cout << "[SENSOR] ConsumerGps running (iter=" << gps_count << ")" << std::endl;

    auto& producer_manager = SensorProducerManager::getInstance();
    auto& frame_manager = SensorFrameManager::getInstance();
    static thread_local GpsProcessing gps;

    try {
        auto last_measurements = producer_manager.gpsRing.popLastNWithTimeout(1, std::chrono::milliseconds(10));
        if (last_measurements.empty()) return;

        double current_timestamp = last_measurements.back().timestamp;
        if (current_timestamp < 0.0) {
            std::cerr << "[WARN] ConsumerGps: invalid timestamp=" << current_timestamp << ", skipping" << std::endl;
            return;
        }
        auto raw_sentence = last_measurements.back().data;
        std::cout << "[Consumer][GPS] processing t=" << current_timestamp << std::endl;

        std::stringstream ss(raw_sentence); // הנחה ש-raw_sentence היא מחרוזת
        GpsObject processed_object = gps.process(ss, current_timestamp);

        // בדיקת פורמט DDMM.MMMM: אם ערכי Z גדולים מדי — ייתכן שה-GPS לא הומר לדציאמל-דגרי
        if (processed_object.Z.size() >= 2) {
            double z0 = std::abs(processed_object.Z(0));
            double z1 = std::abs(processed_object.Z(1));
            if (z0 > 100000.0 || z1 > 100000.0) {
                std::cerr << "[WARN] ConsumerGps: GPS position values suspiciously large"
                          << " Z(0)=" << processed_object.Z(0)
                          << " Z(1)=" << processed_object.Z(1)
                          << " - possible DDMM.MMMM format not converted to decimal degrees" << std::endl;
            }
        }

        auto gps_list = std::make_shared<GpsObject>(processed_object);

        std::shared_ptr<FramePointers> matched_frame = frame_manager.getOrCreateFrame(current_timestamp);

        if (matched_frame) {
            std::lock_guard<std::mutex> lock(matched_frame->frame_mutex);
            matched_frame->gps = gps_list;
            long long bkt = static_cast<long long>(std::floor(current_timestamp / 0.5));
            std::cout << "[FRAME][ADD] sensor=GPS bucket=" << bkt
                      << " ts=" << current_timestamp
                      << " has_gps=" << matched_frame->gps->isValid << std::endl;
        }

    } catch (const std::exception& e) {
        std::cerr << "[ERROR] ConsumerGps crashed: " << e.what() << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}
