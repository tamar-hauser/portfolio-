#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>
#include <webots/accelerometer.h>
#include <webots/gyro.h>
#include <webots/lidar.h>
#include <webots/camera.h>
#include <webots/position_sensor.h>
#include <webots/types.h>
#include <webots/supervisor.h>

#include <Eigen/Dense>
#include <opencv2/core/utils/logger.hpp>
#include <atomic>
#include <cmath>
#include <csignal>
#include <iostream>
#include <memory>
#include <sstream>
#include <thread>
#include <utility>

#include "Thread/SensorConsumer.hpp"
#include "Thread/ThreadGrafFector.hpp"
#include "Thread/StateThread.hpp"
#include "Thread/NavigationManager.hpp"
#include "Thread/StatePriorityQueue.hpp"
#include "Thread/SensorFrameManager.hpp"
#include "Thread/FactorGraphManager.hpp"
#include "Thread/GpsPoseStore.hpp"

#include "MapLive/Move.hpp"

#include "WebotsBridge/WebotsDrive.hpp"
#include "WebotsBridge/SensorProducerWebots.hpp"
#include "WebotsBridge/FrontLidarState.hpp"
#include "Gps/GpsProcessing.hpp"
#include "Thread/SensorProducerManager.hpp"
#include "Sensor/SensorConfig.hpp"
#include "Lidar/LidarCloud.hpp"

std::atomic<bool> keep_running(true);

// מסמנת את הנתיב הגלובלי בסימולציה: כדורות צבעוניות על כל waypoint
// כחול=התחלה, ירוק=סיום, אדום=ביניים + קו לבן בין נקודות עוקבות
static void visualizeGlobalPath(NavigationManager& nav)
{
    auto positions = nav.getPathPositionsLocal();
    if (positions.empty()) {
        std::cerr << "[VIZ] No path positions to visualize\n";
        return;
    }

    // קבלת עמדת הרובוט בעולם Webots (בקואורדינטות עולם, לפני תנועה)
    WbNodeRef robot_node = wb_supervisor_node_get_from_def("WHEELCHAIR");
    if (!robot_node) {
        std::cerr << "[VIZ] Cannot find WHEELCHAIR node\n";
        return;
    }
    const double* rpos = wb_supervisor_node_get_position(robot_node);
    // rpos = (x_wbt, y_wbt, z_wbt) — רובוט בנקודת EKF (0,0)
    // ב-Webots: x=East, y=North (תואם את ההמרה ב-SensorProducerWebots::makeNmea,
    // ששם north_m = v[1] בלי היפוך סימן)
    // local EKF: local_x=East, local_y=North
    // המרה: wbt_x = rpos[0] + local_x
    //        wbt_y = rpos[1] + local_y
    //        wbt_z = 0.2  (מעל הקרקע לנראות)
    const double ox = rpos[0];
    const double oy = rpos[1];
    std::cout << "[VIZ_ANCHOR_DEBUG] ox=" << ox << " oy=" << oy << " oz=" << rpos[2] << std::endl;

    WbNodeRef root = wb_supervisor_node_get_root();
    WbFieldRef children = wb_supervisor_node_get_field(root, "children");

    for (std::size_t i = 0; i < positions.size(); ++i) {
        double wx = ox + positions[i].first;
        double wy = oy + positions[i].second;
        double wz = 0.2;
        std::cout << "[VIZ_POINT_DEBUG] i=" << i
                  << " local=(" << positions[i].first << "," << positions[i].second << ")"
                  << " world=(" << wx << "," << wy << ")" << std::endl;

        // צבע: כחול=התחלה, ירוק=סיום, אדום=ביניים
        double r = 1.0, g = 0.0, b = 0.0;
        if (i == 0)                      { r=0; g=0; b=1; }  // כחול
        else if (i == positions.size()-1){ r=0; g=1; b=0; }  // ירוק

        char node_str[512];
        std::snprintf(node_str, sizeof(node_str),
            "Transform { translation %f %f %f "
            "children [ Shape { appearance Appearance { material Material { "
            "diffuseColor %f %f %f emissiveColor %f %f %f } } "
            "geometry Sphere { radius 0.4 } } ] }",
            wx, wy, wz, r, g, b, r, g, b);
        wb_supervisor_field_import_mf_node_from_string(children, -1, node_str);
    }

    // קו לבן בין נקודות עוקבות
    if (positions.size() >= 2) {
        std::string coord_str;
        for (const auto& p : positions) {
            double wx = ox + p.first;
            double wy = oy + p.second;
            char buf[64];
            std::snprintf(buf, sizeof(buf), "%f %f 0.15 ", wx, wy);
            coord_str += buf;
        }
        char line_node[4096];
        std::snprintf(line_node, sizeof(line_node),
            "Shape { appearance Appearance { material Material { "
            "emissiveColor 1 1 1 } } "
            "geometry IndexedLineSet { coord Coordinate { point [ %s ] } "
            "coordIndex [ ",
            coord_str.c_str());
        std::string idx_str = line_node;
        for (std::size_t i = 0; i < positions.size(); ++i) {
            idx_str += std::to_string(i);
            if (i + 1 < positions.size()) idx_str += " ";
        }
        idx_str += " -1 ] } }";
        wb_supervisor_field_import_mf_node_from_string(children, -1, idx_str.c_str());
    }

    std::cout << "[VIZ] Global path visualized: " << positions.size()
              << " waypoints (blue=start, red=intermediate, green=end)\n";
}

// מציג את המסדרון הגלובלי של הקשת הנוכחית בלבד (לא כל המסלול) כשני "קירות"
// וירטואליים — רצועה בכל צד של הקו בין שני קצות הקשת, במרחק חצי-רוחב
// המסדרון האמיתי (אותה לוגיקה כמו MapLocal::buildRouteLimitLayer, רק נקרא
// ממנה, לא שינוי). נשלח ל-Webots בקריאת IPC אחת. נקרא פעם אחת בכל קשת.
// Draws a color-coded local costmap window around the robot onto the Webots floor.
// Red = LETHAL, orange = inflated, yellow = low cost. One batched IPC call, throttled to 1.5s.
// After the OctomapAdapter fix: lidarLayer obstacles are in GPS world frame.
// shiftOrigin also uses GPS. gridToWorld() therefore returns GPS world coords directly.
// → center window on GPS position; no frame shift needed for VRML placement.
// Shows only lidarLayer + objectLayer obstacles — NOT routeLimitLayer.
static void visualizeCostMap(NavigationManager& nav,
                              const Eigen::VectorXd& /*raw_state*/,
                              const Eigen::VectorXd& nav_state,
                              double simTime)
{
    static double    lastVizTime = -1.0;
    static WbNodeRef lastNode    = nullptr;

    if (simTime - lastVizTime < 1.5) return;

    auto mapData = nav.getMapData();
    if (!mapData) return;

    const int IDX_X = static_cast<int>(Config::StateMembersRobot::StateX);
    const int IDX_Y = static_cast<int>(Config::StateMembersRobot::StateY);

    // Grid is GPS-centered (shiftOrigin uses GPS). Use GPS to find robot cell.
    const float gps_x = static_cast<float>(nav_state(IDX_X));
    const float gps_y = static_cast<float>(nav_state(IDX_Y));

    int robot_col, robot_row;
    if (!mapData->worldToGrid(gps_x, gps_y, robot_col, robot_row)) return;

    constexpr int WINDOW_R = 20; // ±4m at RESOLUTION=0.2m → 41×41 = 1681 cells max

    // Webots world position of the robot (supervisor API).
    // gridToWorld gives GPS-local coords; Webots world has a different origin.
    // We use the robot's Webots position as anchor and compute cell offsets from it.
    WbNodeRef viz_robot_node = wb_supervisor_node_get_from_def("WHEELCHAIR");
    if (!viz_robot_node) return;
    const double* viz_rpos = wb_supervisor_node_get_position(viz_robot_node);
    const double wbt_ox = viz_rpos[0];
    const double wbt_oy = viz_rpos[1];

    std::string node_str;
    node_str.reserve(300 * 500);
    node_str = "Transform { children [ ";

    char buf[256];
    int drawn = 0;
    for (int dr = -WINDOW_R; dr <= WINDOW_R; ++dr) {
        int row = robot_row + dr;
        if (row < 0 || row >= GRID_H) continue;
        for (int dc = -WINDOW_R; dc <= WINDOW_R; ++dc) {
            int col = robot_col + dc;
            if (col < 0 || col >= GRID_W) continue;

            const int idx = mapData->flatIdx(row, col);
            const uint8_t lc = mapData->lidarLayer[idx];
            const uint8_t oc = mapData->objectLayer[idx];

            // Skip cells with no sensor-detected obstacle (ignore routeLimitLayer)
            const bool lidar_hit  = (lc != COST_UNKNOWN && lc != COST_FREE);
            const bool object_hit = (oc != COST_UNKNOWN && oc != COST_FREE);
            if (!lidar_hit && !object_hit) continue;

            // Place tile in Webots world: robot Webots pos + cell offset from robot cell.
            const float wx = static_cast<float>(wbt_ox) + dc * RESOLUTION;
            const float wy = static_cast<float>(wbt_oy) + dr * RESOLUTION;

            const char* color;
            if      (lc == COST_LETHAL || oc == COST_LETHAL) color = "1 0.1 0.1"; // red   — lethal
            else if (lc >= COST_INFLATED)                     color = "1 0.5 0";   // orange — inflated
            else                                              color = "0.8 0.8 0"; // yellow — low cost

            std::snprintf(buf, sizeof(buf),
                "Pose { translation %.3f %.3f 0.005 children [ Shape { "
                "appearance Appearance { material Material { "
                "diffuseColor %s emissiveColor %s } } "
                "geometry Box { size 0.18 0.18 0.01 } } ] } ",
                wx, wy, color, color);
            node_str += buf;
            ++drawn;
        }
    }
    node_str += " ] }";

    if (drawn == 0) { lastVizTime = simTime; return; }

    WbNodeRef root       = wb_supervisor_node_get_root();
    WbFieldRef childrenF = wb_supervisor_node_get_field(root, "children");

    if (lastNode) {
        wb_supervisor_node_remove(lastNode);
        lastNode = nullptr;
    }

    wb_supervisor_field_import_mf_node_from_string(childrenF, -1, node_str.c_str());
    int total = wb_supervisor_field_get_count(childrenF);
    if (total > 0)
        lastNode = wb_supervisor_field_get_mf_node(childrenF, total - 1);

    lastVizTime = simTime;
    std::cout << "[COSTMAP_VIZ] drew=" << drawn
              << " gps_cell=(" << robot_col << "," << robot_row << ")"
              << " gps_pos=(" << gps_x << "," << gps_y << ")"
              << " wbt_anchor=(" << wbt_ox << "," << wbt_oy << ")"
              << std::endl;
}

static bool visualizeRouteCorridor(NavigationManager& nav)
{
    float x0 = 0.0f, y0 = 0.0f, x1 = 0.0f, y1 = 0.0f, halfWidth = 0.0f;
    if (!nav.getCurrentArcWorldCorridor(x0, y0, x1, y1, halfWidth)) return false;

    const double dx = x1 - x0, dy = y1 - y0;
    const double len = std::hypot(dx, dy);
    if (len < 1e-3) return false;

    WbNodeRef root = wb_supervisor_node_get_root();
    WbFieldRef children = wb_supervisor_node_get_field(root, "children");

    static WbNodeRef last_corridor_node = nullptr;
    if (last_corridor_node) {
        wb_supervisor_node_remove(last_corridor_node);
        last_corridor_node = nullptr;
    }

    const double heading = std::atan2(dy, dx);
    const double nx = -dy / len, ny = dx / len;  // perpendicular unit vector
    const double midx = (x0 + x1) * 0.5, midy = (y0 + y1) * 0.5;

    const float strip_thickness_z = 0.02f;  // שטוח על הריצפה — בלי גובה/נפח
    const float strip_floor_z     = 0.015f; // קצת מעל הקרקע, נמנע z-fighting
    const float strip_width       = 0.15f;
    const float strip_length      = static_cast<float>(len) + 0.5f;

    char buf[512];
    std::string node_str = "Transform { children [ ";

    for (int side = -1; side <= 1; side += 2) {
        double cx = midx + nx * halfWidth * side;
        double cy = midy + ny * halfWidth * side;
        std::snprintf(buf, sizeof(buf),
            "Pose { translation %f %f %f rotation 0 0 1 %f children [ Shape { "
            "appearance Appearance { material Material { diffuseColor 0.1 0.9 0.9 "
            "emissiveColor 0.05 0.7 0.7 transparency 0.3 } } "
            "geometry Box { size %f %f %f } } ] } ",
            cx, cy, strip_floor_z, heading, strip_length, strip_width, strip_thickness_z);
        node_str += buf;
    }
    node_str += " ] }";

    wb_supervisor_field_import_mf_node_from_string(children, -1, node_str.c_str());

    int total = wb_supervisor_field_get_count(children);
    if (total > 0) {
        last_corridor_node = wb_supervisor_field_get_mf_node(children, total - 1);
    }
    return true;
}

void signalHandler(int signum)
{
    std::cout << "\nInterrupt signal (" << signum << ") received. Shutting down...\n";
    keep_running = false;
}

static WbDeviceTag requireDevice(const char* name)
{
    WbDeviceTag device = wb_robot_get_device(name);

    if (device == 0) {
        std::cerr << "[Webots] required device not found: " << name << "\n";
    }

    return device;
}

static WbDeviceTag optionalDevice(const char* name)
{
    WbDeviceTag device = wb_robot_get_device(name);

    if (device == 0) {
        std::cerr << "[Webots] optional device not found: " << name << "\n";
    }

    return device;
}

// ─────────────────────────────────────────────────────────────────────────────
// בדיקת תקינות חד-פעמית על מופע EKF זמני ומבודד (לא ה-EKF החי של הניווט!).
// מטרה: לאשר שמדידת Encoder vx=0.2 מזיזה את Vx לכיוון 0.2 ולא לערך שגוי
// כמו 1.6/2.0. לא בודקים שוויון מדויק — רק שהתנועה היא בכיוון הנכון ובסדר גודל סביר.
// ─────────────────────────────────────────────────────────────────────────────
static void runEkfEncoderSanityCheck()
{
    EKF<Config::DimWheelchairStateVector> test_ekf; // x=0, P=Identity (ראה בנאי ekf.hpp)

    const int IDX_VX   = static_cast<int>(Config::StateMembersRobot::StateVx);
    const int IDX_VYAW = static_cast<int>(Config::StateMembersRobot::StateVyaw);

    Eigen::Matrix<double, 2, Config::DimWheelchairStateVector> H =
        Eigen::Matrix<double, 2, Config::DimWheelchairStateVector>::Zero();
    H(0, IDX_VX)   = 1.0;
    H(1, IDX_VYAW) = 1.0;

    Eigen::Matrix<double, 2, 1> Z;
    Z << 0.2, 0.0;

    Eigen::Matrix<double, 2, 2> R = Eigen::Matrix<double, 2, 2>::Zero();
    R(0, 0) = 0.02;
    R(1, 1) = 0.04;

    const double vx_before = test_ekf.x(IDX_VX);
    Eigen::Matrix<double, 2, 1> Z_pred = H * test_ekf.x;

    test_ekf.update<2>(Z, H, R, Z_pred);

    const double vx_after = test_ekf.x(IDX_VX);
    // ציפייה: Vx זז לכיוון המדידה (0.2), לא קופץ לערך לא קשור כמו 1.6/2.0.
    const bool moved_toward_measurement = (vx_after > vx_before) && (vx_after <= 0.25);

    std::cout << "[EKF_SANITY_CHECK] " << (moved_toward_measurement ? "PASS" : "FAIL")
              << " vx_before=" << vx_before
              << " measurement=0.2"
              << " vx_after=" << vx_after
              << " (expected: moves toward 0.2, not toward 1.6/2.0)"
              << std::endl;
}

int main()
{

    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_ERROR);
    runEkfEncoderSanityCheck();
    constexpr int TIME_STEP_MS = 10;  // 100Hz בסיסי

    std::signal(SIGINT,  signalHandler);
    std::signal(SIGTERM, signalHandler);

    if (!ConfigManager::load("C:/Users/User/Desktop/TrackObject/config/sensor_extrinsics.yaml")) {
        std::cerr << "[BOOT][WARN] sensor_extrinsics.yaml not loaded — using identity transforms\n";
    } else {
        std::cout << "[BOOT] sensor_extrinsics.yaml loaded OK\n";
    }

    // אתחול Webots C API
    wb_robot_init();

    // ─────────────────────────────────────────────
    // 1. מנועים
    // ─────────────────────────────────────────────
    WbDeviceTag leftMotor  = requireDevice("left_wheel_motor");
    WbDeviceTag rightMotor = requireDevice("right_wheel_motor");

    if (leftMotor == 0 || rightMotor == 0) {
        std::cerr << "[Webots] Cannot start: wheel motors are missing.\n";
        wb_robot_cleanup();
        return 1;
    }

    WebotsDrive::getInstance().init(
        leftMotor,
        rightMotor,
        0.34,   // WHEEL_RADIUS [m]
        0.96,   // WHEEL_BASE [m]
        12.0    // max wheel velocity [rad/s]
    );

    WebotsDrive::getInstance().stop();

    // ─────────────────────────────────────────────
    // 2. חיישנים
    // ─────────────────────────────────────────────
    SensorProducerWebots::Devices devices{};

    devices.gps           = optionalDevice("gps");
    devices.imu           = optionalDevice("imu");
    devices.accelerometer = optionalDevice("accelerometer");
    devices.gyro          = optionalDevice("gyro");

    devices.frontLidar = optionalDevice("front_lidar");
    devices.leftLidar  = optionalDevice("left_lidar");
    devices.rightLidar = optionalDevice("right_lidar");

    devices.camera = optionalDevice("rgb_camera");
    devices.radar  = optionalDevice("front_radar");

    devices.leftEncoder  = optionalDevice("left_wheel_encoder");
    devices.rightEncoder = optionalDevice("right_wheel_encoder");

    SensorProducerWebots webotsProducer(devices, TIME_STEP_MS);
    webotsProducer.enableDevices();
    

    std::cout << "Starting Wheelchair SLAM System as Webots C controller in C++ project...\n";

    // ─────────────────────────────────────────────
    // 3. מנהלי מערכת קיימים
    // ─────────────────────────────────────────────
    std::cout << "[BOOT] NavigationManager...\n";
    auto& nav = NavigationManager::getInstance();

    std::cout << "[BOOT] StatePriorityQueue...\n";
    auto& stateQ = StatePriorityQueue::getInstance();

    std::cout << "[BOOT] SensorFrameManager...\n";
    auto& frameManager = SensorFrameManager::getInstance();

    std::cout << "[BOOT] FactorGraphManager...\n";
    (void)FactorGraphManager::getInstance();

    std::cout << "[BOOT] create StateThread...\n";
    StateThread state_thread;

    std::cout << "[BOOT] create SensorConsumer...\n";
    SensorConsumer consumer;

    std::cout << "[BOOT] create ThreadGrafFector...\n";
    ThreadGrafFector graph_optimizer;

    std::cout << "[BOOT] all init done\n";

    // לכידת פריים ראשון לויזואליזציה — הקובץ יצא ל-pipeline_capture/
    LidarCloud::enablePipelineCapture(
        "C:/Users/User/Desktop/TrackObject/pipeline_capture");

        bool globalMapCreated = false;
        double lastNavTick = -1.0;
        double lastMapAttemptTime = -999.0;  // throttle: לא לנסות יותר מפעם בX שניות
        std::size_t lastCorridorVizGlobalIndex = static_cast<std::size_t>(-1); // לזיהוי קידום קשת

    // יעד הדגמה. בהמשך אפשר להחליף ליעד שמגיע מממשק משתמש.
    Location myGoal { 32.753974496792566, 35.105129461555855 };  // רכסים — יעד ~184m
    // ─────────────────────────────────────────────
    // 4. לולאת Webots
    // ─────────────────────────────────────────────
    // Before starting threads, wait for first valid GPS and initialize EKF/graph.
    auto waitForFirstValidGpsAndInitializeSystem = [&](double timeoutSeconds) -> bool {
        using namespace std::chrono;
        auto& manager = SensorProducerManager::getInstance();
        GpsProcessing gps_proc;
        auto start = steady_clock::now();

        static int step_count = 0;
        while (keep_running) {
            ++step_count;
            if (step_count <= 3 || step_count % 50 == 0) {
                std::cout << "[DIAG] Before wb_robot_step #" << step_count << std::endl; std::cout.flush();
            }
            if (wb_robot_step(TIME_STEP_MS) == -1) break;
            if (step_count <= 3 || step_count % 50 == 0) {
                std::cout << "[DIAG] After wb_robot_step #" << step_count << std::endl; std::cout.flush();
            }
            const double t = wb_robot_get_time();
            webotsProducer.update();

            // try to pop one gps sentence with short timeout
            auto samples = manager.gpsRing.popLastNWithTimeout(1, milliseconds(TIME_STEP_MS));
            if (samples.empty()) {
                // check timeout
                if (duration_cast<seconds>(steady_clock::now() - start).count() >= timeoutSeconds) {
                    std::cerr << "[BOOT][ERROR] Timeout waiting for first valid GPS (" << timeoutSeconds << "s)\n";
                    return false;
                }
                continue;
            }

            const auto& meas = samples.front();
            std::stringstream ss(meas.data);

            std::cout << "[BOOT] Received GPS sentence, processing... t=" << meas.timestamp << std::endl;

            GpsObject go = gps_proc.process(ss, meas.timestamp);

            // capture raw lat/lon before GpsData::process transforms into local meters
            double raw_lat = go.x_local;
            double raw_lon = go.y_local;
            // validate
            if (!go.isValid) {
                std::cerr << "[BOOT][WARN] GPS reported invalid. continuing...\n";
                continue;
            }

            if (!std::isfinite(go.timestamp) || go.timestamp < 0.0) {
                std::cerr << "[BOOT][WARN] GPS timestamp invalid. continuing...\n";
                continue;
            }

            if (go.Z.size() != Config::MeasurementSizeGps) {
                std::cerr << "[BOOT][WARN] GPS measurement size mismatch. continuing...\n";
                continue;
            }

            // Set map origin
            auto& nav = NavigationManager::getInstance();
            auto map = nav.getMapData();
            if (map) {
                map->origin_lat = raw_lat;
                map->origin_lon = raw_lon;
                std::cout << "[BOOT] Map origin set to lat=" << raw_lat << " lon=" << raw_lon << std::endl;
            }

            // initialize EKF state
            auto& stateQ = StatePriorityQueue::getInstance();
            {
                std::lock_guard<std::mutex> lock(stateQ.getEKFMutex());
                auto& ekf = stateQ.getEKF();
                // zero XY at origin, set Z/heading/speed from GPS measurement
                ekf.x.setZero();
                ekf.x(static_cast<int>(Config::StateMembersRobot::StateZ)) = go.Z(2);

                // זריעת yaw חד-פעמית: ה-heading שמגיע מ-go.Z(3) עובר המרת
                // compass->ENU (π/2 - heading*π/180), אבל heading עצמו הוא בעצם
                // ה-yaw הגולמי של Webots עטוף ל-[0,360) — לא compass אמיתי. ההמרה
                // הזו על ערך שכבר ENU מקלקלת את נקודת ההתחלה. כאן זורעים את ה-yaw
                // ישירות מ-Webots (ground truth), פעם אחת בלבד, בלי לפתוח עדכון שוטף.
                const double old_gps_seed_yaw = (go.Z.size() > 3) ? go.Z(3) : 0.0;
                double webots_yaw_seed = old_gps_seed_yaw; // fallback אם ה-IMU לא קיים
                if (devices.imu) {
                    const double* gt_rpy = wb_inertial_unit_get_roll_pitch_yaw(devices.imu);
                    if (gt_rpy) {
                        webots_yaw_seed = gt_rpy[2];
                    }
                }
                ekf.x(static_cast<int>(Config::StateMembersRobot::StateYaw)) = webots_yaw_seed;

                if (go.Z.size() > 4)
                    ekf.x(static_cast<int>(Config::StateMembersRobot::StateVx)) = go.Z(4);

                // initialize covariance P sensibly
                ekf.P.setIdentity();
                ekf.P *= 1.0; // 1m^2 position variance

                const double ekf_yaw_after_seed = ekf.x(static_cast<int>(Config::StateMembersRobot::StateYaw));
                std::cout << "[YAW_SEED_DEBUG]"
                          << " webots_yaw_seed=" << webots_yaw_seed
                          << " old_gps_seed_yaw=" << old_gps_seed_yaw
                          << " ekf_yaw_after_seed=" << ekf_yaw_after_seed
                          << " diff_ekf_vs_webots=" << (ekf_yaw_after_seed - webots_yaw_seed)
                          << std::endl;
            }

            stateQ.setTimeLastUpdate(go.timestamp);

            // create first keyframe in factor graph
            NodeFactor firstNode;
            {
                std::lock_guard<std::mutex> lock(stateQ.getEKFMutex());
                firstNode.filter = stateQ.getEKF();
            }

            FactorGraphManager::getInstance().addKeyframe(go.timestamp, firstNode);

            std::cout << "[BOOT] First valid GPS processed and initial keyframe created at t=" << go.timestamp << std::endl;
            return true;
        }

        return false;
    };

    // wait for first GPS (timeout 20s)
    if (!waitForFirstValidGpsAndInitializeSystem(20.0)) {
        wb_robot_cleanup();
        return 1;
    }

    // Only after initialization start threads
    std::cout << "[BOOT] init StateThread...\n";
    state_thread.init();

    std::cout << "[BOOT] init SensorConsumer...\n";
    consumer.init();

    std::cout << "[BOOT] init GraphOptimizer...\n";
    graph_optimizer.init();
    while (keep_running && wb_robot_step(TIME_STEP_MS) != -1) {
        const double simTime = wb_robot_get_time();

        // 1. קריאת חיישנים מ־Webots ודחיפה ל־SensorProducerManager.
        // אין קבצי ביניים ואין Python.
        webotsProducer.update();

        // ─────────────────────────────────────────────
        // בדיקה מבודדת של המנועים — זמני! עוקף ניווט/EKF לחלוטין כדי לוודא
        // ש-linear חיובי = קדימה בפועל ב-Webots. לכיבוי: MOTOR_ISOLATION_TEST=false.
        // ─────────────────────────────────────────────
        static constexpr bool MOTOR_ISOLATION_TEST = false;
        if (MOTOR_ISOLATION_TEST) {
            static int motor_test_count = 0;
            if (++motor_test_count % 50 == 0)
                std::cout << "[MOTOR_TEST] sending linear=0.2 angular=0.0 (no navigation, raw test)" << std::endl;
            sendDriveCommand(0.2f, 0.0f);
            continue;
        }

        // 2. מחכים עד שה־EKF יקבל לפחות מדידה אחת.
        if (!globalMapCreated) {
            if (stateQ.getTimeLastUpdate() < 0.0) {
                WebotsDrive::getInstance().stop();
                continue;
            }

            // throttle: לא לנסות לבנות מפה יותר מפעם ב-3 שניות
            if ((simTime - lastMapAttemptTime) < 3.0) {
                WebotsDrive::getInstance().stop();
                continue;
            }
            lastMapAttemptTime = simTime;
            std::cout << "[Nav] Attempting createMapGlobal at t=" << simTime << "\n";

            try {
                auto mapData = nav.getMapData();
                if (!mapData) {
                    WebotsDrive::getInstance().stop();
                    continue;
                }
                // origin_lat/lon stored as decimal degrees from GPS parsing — use directly
                nav.createMapGlobal(Location{mapData->origin_lat, mapData->origin_lon}, myGoal);
            } catch (const std::exception& e) {
                std::cerr << "[Nav] createMapGlobal failed: " << e.what() << "\n";
                WebotsDrive::getInstance().stop();
                continue;
            }

            if (nav.getPateGlobal().empty()) {
                std::cerr << "[Nav] Global A* failed. Will retry in 3s.\n";
                WebotsDrive::getInstance().stop();
                continue;
            }

            globalMapCreated = true;

            std::cout << "[Nav] Global path ready: "
                      << nav.getPateGlobal().size()
                      << " nodes\n";

            // סימון הנתיב הגלובלי בסימולציה
            visualizeGlobalPath(nav);
        }

        // עדכון הסימון הוויזואלי של המסדרון הגלובלי (routeLimitLayer) — פעם אחת
        // בכל קשת (כשמתקדמים לצומת הגלובלי הבא), לא על טיימר קבוע. עכשיו זו
        // קריאת IPC יחידה ל-Webots (PointSet אחד) במקום אחת לכל תא.
        if (globalMapCreated) {
            std::size_t currentGlobalIndex = nav.getGlobalPathIndex();
            if (currentGlobalIndex != lastCorridorVizGlobalIndex) {
                // מנסים לצייר; אם routeLimitLayer לקשת הזו עדיין לא מוכן (runDLite
                // עדיין לא רץ ב-thread האחר), לא מעדכנים את האינדקס — ננסה שוב
                // בלולאה הבאה, עד שבאמת מצליחים לצייר את הקשת הזו.
                if (visualizeRouteCorridor(nav)) {
                    lastCorridorVizGlobalIndex = currentGlobalIndex;
                }
            }
        }

        // 3. ניווט מקומי כל 200ms, כמו NAV_LOOP_PERIOD.
        // GPS_POSE: מחליף x/y של EKF ב-GPS (X/Y מ-GPS כן נחשבים אמינים).
        // WEBOTS_FAKE_NMEA: ה-NMEA ב-Webots נבנה ידנית מתוך נתוני הסימולציה —
        // heading_deg ו-speed (VTG/RMC) לא אמינים ולא ממירים נכון ל-compass.
        // כל עוד זה true: לא להשתמש בהם בכלל ל-yaw/speed של הניווט.
        static constexpr bool   USE_GPS_POSE_FOR_NAVIGATION_TEST = true;
        static constexpr bool   WEBOTS_FAKE_NMEA                 = true;
        static constexpr double GPS_YAW_MIN_SPEED_MPS            = 0.5;
        static constexpr double GPS_YAW_MAX_HEADING_JUMP_DEG      = 5.0;

        if (lastNavTick < 0.0 || (simTime - lastNavTick) >= 0.20) {
            lastNavTick = simTime;

            const Eigen::VectorXd state = stateQ.getStateVector();

            auto currentFrame   = frameManager.getOrCreateFrame(simTime);
            auto trackedObjects = frameManager.getTrackedObject();
            if (nav.shouldStop()) {
                sendDriveCommand(0.0f, 0.0f);
                continue;
            }

            auto maybe_step = nav.getNextStep();

            if (!maybe_step.has_value()) {
                sendDriveCommand(0.0f, 0.0f);
                continue;
            }

            auto mapData = nav.getMapData();

            if (!mapData) {
                std::cerr << "[Nav] mapData is null. Stopping.\n";
                sendDriveCommand(0.0f, 0.0f);
                continue;
            }

            // בניית nav_state: EKF כברירת מחדל, GPS x/y אם הדגל פעיל והנתון תקין.
            // ה-yaw: EKF כברירת מחדל; GPS heading נחשב רק אם המהירות מעל סף וההדינג יציב.
            Eigen::VectorXd nav_state = state;
            {
                static double last_gps_heading_deg = 0.0;
                static bool   has_last_gps_heading  = false;

                const int IDX_X   = static_cast<int>(Config::StateMembersRobot::StateX);
                const int IDX_Y   = static_cast<int>(Config::StateMembersRobot::StateY);
                const int IDX_YAW = static_cast<int>(Config::StateMembersRobot::StateYaw);
                const int IDX_VX  = static_cast<int>(Config::StateMembersRobot::StateVx);

                const double ekf_yaw = state(IDX_YAW);
                const double ekf_vx  = state(IDX_VX);

                if (USE_GPS_POSE_FOR_NAVIGATION_TEST && GpsPoseStore::getInstance().hasValid()) {
                    auto gp = GpsPoseStore::getInstance().get();
                    const double gps_yaw       = M_PI / 2.0 - gp.heading_deg * M_PI / 180.0;
                    const double gps_speed_mps = gp.speed_mps;

                    bool heading_stable = false;
                    if (has_last_gps_heading) {
                        double dh = gp.heading_deg - last_gps_heading_deg;
                        while (dh >  180.0) dh -= 360.0;
                        while (dh < -180.0) dh += 360.0;
                        heading_stable = std::fabs(dh) < GPS_YAW_MAX_HEADING_JUMP_DEG;
                    }
                    last_gps_heading_deg   = gp.heading_deg;
                    has_last_gps_heading   = true;

                    // בעיה 1: ב-Webots ה-NMEA מזויף — heading/speed לא אמינים בכלל.
                    // X/Y נחשבים אמינים (use_position=1) ולכן עדיין משמשים לניווט.
                    const bool use_position = true;
                    const bool use_heading  = !WEBOTS_FAKE_NMEA;
                    const bool use_speed    = !WEBOTS_FAKE_NMEA;
                    const char* nmea_source = WEBOTS_FAKE_NMEA ? "WEBOTS_FAKE_NMEA" : "REAL_GPS";

                    std::cout << "[NMEA_TRUST_DEBUG]"
                              << " use_position=" << use_position
                              << " use_heading=" << use_heading
                              << " use_speed=" << use_speed
                              << " source=" << nmea_source
                              << std::endl;

                    // בעיה 2: GPS_POS + EKF_YAW. GPS yaw נשקל רק אם use_heading/use_speed
                    // פעילים (כלומר לא במצב Webots fake NMEA), בנוסף לסף מהירות ויציבות.
                    const bool speed_valid = std::isfinite(gps_speed_mps);
                    const bool use_gps_yaw = use_heading && use_speed
                                              && speed_valid
                                              && (gps_speed_mps >= GPS_YAW_MIN_SPEED_MPS)
                                              && heading_stable;
                    const double nav_yaw    = use_gps_yaw ? gps_yaw : ekf_yaw;
                    const char* pose_source = use_gps_yaw ? "GPS_POS+GPS_YAW" : "GPS_POS+EKF_YAW";
                    const char* yaw_source  = use_gps_yaw ? "GPS_POS_GPS_YAW" : "GPS_POS_EKF_YAW";

                    nav_state(IDX_X)   = gp.x;
                    nav_state(IDX_Y)   = gp.y;
                    nav_state(IDX_YAW) = nav_yaw;

                    std::cout << "[NAV_POSE_SOURCE] source=" << pose_source
                              << " robot_x=" << gp.x << " robot_y=" << gp.y
                              << " robot_yaw=" << nav_yaw
                              << " gps_heading_deg=" << gp.heading_deg << std::endl;

                    std::cout << "[YAW_SOURCE_DEBUG]"
                              << " gps_yaw=" << gps_yaw
                              << " ekf_yaw=" << ekf_yaw
                              << " chosen_yaw=" << nav_yaw
                              << " gps_speed=" << gps_speed_mps
                              << " ekf_vx=" << ekf_vx
                              << " heading_stable=" << heading_stable
                              << " use_gps_yaw=" << use_gps_yaw
                              << " source=" << yaw_source
                              << std::endl;

                    // אבחון בלבד: webots_yaw_raw הוא ground truth אמיתי מ-Webots
                    // (wb_inertial_unit_get_roll_pitch_yaw) — לא מחושב, לא משוער.
                    // משווים אותו ישירות ל-ekf_yaw ול-gps_yaw_enu כדי לבדוק התאמה.
                    {
                        const double* gt_rpy = devices.imu ? wb_inertial_unit_get_roll_pitch_yaw(devices.imu) : nullptr;
                        if (gt_rpy) {
                            std::cout << "[YAW_GROUND_TRUTH_DEBUG]"
                                      << " webots_yaw_raw=" << gt_rpy[2]
                                      << " ekf_yaw=" << ekf_yaw
                                      << " gps_yaw_enu=" << gps_yaw
                                      << " diff_ekf_vs_webots=" << (ekf_yaw - gt_rpy[2])
                                      << " diff_gps_vs_webots=" << (gps_yaw - gt_rpy[2])
                                      << std::endl;
                        }
                    }
                } else {
                    std::cout << "[NAV_POSE_SOURCE] source=EKF robot_x=" << state(IDX_X)
                              << " robot_y=" << state(IDX_Y) << " robot_yaw=" << state(IDX_YAW)
                              << std::endl;
                }
            }

            for (;;) {
                auto s = nav.getNextStep();
                if (!s.has_value()) break;
                if (distToCell(nav_state, s.value(), *mapData) >= CELL_REACH_THRESHOLD) break;
                nav.popNextStep();
            }
            auto target = nav.getLookaheadStep(1.0f, nav_state, *mapData);
            if (target.has_value()) {
                // --- אבחון בלבד: לא משנה שום לוגיקה, לא נוגע ב-Move/IMU/GPS/FactorGraph ---
                {
                    const int IDX_X   = static_cast<int>(Config::StateMembersRobot::StateX);
                    const int IDX_Y   = static_cast<int>(Config::StateMembersRobot::StateY);
                    const int IDX_YAW = static_cast<int>(Config::StateMembersRobot::StateYaw);

                    const double ekf_yaw      = state(IDX_YAW);
                    const double robot_yaw_used = nav_state(IDX_YAW);
                    double gps_heading_deg = 0.0;
                    double gps_yaw_enu     = 0.0;
                    if (GpsPoseStore::getInstance().hasValid()) {
                        auto gp = GpsPoseStore::getInstance().get();
                        gps_heading_deg = gp.heading_deg;
                        gps_yaw_enu     = M_PI / 2.0 - gp.heading_deg * M_PI / 180.0;
                    }

                    const float  robot_x = static_cast<float>(nav_state(IDX_X));
                    const float  robot_y = static_cast<float>(nav_state(IDX_Y));
                    const float  target_x = mapData->originX + (target.value().first  + 0.5f) * RESOLUTION;
                    const float  target_y = mapData->originY + (target.value().second + 0.5f) * RESOLUTION;
                    const double target_heading = std::atan2(target_y - robot_y, target_x - robot_x);
                    double heading_error = target_heading - robot_yaw_used;
                    while (heading_error >  M_PI) heading_error -= 2.0 * M_PI;
                    while (heading_error < -M_PI) heading_error += 2.0 * M_PI;

                    std::cout << "[YAW_CONSISTENCY_DEBUG]"
                              << " gps_heading_deg=" << gps_heading_deg
                              << " gps_yaw_enu=" << gps_yaw_enu
                              << " ekf_yaw=" << ekf_yaw
                              << " robot_yaw_used_for_navigation=" << robot_yaw_used
                              << " target_heading=" << target_heading
                              << " heading_error=" << heading_error
                              << std::endl;

                    const Location goalLoc = nav.getLocation();
                    const double lat_rad = mapData->origin_lat * (M_PI / 180.0);
                    const double final_goal_x = (goalLoc.lon - mapData->origin_lon) * 111320.0 * std::cos(lat_rad);
                    const double final_goal_y = (goalLoc.lat - mapData->origin_lat) * 111320.0;

                    const double tdx = target_x - robot_x;
                    const double tdy = target_y - robot_y;
                    const double tlen = std::sqrt(tdx * tdx + tdy * tdy);
                    const double gdx = final_goal_x - robot_x;
                    const double gdy = final_goal_y - robot_y;
                    const double glen = std::sqrt(gdx * gdx + gdy * gdy);
                    double dot = 0.0;
                    if (tlen > 1e-6 && glen > 1e-6) {
                        dot = (tdx * gdx + tdy * gdy) / (tlen * glen);
                    }

                    std::cout << "[TARGET_DIRECTION_DEBUG]"
                              << " robot_x=" << robot_x << " robot_y=" << robot_y
                              << " target_x=" << target_x << " target_y=" << target_y
                              << " final_goal_x=" << final_goal_x << " final_goal_y=" << final_goal_y
                              << " dot=" << dot
                              << (dot < 0.0 ? " [WARNING_TARGET_AWAY_FROM_GOAL]" : "")
                              << std::endl;

                    // אבחון בלבד (בעיה 4): מקור היעד, lat/lon, world/local x,y, מרחק בפועל.
                    // myGoal הוא קבוע ב-main.cpp ("ידני") — לא OSM, לא Webots Supervisor.
                    {
                        const double dist_to_goal = std::hypot(final_goal_x - robot_x, final_goal_y - robot_y);
                        std::cout << "[FINAL_GOAL_DEBUG]"
                                  << " goal_source=MANUAL_HARDCODED_MAIN_CPP"
                                  << " goal_lat=" << goalLoc.lat << " goal_lon=" << goalLoc.lon
                                  << " goal_world_x=" << final_goal_x << " goal_world_y=" << final_goal_y
                                  << " robot_x=" << robot_x << " robot_y=" << robot_y
                                  << " distance_to_goal=" << dist_to_goal
                                  << std::endl;
                    }
                }

                auto [linear, angular] = computeCommand(nav_state, target.value(), *mapData);

                // Emergency stop: raw front-center LiDAR overrides D*/Move/fusion.
                // Triggers even if tracked_objects_count=0 or D* hasn't replanned yet.
                {
                    constexpr float  ESTOP_THRESHOLD_M = 1.3f;
                    constexpr double ESTOP_MAX_AGE_S   = 0.5;
                    const float  fr = FrontLidar::g_min_range_m.load(std::memory_order_relaxed);
                    const double fts = FrontLidar::g_min_range_ts.load(std::memory_order_relaxed);
                    const double age = (fts > 0.0) ? (simTime - fts) : std::numeric_limits<double>::infinity();
                    if (linear > 0.0f && fr > 0.0f && fr < ESTOP_THRESHOLD_M && age < ESTOP_MAX_AGE_S) {
                        std::cout << "[RAW_LIDAR_EMERGENCY_STOP]"
                                  << " min_front_range=" << fr
                                  << " threshold=" << ESTOP_THRESHOLD_M
                                  << " original_linear=" << linear
                                  << " original_angular=" << angular
                                  << " sensor_age_s=" << age
                                  << std::endl;
                        linear  = 0.0f;
                        angular = 0.0f;
                    }
                }

                sendDriveCommand(linear, angular);
            } else {
                sendDriveCommand(0.0f, 0.0f);
            }

            visualizeCostMap(nav, state, nav_state, simTime);
        }
    }

    // ─────────────────────────────────────────────
    // 5. כיבוי מסודר
    // ─────────────────────────────────────────────
    WebotsDrive::getInstance().stop();

    std::cout << "Stopping system...\n";

    graph_optimizer.stop();
    consumer.stop();
    state_thread.stop();

    wb_robot_cleanup();

    std::cout << "System safely terminated.\n";

    return 0;
}