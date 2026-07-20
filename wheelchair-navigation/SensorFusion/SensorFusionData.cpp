#include "SensorFusionData.hpp"
#include "SensorFusionObject.hpp"

// תיקון: טיפוס הפרמטר שונה ל-SensorfusionObject (f קטנה)
void SensorFusionData::process(SensorFusionObject& SO) {
    int z_size = 0;
    if (SO.has_lidar)  z_size += 4; 
    if (SO.has_camera) z_size += 3; 
    if (SO.has_radar)  z_size += 6; 

    if (z_size == 0) return;

    // תיקון: הורדת ה-this. או שינוי ל-this-> מכיוון ש-this הוא מצביע (Pointer)
    SO.Z = Eigen::VectorXd::Zero(z_size);
    SO.H = Eigen::MatrixXd::Zero(z_size, 8); 
    SO.R = Eigen::MatrixXd::Zero(z_size, z_size);

    // תיקון: תיקון שגיאת כתיב מ-builZ ל-buildZ (הוספת d)
    buildZ(SO);
    buildH(SO);
    buildR(SO);
    buildF(SO);
}

void SensorFusionData::buildH(SensorFusionObject& SO) {
    int curr_row = 0;
    double x = SO.filter.x(0);
    double y = SO.filter.x(1);
    double vx = SO.filter.x(3);
    double vy = SO.filter.x(4);
    double v2 = vx * vx + vy * vy;

    // --- בלוק לידאר (לא לינארי עבור Yaw) ---
    if (SO.has_lidar) {
        SO.H(curr_row, 0) = 1.0; SO.H(curr_row + 1, 1) = 1.0; SO.H(curr_row + 2, 2) = 1.0;
        // נגזרת ה-Yaw לפי vx ו-vy (כפי שמופיע ב-LidarData.cpp)
        if (v2 > 0.0001) {
           SO.H(curr_row + 3, 3) = -vy / v2;
           SO.H(curr_row + 3, 4) =  vx / v2;
        }
        curr_row += 4;
    }

    // --- בלוק מצלמה (לינארי למיקום) ---
    if (SO.has_camera) {
        SO.H(curr_row, 0) = 1.0; SO.H(curr_row + 1, 1) = 1.0; SO.H(curr_row + 2, 2) = 1.0;
        curr_row += 3;
    }

    // --- בלוק רדאר (יעקוביאן פולארי/קרטזי משולב) ---
    if (SO.has_radar) {
        double rho = std::sqrt(x*x + y*y);
        if (rho < 0.0001) { // מניעת חלוקה ב-0
            SO.H.block<3,3>(curr_row, 0).setIdentity();
        } else {
            // נגזרות טווח וזווית (מתוך RadarData.cpp)
            SO.H(curr_row, 0) = x / rho;            // d(rho)/dx
            SO.H(curr_row, 1) = y / rho;            // d(rho)/dy
            SO.H(curr_row + 1, 0) = -y / (rho*rho); // d(theta)/dx
            SO.H(curr_row + 1, 1) = x / (rho*rho);  // d(theta)/dy
            SO.H(curr_row + 2, 2) = 1.0;            // pz
        }
        // נגזרות מהירות (לינאריות במודל קרטזי)
        SO.H(curr_row + 3, 3) = 1.0; SO.H(curr_row + 4, 4) = 1.0; SO.H(curr_row + 5, 5) = 1.0;
        curr_row += 6;
    }
}

void SensorFusionData::buildR(SensorFusionObject& SO) {
    int curr_row = 0;

    if (SO.has_lidar) {
        // במידה ושמרת את ה-R בתוך האובייקט בזמן ה-process של הלידאר
        // R.block<4, 4>(curr_row, curr_row) = SO.lidar_R; 
        // כרגע נשתמש במימוש דינמי מבוסס ביטחון:
        SO.R.block<4, 4>(curr_row, curr_row) = Eigen::Matrix4d::Identity() * (1.0 / SO.confidence);
        curr_row += 4;
    }

    if (SO.has_camera) {
        // רעש מצלמה דינמי
        SO.R.block<3, 3>(curr_row, curr_row) = Eigen::Matrix3d::Identity() * 0.5; 
        curr_row += 3;
    }

    if (SO.has_radar) {
        // שימוש במטריצת הרעש של הרדאר
        SO.R.block<6, 6>(curr_row, curr_row) = Eigen::MatrixXd::Identity(6, 6) * 0.2;
        curr_row += 6;
    }
}

// תיקון: שונה ל-SensorfusionObject (f קטנה) כדי להתאים להגדרות הטיפוס בקובץ
void SensorFusionData::buildF(SensorFusionObject& SO) {
    double dt = 0.1; // יש לחשב dt = current_time - SO.timestamp
    
    // תיקון: עדכון מטריצת F של האובייקט (SO.F) ולא של המחלקה הנוכחית
    SO.F = Eigen::MatrixXd::Identity(8, 8);
    
    // מודל Constant Velocity (לינאריזציה של המעבר)
    SO.F(0, 3) = dt; SO.F(1, 4) = dt; SO.F(2, 5) = dt; // p = p + v*dt
    SO.F(6, 7) = dt;                                   // yaw = yaw + yaw_rate*dt
}
void SensorFusionData::buildZ(SensorFusionObject& SO) {
    int curr_row = 0;

    // --- לידאר ---
    // משתמשים בנתונים הקיימים (כמו ענן הנקודות או המיקום שמופיע ב-SO)
    if (SO.has_lidar) {
        SO.Z(curr_row + 0) = SO.position().x(); // שימוש במתודה הקיימת ב-SO
        SO.Z(curr_row + 1) = SO.position().y();
        SO.Z(curr_row + 2) = SO.position().z();
        SO.Z(curr_row + 3) = SO.yaw;            // משתנה קיים ב-SO
        curr_row += 4;
    }

    // --- מצלמה ---
    if (SO.has_camera) {
        // שימוש בנתוני ה-bounding_box או ה-img_points הקיימים ב-SO
        SO.Z(curr_row + 0) = (float)SO.bounding_box.x + (SO.bounding_box.width / 2.0f);
        SO.Z(curr_row + 1) = (float)SO.bounding_box.y + (SO.bounding_box.height / 2.0f);
        SO.Z(curr_row + 2) = SO.position().z(); // הערכת עומק לפי ה-filter הנוכחי
        curr_row += 3;
    }

    // --- רדאר ---
    if (SO.has_radar) {
        // במידה ואין אובייקט רדאר נפרד, נגזור מה-filter הנוכחי של האובייקט
        SO.Z(curr_row + 0) = SO.position().norm(); // rho (מרחק)
        SO.Z(curr_row + 1) = atan2(SO.position().y(), SO.position().x()); // theta
        SO.Z(curr_row + 2) = SO.velocity().norm(); // range_rate (קירוב)
        SO.Z(curr_row + 3) = SO.velocity().x();    // vx
        SO.Z(curr_row + 4) = SO.velocity().y();    // vy
        SO.Z(curr_row + 5) = SO.velocity().z();    // vz
        curr_row += 6;
    }
}