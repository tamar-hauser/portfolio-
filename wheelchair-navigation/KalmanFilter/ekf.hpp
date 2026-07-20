#pragma once
#include <Eigen/Dense>
#include <cmath>
#ifndef M_PI
constexpr double M_PI = 3.14159265358979323846;
#endif

// תיקון: הורדת MeasurementSize מהמחלקה הכללית
template<int StateSize>
class EKF {
public:
    Eigen::Matrix<double, StateSize, 1> x;
    Eigen::Matrix<double, StateSize, StateSize> P;

    EKF() {
        x.setZero();
        P.setIdentity();
    }

    // פונקציית עזר הכרחית לנרמול זווית
    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void predict(const Eigen::Matrix<double, StateSize, 1>& x_next,
                 const Eigen::Matrix<double, StateSize, StateSize>& F,
                 const Eigen::Matrix<double, StateSize, StateSize>& Q) {
        x = x_next;
        P = F * P * F.transpose() + Q;
    }

    // פונקציית ה-update נשארת טמפלייט שמקבלת את גודל המדידה דינמית בריצה
    template<int MeasurementSize>
    void update(const Eigen::Matrix<double, MeasurementSize, 1>& z,
                const Eigen::Matrix<double, MeasurementSize, StateSize>& H,
                const Eigen::Matrix<double, MeasurementSize, MeasurementSize>& R,
                const Eigen::Matrix<double, MeasurementSize, 1>& z_pred) {

        Eigen::Matrix<double, MeasurementSize, 1> y = z - z_pred;

        Eigen::Matrix<double, MeasurementSize, MeasurementSize> S =
            H * P * H.transpose() + R;

        Eigen::Matrix<double, StateSize, MeasurementSize> K =
            P * H.transpose() * S.ldlt().solve(
                Eigen::Matrix<double, MeasurementSize, MeasurementSize>::Identity()
            );

        x = x + K * y;

        Eigen::Matrix<double, StateSize, StateSize> I =
            Eigen::Matrix<double, StateSize, StateSize>::Identity();

        // Joseph Form לעדכון השונות
        P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();

        if (StateSize > 6) {
            x(6) = normalize_angle(x(6));
        }
    }

    // גרסה דינמית — לשימוש כשגודל המדידה נקבע בזמן ריצה (SensorFusionData)
    void updateDynamic(const Eigen::VectorXd& z,
                       const Eigen::MatrixXd& H,
                       const Eigen::MatrixXd& R,
                       const Eigen::VectorXd& z_pred) {

        Eigen::VectorXd y = z - z_pred;

        Eigen::MatrixXd S = H * P * H.transpose() + R;

        Eigen::MatrixXd K = P * H.transpose() *
            S.ldlt().solve(Eigen::MatrixXd::Identity(S.rows(), S.cols()));

        x = x + K * y;

        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(StateSize, StateSize);
        P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();

        if (StateSize > 6) {
            x(6) = normalize_angle(x(6));
        }
    }

    const Eigen::Matrix<double, StateSize, 1>& getState() const {
        return x;
    }
};