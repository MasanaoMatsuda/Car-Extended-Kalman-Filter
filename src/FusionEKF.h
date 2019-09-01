#include "kalman_filter.h"
#include "measurement_package.h"
#include "tools.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class FusionEKF
{
    MatrixXd R_laser_;
    MatrixXd R_radar_;
    MatrixXd H_laser_;
    MatrixXd Hj_;
    MatrixXd P_;
    MatrixXd F_;
    MatrixXd Q_;
    VectorXd x_;
    bool is_initialized_;
    long long previous_timestamp_;
    const int noise_ax = 9;
    const int noise_ay = 9;
    Tools tools;

public:
    KalmanFilter ekf_;
    FusionEKF();
    ~FusionEKF();
    void ProcessMeasurement(const MeasurementPackage &measurement_pack);
    VectorXd InitializeStateX(const MeasurementPackage &measurement_pack);
    MatrixXd CalculateTransitionCovariance(const float &dt);
    MatrixXd CalculateProcessCovariance(const float &dt);
    MatrixXd CalculateJacobian(const VectorXd &x_state);
};
