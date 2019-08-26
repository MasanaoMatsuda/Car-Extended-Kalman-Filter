#include "kalman_filter.h"
#include "measurement_package.h"
#include "Eigen/Dense"

using std::cout;
using Eigen::MatrixXd;

class FusionEKF
{
    MatrixXd R_laser_;
    MatrixXd R_radar_;
    MatrixXd H_laser_;
    MatrixXd Hj_;
    bool is_initialized_;
    long long previous_timestamp_;
    const int noise_ax;
    const int noise_ay;

public:
    KalmanFilter ekf_;
    FusionEKF();
    ~FusionEKF();
    void ProcessMeasurement(const MeasurementPackage &measurement_pack);
};
