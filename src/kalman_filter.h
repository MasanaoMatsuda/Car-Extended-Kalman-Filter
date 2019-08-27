#include "Eigen/Dense"

using Eigen::VectorXd;
using Eigen::MatrixXd;

class KalmanFilter
{
    VectorXd x_;
    MatrixXd P_;
    MatrixXd I_;

public:
    KalmanFilter();
    ~KalmanFilter();
    void InitState(VectorXd &x_in, MatrixXd &P_in);
    void Predict(MatrixXd F_, MatrixXd Q_);
    void Update(const VectorXd &z, MatrixXd H_, MatrixXd R_);
    void UpdateEKF(const VectorXd &z, MatrixXd H_, MatrixXd R_);
    VectorXd GetX();
};
