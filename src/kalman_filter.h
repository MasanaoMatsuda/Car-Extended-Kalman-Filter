#include "Eigen/Dense"

using Eigen::VectorXd;
using Eigen::MatrixXd;

class KalmanFilter
{
public:
    VectorXd x_;
    MatrixXd P_;
    MatrixXd F_;
    MatrixXd H_;
    MatrixXd R_;
    MatrixXd Q_;
    MatrixXd I_;

    KalmanFilter();
    ~KalmanFilter();
    void Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
              MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in);
    void Predict();
    void Update(const VectorXd &z);
    void UpdateEKF(const VectorXd &z);
};
