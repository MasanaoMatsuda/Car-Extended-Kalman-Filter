#include "kalman_filter.h"
#include <iostream>

using std::cout;
using std::endl;

KalmanFilter::KalmanFilter()
{
}

KalmanFilter::~KalmanFilter()
{
}

void KalmanFilter::InitState(VectorXd &x_in, MatrixXd &P_in)
{
    x_ = x_in;
    P_ = P_in;
}

void KalmanFilter::Predict(MatrixXd F_, MatrixXd Q_)
{
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z, MatrixXd H_, MatrixXd R_)
{
    cout << "Update()" << endl;
    VectorXd y = z - H_ * x_;
    MatrixXd S = H_ * P_ * H_.transpose() + R_;
    MatrixXd K = P_ * H_.transpose() * S.inverse();

    x_ = x_ + (K * y);
    MatrixXd I_ = MatrixXd::Identity(x_.size(), x_.size());
    P_ = (I_ - K * H_) * P_;
    cout << "End of Update()" << endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z, MatrixXd H_, MatrixXd R_)
{

}
