#include "kalman_filter.h"

KalmanFilter::KalmanFilter()
{
}

KalmanFilter::~KalmanFilter()
{
}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in)
{
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict()
{
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z)
{
    VecrotXd y = z - H_ * x_;
    MatrixXd S = H_ * P_ * H_.transpose() + R_;
    MatrixXd K = P_ * H_.transpose() * S.inverse();

    x_ = x_ + (K * y);
    MatrixXd I_ = MatrixXd::Identity(x_.size(), x_.size());
    P_ = (I_ - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z)
{

}
