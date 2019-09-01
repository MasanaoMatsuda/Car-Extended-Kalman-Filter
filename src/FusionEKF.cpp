#include "FusionEKF.h"
#include <iostream>

using std::cout;
using std::endl;

FusionEKF::FusionEKF()
{
    is_initialized_ = false;
    previous_timestamp_ = 0;

    R_laser_ = MatrixXd(2,2);
    R_radar_ = MatrixXd(3,3);
    H_laser_ = MatrixXd(2,4);
    P_ = MatrixXd(4,4);

    // measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
               0, 0.0225;
    // measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
                0, 0.09, 0,
                0, 0, 0.09;

    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;

    P_ << 1000, 0, 0, 0,
          0, 1000, 0, 0,
          0, 0, 1000, 0,
          0, 0, 0, 1000;
}


FusionEKF::~FusionEKF()
{
}


void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{
    cout<<"FusionEKF::ProcessMeasurement()"<<endl;

    if (!is_initialized_)
    {
        x_ = InitializeStateX(measurement_pack);
        ekf_.InitState(x_, P_);
        previous_timestamp_ = measurement_pack.timestamp_;
        is_initialized_ = true;
        return;
    }

    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; // express in sec.
    previous_timestamp_ = measurement_pack.timestamp_;

    F_ = CalculateTransitionCovariance(dt);
    Q_ = CalculateProcessCovariance(dt);
    ekf_.Predict(F_, Q_);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
        MatrixXd Hj = CalculateJacobian(x_);
        ekf_.UpdateEKF(measurement_pack.raw_measurements_, Hj, R_radar_);
    }
    else
    {
        ekf_.Update(measurement_pack.raw_measurements_, H_laser_, R_laser_);
    }
}

VectorXd FusionEKF::InitializeStateX(const MeasurementPackage &measurement_pack)
{
    cout<<"FusionEKF::InitializeStateX()"<<endl;
    VectorXd x(4);
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
        VectorXd cartesian_xy = tools.ConvertPolar2Cartesian(measurement_pack.raw_measurements_);
        x << cartesian_xy(0),
             cartesian_xy(1),
             0,
             0;
    }
    else
    {
        x << measurement_pack.raw_measurements_[0],
             measurement_pack.raw_measurements_[1],
             0,
             0;
    }
    return x;
}


MatrixXd FusionEKF::CalculateTransitionCovariance(const float &dt)
{
    MatrixXd F(4,4);
    F << 1, 0, dt, 0,
         0, 1, 0, dt,
         0, 0, 1, 0,
         0, 0, 0, 1;
    return F;
}


MatrixXd FusionEKF::CalculateProcessCovariance(const float &dt)
{
    float dt2 = dt * dt;
    float dt3 = dt * dt2;
    float dt4 = dt * dt3;

    MatrixXd Q(4,4);
    Q << dt4/4*noise_ax, 0, dt3/2*noise_ax, 0,
         0, dt4/4*noise_ay, 0, dt3/2*noise_ay,
         dt3/2*noise_ax, 0, dt2*noise_ax, 0,
         0, dt3/2*noise_ay, 0, dt2*noise_ay;
    return Q;
}


MatrixXd FusionEKF::CalculateJacobian(const VectorXd &x_state)
{
    MatrixXd Hj(3,4);

    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    float c1 = px*px+py*py;
    float c2 = sqrt(c1);
    float c3 = (c1*c2);

    if (fabs(c1) < 0.0001)
    {
        cout << "CalculateJacobian() - Error - Division by Zero" << endl;
        return Hj;
    }

    Hj << (px/c2), (py/c2), 0, 0,
          -(py/c1), (px/c1), 0, 0,
          py*(vx*py - vy*px)/c3, px*(px*vy-py*vx)/c3, px/c2, py/c2;

    return Hj;
}
