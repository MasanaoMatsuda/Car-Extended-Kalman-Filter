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
    Hj_ = MatrixXd(3,4);

    // measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
               0, 0.0225;
    // measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
                0, 0.0009, 0,
                0, 0, 0.09;
    //
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;

    P_ = MatrixXd(4,4);
    P_ << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1000, 0,
          0, 0, 0, 1000;

    F_ = MatrixXd(4,4);
    F_ << 1, 0, 1, 0,
          0, 1, 0, 1,
          0, 0, 1, 0,
          0, 0, 0, 1;

    Q_ = MatrixXd(4,4);

    x_ = VectorXd(4);
}


FusionEKF::~FusionEKF()
{
}


void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{
    if (!is_initialized_)
    {
        cout << "EKF: " << endl;

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
        {
            x_ = tools.ConvertPolar2Cartesian(measurement_pack.raw_measurements_);
            ekf_.Init(x_, P_, F_, Hj_, R_radar_, Q_);
            return;
        }
        else
        {
            x_ << measurement_pack.raw_measurements_[0],
                  measurement_pack.raw_measurements_[1],
                  0,
                  0;
            ekf_.Init(x_, P_, F_, H_laser_, R_laser_, Q_);
        }

        previous_timestamp_ = measurement_pack.timestamp_;
        is_initialized_ = true;

        return;
    }

    float dt = (measurement_pack.timestamp_ - previous_timestamp_) * 1000000.0; // express in sec.
    F_ = CalculateTransitionCovariance(dt);
    Q_ = CalculateProcessCovariance(dt);

    ekf_.Predict();

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
        Hj_ = tools.CalculateJacobian(measurement_pack.raw_measurements_);

        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
        return;
    }
    else
    {
        ekf_.H_ = H_laser_;
        ekf_.R_ = R_laser_;
        ekf_.Update(measurement_pack.raw_measurements_);
    }

    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}


MatrixXd CalculateTransitionCovariance(const float &dt)
{
    MatrixXd F(4,4);
    F << 1, 0, dt, 0,
         0, 1, 0, dt,
         0, 0, 1, 0,
         0, 0, 0, 1;
    return F;
}


MatrixXd CalculateProcessCovariance(const float &dt)
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
