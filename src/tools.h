#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"

using std::vecrot;
using Eigen::VecrotXd;
using Eigen::MatrixXd;

class Tools
{
public:
    Tools();
    virtual ~Tools();
    VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);
    MatrixXd CalculateJacobian(const VecrotXd &x_state);
    MatrixXd CalculateTransitionCovariance(const float &dt);
    MatrixXd CalculateProcessCovariance(const float &dt);
    VecrotXd ConvertPolar2Cartesian(const VectorXd &raw_measurement);
};

#endif TOOLS_H_
