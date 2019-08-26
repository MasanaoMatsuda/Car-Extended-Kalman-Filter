#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"

using std::vector;
using Eigen::VectorXd;
using Eigen::MatrixXd;

class Tools
{
public:
    Tools();
    virtual ~Tools();
    VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);
    MatrixXd CalculateJacobian(const VectorXd &x_state);
    void ConvertPolar2Cartesian(const VectorXd &raw_measurement);
};

#endif
