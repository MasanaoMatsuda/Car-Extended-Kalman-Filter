#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include <math.h>
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
    VectorXd ConvertPolar2Cartesian(const VectorXd &raw_measurement);
    VectorXd ConvertCartesian2Polar(const VectorXd &x_pred);
};

#endif
