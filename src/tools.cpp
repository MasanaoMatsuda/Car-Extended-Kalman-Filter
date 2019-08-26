#include "tools.h"
#include <iostream>

using std::cout;
using std::endl;

Tools::Tools()
{
}

Tools::~Tools()
{
}


VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth)
{
    VectorXd rmse(4);
    rmse << 0,0,0,0;

    if (estimations.size() != ground_truth.size() || estimations.size() == 0)
    {
        cout << "Invalid estimation or ground_truth data.." << endl;
        return rmse;
    }

    for (int i = 0; i < estimations.size(); ++i)
    {
        VectorXd residual = estimations[i] - ground_truth[i];
        residual = residual.array() * residual.array(); // coefficient-wise multiplication
        rmse += residual;
    }

    rmse = rmse / estimation.size();
    rmse = rmse.array().sqrt();

    return rmse;
}


MatrixXd Tools::CalculateJacobian(const VectorXd &x_state)
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


VectorXd Tools::ConvertPolar2Cartesian(const VectorXd &raw_measurement)
{
    float rho = raw_measurement[0];
    float phi = raw_measurement[1];
    float rho_dot = raw_measurement[2];

    return null;
}
