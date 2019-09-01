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
    cout<<"Tools::CalculateRMSE()"<<endl;

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

    rmse = rmse / estimations.size();
    rmse = rmse.array().sqrt();

    return rmse;
}


VectorXd Tools::ConvertPolar2Cartesian(const VectorXd &raw_measurement)
{
    float rho = raw_measurement[0];
    float phi = raw_measurement[1];

    VectorXd xy(2);
    xy << rho * cos(phi),
          rho * sin(phi);

    return xy;
}


VectorXd Tools::ConvertCartesian2Polar(const VectorXd &x_pred)
{
    float px = x_pred(0);
    float py = x_pred(1);
    float vx = x_pred(2);
    float vy = x_pred(3);

    float rho_pred = sqrt(pow(px,2) + pow(py,2));
    if (rho_pred < 0.0001)
    {
        rho_pred = 0.0001;
    }

    VectorXd z_pred(3);
    z_pred << rho_pred,
              atan2(py, px),
              (px*vx + py*vy) / rho_pred;
    return z_pred;
}
