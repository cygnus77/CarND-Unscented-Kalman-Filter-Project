#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
  const vector<VectorXd> &ground_truth) {
  /**
  * Calculate the RMSE here.
  */
  VectorXd rmse(2);
  rmse.setZero();

  if (estimations.size() != ground_truth.size() || estimations.size() == 0) return rmse;

  //accumulate squared residuals
  VectorXd residual(2);
  residual.setZero();
  for (int i = 0; i < 2; ++i) {
    VectorXd d = estimations[i] - ground_truth[i];
    residual = residual.array() + (d.array() * d.array());
  }
  residual /= estimations.size();
  rmse = residual.array().sqrt();

  return rmse;
}

Eigen::VectorXd Tools::RadarToCTRV(const Eigen::VectorXd &z)
{
  double rho = z(0);
  double phi = z(1);
  double rhodot = z(2);

  double py = rho * sin(phi);
  double px = rho * cos(phi);
  double v = rhodot;
  double psi = phi;
  double psidot = 0;

  VectorXd result(5);
  result << px, py, v, psi, psidot;
  return result;
}
