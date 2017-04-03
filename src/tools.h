#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

class Tools {
public:

  /**
  * A helper method to calculate RMSE - just px and py
  */
  static Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);

  /**
  * A helper function to map radar's polar space (rho, phi, rhodot) back to CTRV state (px,py,v,psi,psidot)
  */
  static Eigen::VectorXd RadarToCTRV(const Eigen::VectorXd &z);

};

#endif /* TOOLS_H_ */
