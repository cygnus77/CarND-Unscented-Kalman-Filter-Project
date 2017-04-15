#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* 
  MatrixXd Radar_R_, Lidar_R_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  static double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  static double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 & 2 in m
  static double std_lasp_xy_;

  ///* Radar measurement noise standard deviation radius in m
  static double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  static double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  static double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;
  MatrixXd weights2d_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* No of sigma points
  int n_sig_;

  ///* Sigma point spreading parameter
  double lambda_;

  ///* the current NIS for radar
  double NIS_radar_;

  ///* the current NIS for laser
  double NIS_laser_;

  ///* previous timestamp
  long long previous_timestamp_;

  const int LIDAR_MEAS_DIM = 2, RADAR_MEAS_DIM = 3;

  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);
};

#endif /* UKF_H */
