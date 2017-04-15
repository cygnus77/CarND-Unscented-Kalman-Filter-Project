#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

double UKF::std_a_;
double UKF::std_yawdd_;
double UKF::std_lasp_xy_;
double UKF::std_radr_;
double UKF::std_radphi_;
double UKF::std_radrd_;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  /**
  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  //set state dimension
  n_x_ = 5;

  n_aug_ = 7;

  // No sigma points
  n_sig_ = 2 * n_aug_ + 1;

  lambda_ = 3 - n_aug_;

  weights_ = VectorXd(n_sig_);
  weights2d_ = MatrixXd(n_sig_, n_sig_);
  weights2d_.setZero();

  // pre-compute weights
  double weight_0 = lambda_ / (lambda_ + n_aug_);
  weights_(0) = weight_0;
  weights2d_(0, 0) = weight_0;
  for (int i = 1; i < n_sig_; i++) {  //2n+1 weights
    double weight = 0.5 / (n_aug_ + lambda_);
    weights_(i) = weight;
    weights2d_(i, i) = weight;
  }

  //set measurement dimension, lidar can measure x & y
  Lidar_R_ = MatrixXd(LIDAR_MEAS_DIM, LIDAR_MEAS_DIM);
  Lidar_R_.setZero();
  Lidar_R_.diagonal() << std_lasp_xy_*std_lasp_xy_, std_lasp_xy_*std_lasp_xy_;

  //set measurement dimension, radar can measure r, phi, and r_dot
  Radar_R_ = MatrixXd(RADAR_MEAS_DIM, RADAR_MEAS_DIM);
  Radar_R_.setZero();
  Radar_R_.diagonal() << std_radr_*std_radr_, std_radphi_*std_radphi_, std_radrd_*std_radrd_;

  NIS_laser_ = 0;
  NIS_radar_ = 0;

  is_initialized_ = false;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 * Main calls this with new measurement
 */
void UKF::ProcessMeasurement(MeasurementPackage measurement_pack) {
  /**
  Switch between laser / radar measurement
  */
  if ((!use_laser_ && measurement_pack.sensor_type_ == MeasurementPackage::LASER) ||
    (!use_radar_ && measurement_pack.sensor_type_ == MeasurementPackage::RADAR)) {
    return;
  }

  if (measurement_pack.raw_measurements_(0) == 0 && measurement_pack.raw_measurements_(1) == 0)
    measurement_pack.raw_measurements_(0) = measurement_pack.raw_measurements_(1) = 1e-4;

  // Initialize with first measurement
  if (!this->is_initialized_) {

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      x_ = Tools::RadarToCTRV(measurement_pack.raw_measurements_);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0, 0;
    }

    // Initialize uncertainty covariance matrix
    P_ << 1, 0, 0, 0, 0,
      0, 1, 0, 0, 0,
      0, 0, 1, 0, 0,
      0, 0, 0, 1, 0,
      0, 0, 0, 0, 1;

    this->is_initialized_ = true;
  }
  else {
    //compute the time elapsed between the current and previous measurements
    double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds

    // Prediction
    Prediction(dt);

    // Update
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      UpdateRadar(measurement_pack);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      UpdateLidar(measurement_pack);
    }

  }
  previous_timestamp_ = measurement_pack.timestamp_;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  // Code from Augmentaton Assignment - 1

  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.setZero();

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.setZero();

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_);

  //create augmented mean state
  x_aug.topLeftCorner(5, 1) = x_;

  //create augmented covariance matrix
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  MatrixXd Q = MatrixXd(2, 2);
  Q << std_a_*std_a_, 0, 0, std_yawdd_*std_yawdd_;
  P_aug.bottomRightCorner(2, 2) = Q;


  //create square root matrix
  MatrixXd A = P_aug.llt().matrixL();
  MatrixXd B = sqrt(lambda_ + n_aug_)*A;

  //create augmented sigma points - without explicit looping
  Xsig_aug.col(0) = x_aug;
  Xsig_aug.block(0, 1, n_aug_, n_aug_) = B.colwise() + x_aug;
  Xsig_aug.block(0, n_aug_ + 1, n_aug_, n_aug_) = -(B.colwise() - x_aug);

  //// Code from Sigma Point Prediction Assignment - 1

  //create matrix with predicted sigma points as columns
  Xsig_pred_ = MatrixXd(n_x_, n_sig_);

  //predict sigma points
  for (int i = 0; i < Xsig_aug.cols(); i++) {
    VectorXd x = Xsig_aug.col(i);
    double cos_phi = cos(x(3));
    double sin_phi = sin(x(3));
    double v = x(2);
    double phi = x(3);
    double phi_dot = x(4);
    double nu_a = x(5);
    double nu_phi_dd = x(6);

    VectorXd A = VectorXd(5);
    if (phi_dot == 0) {
      A << v*cos_phi*delta_t,
        v*sin_phi*delta_t,
        0,
        0, //phi_dot*delta_t,
        0;
    }
    else {
      A << (v / phi_dot)*(sin(phi + phi_dot*delta_t) - sin_phi),
        (v / phi_dot)*(cos_phi - cos(phi + phi_dot*delta_t)),
        0,
        phi_dot*delta_t,
        0;
    }
    VectorXd N = VectorXd(5);
    N << 0.5*delta_t*delta_t*cos_phi*nu_a,
      0.5*delta_t*delta_t*sin_phi*nu_a,
      delta_t*nu_a,
      0.5*delta_t*delta_t*nu_phi_dd,
      delta_t*nu_phi_dd;
    Xsig_pred_.col(i) = x.topRows(5) + A + N;
  }

  //// Code from Predicted Mean and Covariance Assignment - 1

  //create vector for predicted state
  VectorXd x = VectorXd(n_x_);
  x.setZero();

  //predict state mean
  x += (Xsig_pred_*weights2d_).rowwise().sum();

  //create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x_, n_x_);
  auto Pview = P.setZero().selfadjointView<Eigen::Lower>();
  //predict state covariance matrix
  for (int i = 0; i < Xsig_pred_.cols(); i++) {
    // efficient way to calculate W*M*Mt
    Pview.rankUpdate(Xsig_pred_.col(i) - x, weights_(i));
  }

  x_ = x;
  P_ = Pview;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  //// Code from Predict Radar Measurement - Assignment 1

  //create matrix for sigma points in measurement space
  //transform sigma points into measurement space  
  MatrixXd Zsig = Xsig_pred_.block(0, 0, 2, Xsig_pred_.cols());

  //mean predicted measurement
  //calculate mean predicted measurement
  VectorXd z_pred = (Zsig * weights2d_).rowwise().sum();

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(LIDAR_MEAS_DIM, LIDAR_MEAS_DIM);
  //calculate measurement covariance matrix S
  auto Sview = S.setZero().selfadjointView<Eigen::Lower>();
  for (int i = 0; i < Xsig_pred_.cols(); i++) {
    //efficient way of doing: S += weights_(i) * (Zsig.col(i) - z_pred) * ((Zsig.col(i) - z_pred).transpose());
    Sview.rankUpdate((Zsig.col(i) - z_pred), weights_(i));
  }
  S = Sview;
  S += Lidar_R_;

  //// Code from UKF Update Assignment - 1
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, LIDAR_MEAS_DIM);
  Tc.setZero();

  //calculate cross correlation matrix
  for (int i = 0; i < Zsig.cols(); i++) {
    Tc += weights_(i) * (Xsig_pred_.col(i) - x_) * ((Zsig.col(i) - z_pred).transpose());
  }
  //calculate Kalman gain K;
  MatrixXd Sinv = S.inverse();
  MatrixXd K = Tc * Sinv;

  MatrixXd z_diff = meas_package.raw_measurements_ - z_pred;

  NIS_laser_ = (z_diff.transpose() * Sinv * z_diff)(0);

  //update state mean and covariance matrix
  x_ = x_ + K*z_diff;
  P_ = P_ - K*S*(K.transpose());
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  //// Code from Predict Radar Measurement - Assignment 1

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(RADAR_MEAS_DIM, n_sig_);

  //transform sigma points into measurement space
  for (int i = 0; i < Xsig_pred_.cols(); i++) {
    VectorXd x = Xsig_pred_.col(i);
    double px = x(0), py = x(1), v = x(2), psi = x(3);
    double rho = sqrt(x(0)*x(0) + x(1)*x(1));
    double phi = atan2(py, px);
    // normalize phi without explicit loop
    phi = atan2(sin(phi), cos(phi));
    double rhodot = (px*cos(psi)*v + py*sin(psi)*v) / rho;
    Zsig.col(i) << rho, phi, rhodot;
  }

  //mean predicted measurement
  //calculate mean predicted measurement
  VectorXd z_pred = (Zsig * weights2d_).rowwise().sum();

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(RADAR_MEAS_DIM, RADAR_MEAS_DIM);
  auto Sview = S.setZero().selfadjointView<Eigen::Lower>();
  for (int i = 0; i < Xsig_pred_.cols(); i++) {
    //efficient way to do: S += weights_(i) * (Zsig.col(i) - z_pred) * ((Zsig.col(i) - z_pred).transpose());
    Sview.rankUpdate(Zsig.col(i) - z_pred, weights_(i));
  }
  S = Sview;
  S += Radar_R_;

  //// Code from UKF Update Assignment - 1
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, RADAR_MEAS_DIM);
  Tc.setZero();

  //calculate cross correlation matrix
  for (int i = 0; i < Zsig.cols(); i++) {
    Tc += weights_(i) * (Xsig_pred_.col(i) - x_) * ((Zsig.col(i) - z_pred).transpose());
  }

  //calculate Kalman gain K;
  MatrixXd Sinv = S.inverse();
  MatrixXd K = Tc * Sinv;

  MatrixXd z_diff = meas_package.raw_measurements_ - z_pred;

  //angle normalization
  z_diff(1) = atan2(sin(z_diff(1)), cos(z_diff(1)));

  NIS_radar_ = (z_diff.transpose() * Sinv * z_diff)(0);

  //update state mean and covariance matrix
  x_ = x_ + K*z_diff;
  P_ = P_ - K*S*(K.transpose());

}
