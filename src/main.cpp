
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "ukf.h"
#include "ground_truth_package.h"
#include "measurement_package.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

typedef struct {
  double rmse_x, rmse_y, rmse_px, rmse_py;
  double nis_total, nis_radar, nis_lidar;
} result_t;

struct nullostream : std::ostream {
  nullostream() : std::ios(0), std::ostream(0) {}
};

static nullostream null_out;

result_t process_file(istream& in_file, ostream& out_file) {

  int lidar_large_NIS = 0, radar_large_NIS = 0;
  int lidar_count = 0, radar_count = 0;
  const double NIS_threshold = 7.8;

  /**********************************************
   *  Set Measurements                          *
   **********************************************/

  vector<MeasurementPackage> measurement_pack_list;
  vector<GroundTruthPackage> gt_pack_list;

  string line;

  // prep the measurement packages (each line represents a measurement at a
  // timestamp)
  while (getline(in_file, line)) {
    string sensor_type;
    MeasurementPackage meas_package;
    GroundTruthPackage gt_package;
    istringstream iss(line);
    long long timestamp;

    // reads first element from the current line
    iss >> sensor_type;

    if (sensor_type.compare("L") == 0) {
      // laser measurement

      // read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = VectorXd(2);
      float px;
      float py;
      iss >> px;
      iss >> py;
      meas_package.raw_measurements_ << px, py;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    }
    else if (sensor_type.compare("R") == 0) {
      // radar measurement

      // read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::RADAR;
      meas_package.raw_measurements_ = VectorXd(3);
      float ro;
      float phi;
      float ro_dot;
      iss >> ro;
      iss >> phi;
      iss >> ro_dot;
      meas_package.raw_measurements_ << ro, phi, ro_dot;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    }

    // read ground truth data to compare later
    float x_gt;
    float y_gt;
    float vx_gt;
    float vy_gt;
    iss >> x_gt;
    iss >> y_gt;
    iss >> vx_gt;
    iss >> vy_gt;
    gt_package.gt_values_ = VectorXd(4);
    gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
    gt_pack_list.push_back(gt_package);
  }

  // Create a UKF instance
  UKF ukf;

  // used to compute the RMSE later
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  // start filtering from the second frame (the speed is unknown in the first
  // frame)

  size_t number_of_measurements = measurement_pack_list.size();

  //// column names for output file
  out_file << "type" << "\t";
  out_file << "px" << "\t";
  out_file << "py" << "\t";
  out_file << "v" << "\t";
  out_file << "yaw_angle" << "\t";
  out_file << "yaw_rate" << "\t";
  out_file << "px_measured" << "\t";
  out_file << "py_measured" << "\t";
  out_file << "px_true" << "\t";
  out_file << "py_true" << "\t";
  out_file << "vx_true" << "\t";
  out_file << "vy_true" << "\t";
  out_file << "NIS" << "\n";


  for (size_t k = 0; k < number_of_measurements; ++k) {

    // Call the UKF-based fusion
    ukf.ProcessMeasurement(measurement_pack_list[k]);
    // output the estimation
    out_file << (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER ? "L\t" : "R\t");
    out_file << ukf.x_(0) << "\t"; // pos1 - est
    out_file << ukf.x_(1) << "\t"; // pos2 - est
    out_file << ukf.x_(2) << "\t"; // vel_abs -est
    out_file << ukf.x_(3) << "\t"; // yaw_angle -est
    out_file << ukf.x_(4) << "\t"; // yaw_rate -est

    // output the measurements
    if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER) {
      // output the estimation

      // p1 - meas
      out_file << measurement_pack_list[k].raw_measurements_(0) << "\t";

      // p2 - meas
      out_file << measurement_pack_list[k].raw_measurements_(1) << "\t";
    }
    else if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR) {
      // output the estimation in the cartesian coordinates
      float ro = measurement_pack_list[k].raw_measurements_(0);
      float phi = measurement_pack_list[k].raw_measurements_(1);
      out_file << ro * cos(phi) << "\t"; // p1_meas
      out_file << ro * sin(phi) << "\t"; // p2_meas
    }

    // output the ground truth packages
    out_file << gt_pack_list[k].gt_values_(0) << "\t";
    out_file << gt_pack_list[k].gt_values_(1) << "\t";
    out_file << gt_pack_list[k].gt_values_(2) << "\t";
    out_file << gt_pack_list[k].gt_values_(3) << "\t";
    // output the NIS values

    if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER) {
      //out_file << ukf.NIS_laser_ << "\n";
      lidar_count++;
      if (ukf.NIS_laser_ > NIS_threshold)
        lidar_large_NIS++;
    }
    else if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR) {
      //out_file << ukf.NIS_radar_ << "\n";
      radar_count++;
      if (ukf.NIS_radar_ > NIS_threshold)
        radar_large_NIS++;
    }


    // convert ukf x vector to cartesian to compare to ground truth
    VectorXd ukf_x_cartesian_ = VectorXd(4);

    float x_estimate_ = ukf.x_(0);
    float y_estimate_ = ukf.x_(1);
    float vx_estimate_ = ukf.x_(2) * cos(ukf.x_(3));
    float vy_estimate_ = ukf.x_(2) * sin(ukf.x_(3));

    ukf_x_cartesian_ << x_estimate_, y_estimate_, vx_estimate_, vy_estimate_;

    estimations.push_back(ukf_x_cartesian_);
    ground_truth.push_back(gt_pack_list[k].gt_values_);

  }

  // compute the accuracy (RMSE)
  Tools tools;
  VectorXd rmse = tools.CalculateRMSE(estimations, ground_truth);

  result_t result;
  result.rmse_x = rmse(0);
  result.rmse_y = rmse(1);
  result.rmse_px = rmse(2);
  result.rmse_py = rmse(3);
  result.nis_lidar = (lidar_large_NIS * 100.0 / lidar_count);
  result.nis_radar = (radar_large_NIS * 100.0 / radar_count);
  result.nis_total = ((lidar_large_NIS + radar_large_NIS) * 100.0) / (lidar_count + radar_count);
  return result;
}

int scan() {
#define DIR_LOC "C:\\Users\\Anand\\Desktop\\CarND-Unscented-Kalman-Filter-Project\\data\\"

  UKF::std_a_ = 0.2;//0.3;//0.2;
  UKF::std_yawdd_ = 0.2;//0.3;// 0.2;
  UKF::std_laspx_ = 0.15;//0.15;//0.15;
  UKF::std_laspy_ = 0.15;//0.15;// 0.15;
  UKF::std_radr_ = 0.3; //0.5; // 0.3;
  UKF::std_radphi_ = 0.0175; //0.07; // 0.0175;
  UKF::std_radrd_ = 0.1; //0.6; //0.1;

  ofstream result_file(DIR_LOC"good_values.txt", ofstream::out);
  // column names for output file
  result_file << "std_a_:"
    << ",std_yawdd_\t"
    << ",std_laspx_\t"
    << ",std_laspy_\t"
    << ",std_radr_\t"
    << ",std_radphi_\t"
    << ",std_radrd_\t"
    << "rmse_x \t"
    << "1.rmse_y   \t"
    << "1.nis_total\t"
    << "1.nis_lidar\t"
    << "1.nis_radar\t"
    << "2.rmse_x   \t"
    << "2.rmse_y   \t"
    << "2.nis_total\t"
    << "2.nis_lidar\t"
    << "2.nis_radar"
    << endl;

  for (UKF::std_a_ = 0.05; UKF::std_a_ < 1.05; UKF::std_a_ += 0.2) {
    for (UKF::std_yawdd_ = 0.05; UKF::std_yawdd_ < 1.05; UKF::std_yawdd_ += 0.2) {
      for (UKF::std_laspx_ = UKF::std_laspy_ = 0.05; UKF::std_laspx_ < 2.05; UKF::std_laspx_ += 0.25, UKF::std_laspy_ += 0.25) {
        for (UKF::std_radr_ = 0.05; UKF::std_radr_ < 1.05; UKF::std_radr_ += 0.2) {
          for (UKF::std_radphi_ = 0.01; UKF::std_radphi_ < 0.51; UKF::std_radphi_ += 0.05) {
            for (UKF::std_radrd_ = 0.05; UKF::std_radrd_ < 1.05; UKF::std_radrd_ += 0.2) {


              ifstream in_file2(DIR_LOC"sample-laser-radar-measurement-data-2.txt", ifstream::in);
              
              result_t result2 = process_file(in_file2, null_out);

              in_file2.close();

              if (result2.rmse_x > 0.2 || result2.rmse_y > 0.2) continue;

              ifstream in_file1(DIR_LOC"sample-laser-radar-measurement-data-1.txt", ifstream::in);
              
              result_t result1 = process_file(in_file1, null_out);

              in_file1.close();

              if (result1.rmse_x > 0.09 || result1.rmse_y > 0.09) continue;
              cout << "std_a_:" << UKF::std_a_
                << ",std_yawdd_:" << UKF::std_yawdd_
                << ",std_laspx_:" << UKF::std_laspx_
                << ",std_laspy_:" << UKF::std_laspy_
                << ",std_radr_:" << UKF::std_radr_
                << ",std_radphi_:" << UKF::std_radphi_
                << ",std_radrd_:" << UKF::std_radrd_
                << endl;

              result_file << UKF::std_a_ << "\t"
                << UKF::std_yawdd_ << "\t"
                << UKF::std_laspx_ << "\t"
                << UKF::std_laspy_ << "\t"
                << UKF::std_radr_ << "\t"
                << UKF::std_radphi_ << "\t"
                << UKF::std_radrd_ << "\t"
                << result1.rmse_x << "\t"
                << result1.rmse_y << "\t"
                << result1.nis_total << "\t"
                << result1.nis_lidar << "\t"
                << result1.nis_radar << "\t"
                << result2.rmse_x << "\t"
                << result2.rmse_y << "\t"
                << result2.nis_total << "\t"
                << result2.nis_lidar << "\t"
                << result2.nis_radar << endl;
            }
          }
        }
      }
    }
  }
  result_file.close();
  return 0;
}

int main(int argc, char** argv) {
  if (argc < 2) {
    cout << "Usage: " << endl;
    return 0;
  }

  string cmd(argv[0]);
  if (cmd == "scan") {
    scan();
  }
  else if (cmd == "eval") {
    ifstream values_file(argv[2], ifstream::in);
    string line;
    while (getline(values_file, line)) {
      // Parse values, apply to statics,
      // exectue process on both files
    }
    values_file.close();
  }
  else {
    // process one file
    ifstream in_file(argv[1], ifstream::in);
    ofstream out_file(argv[2], ofstream::out);
    process_file(in_file, out_file);
    in_file.close();
    out_file.close();
  }
}