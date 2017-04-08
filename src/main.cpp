
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

// Encapsulate STD values and RMSE, NIS results produced on one file
typedef struct {
  double rmse_x, rmse_y, rmse_vx, rmse_vy;
  double nis_total, nis_radar, nis_lidar;
  bool isInvalid() {
    return isnan(rmse_x) || isnan(rmse_y) || isnan(rmse_vx) || isnan(rmse_vy);
  }
} result_t;

ostream& operator << (ostream& os, result_t& result) {
  os << result.rmse_x << "\t" << result.rmse_y << "\t" << result.rmse_vx << "\t" << result.rmse_vy << "\t" << result.nis_total << "\t" << result.nis_radar << "\t" << result.nis_lidar << endl;
  return os;
}

// Null stream discards output - used in parameter search
struct nullostream : std::ostream {
  nullostream() : std::ios(0), std::ostream(0) {}
};
static nullostream null_out;

// Execute 
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
      out_file << ukf.NIS_laser_ << "\n";
      lidar_count++;
      if (ukf.NIS_laser_ > NIS_threshold)
        lidar_large_NIS++;
    }
    else if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR) {
      out_file << ukf.NIS_radar_ << "\n";
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
  result.rmse_vx = rmse(2);
  result.rmse_vy = rmse(3);
  result.nis_lidar = (lidar_large_NIS * 100.0 / lidar_count);
  result.nis_radar = (radar_large_NIS * 100.0 / radar_count);
  result.nis_total = ((lidar_large_NIS + radar_large_NIS) * 100.0) / (lidar_count + radar_count);
  return result;
}

struct var_range {
  double* pvar;
  double start;
  double end;
  double step;
  var_range(double* pvar, double start, double end, double step) : pvar(pvar), start(start), end(end), step(step) { }
};

bool isGridSearchDone(const vector<var_range>& variables) {
  for (int i = 0; i < variables.size(); i++) {
    if (*(variables[i].pvar) < variables[i].end)
      return  false;
  }
  return true;
}

void incrementGridSearch(const vector<var_range>& variables) {
  for (int i = 0; i < variables.size(); i++) {
    if (*(variables[i].pvar) < variables[i].end) {
      *(variables[i].pvar) += variables[i].step;
      return;
    }
    else {
      *(variables[i].pvar) = variables[i].start;
      continue;
    }
  }
}

void doGridSearch(vector<var_range>& variables, std::function<void(void)> fn)
{
  // Initialize all variables
  for_each(variables.begin(), variables.end(), [](var_range& x) { *(x.pvar) = x.start; });
  // Keep going till search is completed
  while (!isGridSearchDone(variables)) {
    // execute function
    fn();
    // increment to next search
    incrementGridSearch(variables);
  }
}

struct datafile {
  string filename;
  double limit_pxy, limit_vxy;
  result_t result;
  datafile(const string& fname, double lim_pxy, double lim_vxy) : filename(fname), limit_pxy(lim_pxy), limit_vxy(lim_vxy) { }

  bool isInvalid() {
    return (result.isInvalid() ||
      result.rmse_x > limit_pxy || result.rmse_y > limit_pxy ||
      result.rmse_vx > limit_vxy || result.rmse_vy > limit_vxy);
  }
};

int gridsearch(vector<datafile> datafiles, vector<var_range>& variables, const string& results_file_name) {

  ofstream result_file(results_file_name, ofstream::out);
  // column names for output file
  result_file << "std_a_"
    << "std_yawdd_\t"
    << "std_laspx_\t"
    << "std_laspy_\t"
    << "std_radr_\t"
    << "std_radphi_\t"
    << "std_radrd_\t";
  for (int i = 0; i < datafiles.size(); i++) {
    result_file << i + 1 << "rmse_px\t"
      << i + 1 << "rmse_py\t"
      << i + 1 << "rmse_vx\t"
      << i + 1 << "rmse_vy\t"
      << i + 1 << "nis_total\t"
      << i + 1 << "nis_lidar\t"
      << i + 1 << "nis_radar\t";
  }
  result_file << endl;
  
  doGridSearch(variables, [&datafiles, &result_file] () {
    cout << "std_a_:" << UKF::std_a_
      << ",std_yawdd_:" << UKF::std_yawdd_
      << ",std_laspx_:" << UKF::std_lasp_xy_
      << ",std_radr_:" << UKF::std_radr_
      << ",std_radphi_:" << UKF::std_radphi_
      << ",std_radrd_:" << UKF::std_radrd_ << endl;

    for (vector<datafile>::iterator df = datafiles.begin(); df != datafiles.end(); df++) {
      // process file 1
      ifstream in_file(df->filename, ifstream::in);
      if (!in_file.is_open()) throw new string("Unable to open file");
      df->result = process_file(in_file, null_out);
      in_file.close();
      // check results and save it if the numbers look good
      if(df->isInvalid()) return;
    }

    result_file << UKF::std_a_ << "\t"
      << UKF::std_yawdd_ << "\t"
      << UKF::std_lasp_xy_ << "\t"
      << UKF::std_radr_ << "\t"
      << UKF::std_radphi_ << "\t"
      << UKF::std_radrd_ << "\t";

    for (vector<datafile>::iterator df = datafiles.begin(); df != datafiles.end(); df++) {
      cout << "\t" << df->result;
      result_file << df->result << "\t";
    }
    cout << endl;
    result_file  << endl;
  });

  result_file.close();
  return 0;
}

int main(int argc, char** argv) {
  if (argc < 3) {
    cout << "Usage: \n\t<data_file> <results_file>\n\tgridsearch <results_file> {<data_file1> <limit_pxy> <limit_vxy>}+" << endl;
    return 0;
  }
  if (strncmp(argv[1], "gridsearch", 10) == 0) {
    if (argc < 6) {
      cout << "Usage: \n\tgridsearch <results_file> {<data_file1> <limit_pxy> <limit_vxy>}+" << endl;
      return 0;
    }

    vector<var_range> ranges;
    ranges.push_back(var_range(&UKF::std_a_, 0.05, 1.05, 0.2));
    ranges.push_back(var_range(&UKF::std_yawdd_, 0.05, 1.05, 0.2));
    ranges.push_back(var_range(&UKF::std_lasp_xy_, 0.05, 2.05, 0.25));
    ranges.push_back(var_range(&UKF::std_radr_, 0.05, 1.05, 0.2));
    ranges.push_back(var_range(&UKF::std_radphi_, 0.01, 0.51, 0.05));
    ranges.push_back(var_range(&UKF::std_radrd_, 0.05, 1.05, 0.2));

    vector<datafile> datafiles;
    for (int i = 3; i < argc; i += 3) {
      datafiles.push_back(datafile(argv[i], atof(argv[i + 1]), atof(argv[i + 2])));
    }

    gridsearch(datafiles, ranges, argv[2]);
  }
  else {
    // nan: std_a_:1.05,std_yawdd_:0.25,std_laspx_:0.55,std_radr_:0.25,std_radphi_:0.01,std_radrd_:0.050 

    /* Values selected from scan results */
    UKF::std_a_ = 1.05;//0.3;//0.2;
    UKF::std_yawdd_ = 0.25;//0.3;// 0.2;
    UKF::std_lasp_xy_ = 0.55;//0.15;//0.15;
    UKF::std_radr_ = 0.25; //0.5; // 0.3;
    UKF::std_radphi_ = 0.01; //0.07; // 0.0175;
    UKF::std_radrd_ = 0.05; //0.6; //0.1;

    // process one file
    ifstream in_file(argv[1], ifstream::in);
    ofstream out_file(argv[2], ofstream::out);
    result_t result = process_file(in_file, out_file);
    in_file.close();
    out_file.close();

    cout << "RMSE: " << result << endl;
  }
}
