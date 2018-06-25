#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO - Finished:
    * Calculate the RMSE here.
  */

  VectorXd rmse(4); // The RMSE is a four row vector for [Px, Py, Vx, Vy]
  rmse << 0, 0, 0, 0;

  // First, to check validity of input:
  //  the sizes of two input vector should be the same

  if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
    std::cout << "The input for estimation is not valid, please check the size of the estimation input vector" << endl;
    return rmse;
  }

  for (int i = 0; i < estimations.size(); ++i) {

    VectorXd errorVec = estimations[i] - ground_truth[i];
    errorVec = errorVec.array() * errorVec.array();
    rmse += errorVec;
  }
  VectorXd lastVec = estimations[estimations.size()-1] - ground_truth[estimations.size()-1];
  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();

  std::cout<<"RMSE is:"<<rmse(0)<<','<<rmse(1)<<','<<rmse(2)<<','<<rmse(3)<<endl;

  std::cout<<"The error in last run is:" << lastVec<< endl;
  return rmse;
}