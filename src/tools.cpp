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
  TODO:
    * Calculate the RMSE here.
  */
  Eigen::VectorXd sum;
  sum = VectorXd(4);
  sum << 0, 0, 0, 0;
  for (int i=0; i < estimations.size(); i++)
  {
    Eigen::VectorXd diff = estimations[i] - ground_truth[i];
    sum += diff.cwiseProduct(diff);
  }

  return (sum / estimations.size()).array().sqrt();
}
