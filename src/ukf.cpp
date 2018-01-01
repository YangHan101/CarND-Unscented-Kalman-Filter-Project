#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // std::cout << "Constructor Start:" << '\n';

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1;

  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_aug_;
  n_aug_sig_ = 2 * n_aug_ + 1;

  Xsig_pred_ = MatrixXd(n_x_, n_aug_sig_);

  weights_ = VectorXd(n_aug_sig_);
  weights_(0) = lambda_ * 1.0 / (lambda_ + n_aug_);
  double weights_others_ = 0.5 / (lambda_ + n_aug_);

  for (int i = 0; i < n_aug_; i++)
  {
    weights_(i + 1) = weights_others_;
    weights_(i + n_aug_ + 1) = weights_others_;
  }
  // std::cout << "weights =" << weights_.transpose() << '\n';
  // std::cout << "Constructor End" << '\n';

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  // std::cout << "Processing the messages" << '\n';
   if (!is_initialized_)
   {
     x_ = VectorXd(5);
     x_ << 0, 0, 0, 0, 0;

     P_ = MatrixXd(5, 5);
     P_ << 1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 10, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 0.2;
     if ((meas_package.sensor_type_ == MeasurementPackage::RADAR) && use_radar_)
     {
       /**
       Convert radar from polar to cartesian coordinates and initialize state.
       */
       double rho_ = meas_package.raw_measurements_(0);
       double theta_ = meas_package.raw_measurements_(1);
       double rho_dot_ = meas_package.raw_measurements_(2);
       double cos_theta_ = std::cos(theta_);
       double sin_theta_ = std::sin(theta_);

       x_(0) = rho_ * cos_theta_;
       x_(1) = rho_ * sin_theta_;
       x_(2) = rho_dot_;
       x_(3) = 0;
       is_initialized_ = true;
     }
     else if ((meas_package.sensor_type_ == MeasurementPackage::LASER) && use_laser_)
     {
       /**
       Initialize state.
       */
       x_(0) = meas_package.raw_measurements_(0);
       x_(1) = meas_package.raw_measurements_(1);
       is_initialized_ = true;
     }

     previous_timestamp_ = meas_package.timestamp_;

     // done initializing, no need to predict or update

     // std::cout << "Initialization success" << '\n';
     // std::cout << "x =" << x_ << '\n';
     // std::cout << "P =" << P_ << '\n';
     return;
   }

   // std::cout << "New timestamp = " << meas_package.timestamp_ << '\n';
   // std::cout << "Old timestamp = " << previous_timestamp_ << '\n';
   double delta_t = (meas_package.timestamp_ - previous_timestamp_) * 1e-6;

   previous_timestamp_ = meas_package.timestamp_;
   // std::cout << "delta_t = " << delta_t << '\n';
   Prediction(delta_t);
   // std::cout << "Prediction success:" << '\n';
   // std::cout << "x =" << x_ << '\n';
   // std::cout << "P =" << P_ << '\n';

   if ((meas_package.sensor_type_ == MeasurementPackage::LASER) && use_laser_)
   {
     UpdateLidar(meas_package);
     // std::cout << "Updata Type: LASER" << '\n';
   }
   else if ((meas_package.sensor_type_ == MeasurementPackage::RADAR) && use_radar_)
   {
     UpdateRadar(meas_package);
     // std::cout << "Update Type: RADAR" << '\n';

   }
   // std::cout << "Update success:" << '\n';
   // std::cout << "x =" << x_ << '\n';
   // std::cout << "P =" << P_ << '\n';
   // std::cout << "***********************************************************************" << '\n';

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t)
{
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  // std::cout << "Prediction Started..." << '\n';

  MatrixXd P_aug_ = MatrixXd::Zero(n_aug_, n_aug_);

  P_aug_.topLeftCorner(n_x_, n_x_)  = P_;
  P_aug_(n_x_, n_x_) = std_a_ * std_a_;
  P_aug_(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;
  // std::cout << "P_aug =" << P_aug_ << '\n';


  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.head(n_x_) = x_;
  for (int i = n_x_; i < n_aug_; i++)
  {
    x_aug(i) = 0;
  }

  MatrixXd A = P_aug_.llt().matrixL();

  MatrixXd Xsig_ = MatrixXd(n_aug_, n_aug_sig_);
  Xsig_.col(0) = x_aug;

  for (int i = 0; i < n_aug_; i++)
  {
    Xsig_.col(i + 1) = x_aug + A.col(i) * std::sqrt(lambda_ + n_aug_);
    Xsig_.col(i + n_aug_ + 1) = x_aug - A.col(i) * std::sqrt(lambda_ + n_aug_);
  }

  // std::cout << "Xsig Computed" << '\n';
  // std::cout << "Xsig =" << Xsig_ << '\n';

  VectorXd x_pred_ = VectorXd(n_x_);
  x_pred_ << 0, 0, 0, 0, 0;
  // std::cout << "x_pred_ init" << x_pred_.transpose()<< '\n';

  for (int i = 0; i < n_aug_sig_; i++)
  {
    VectorXd x_sig_ = Xsig_.col(i);
    VectorXd x_prop_ = Xsig_.col(i);

    if (fabs(x_sig_(4)) < 0.001)
    {
      x_prop_(0) += x_sig_(2) * cos(x_sig_(3)) * delta_t;
      x_prop_(1) += x_sig_(2) * sin(x_sig_(3)) * delta_t;
    }
    else {
      x_prop_(0) += x_sig_(2) / x_sig_(4) * (sin(x_sig_(3) + x_sig_(4) * delta_t) - sin(x_sig_(3)));
      x_prop_(1) += x_sig_(2) / x_sig_(4) * (cos(x_sig_(3)) - cos(x_sig_(3) + x_sig_(4) * delta_t));
      x_prop_(3) += x_sig_(4) * delta_t;
    }

    x_prop_(0) += 0.5 * std::pow(delta_t, 2) * cos(x_sig_(3)) * x_sig_(5);
    x_prop_(1) += 0.5 * std::pow(delta_t, 2) * sin(x_sig_(3)) * x_sig_(5);
    x_prop_(2) += delta_t * x_sig_(5);
    x_prop_(3) += 0.5 * std::pow(delta_t, 2) * x_sig_(6);
    x_prop_(4) += delta_t * x_sig_(6);

    Xsig_pred_.col(i) = x_prop_.head(5);

    x_pred_ += weights_(i) * Xsig_pred_.col(i);
  }

  // std::cout << "x_pred_ computed" << '\n';
  // std::cout << "x_pred_ =" << x_pred_.transpose() << '\n';
  // std::cout << "Xsig_pred_ =" << Xsig_pred_ << '\n';

  MatrixXd P_pred_ = MatrixXd::Zero(n_x_, n_x_);
  for (int i = 0; i < n_aug_sig_; i++)
  {
    VectorXd col = Xsig_pred_.col(i) - x_pred_;
    //std::cout << "col =" << col.transpose() << '\n';
    //std::cout << "P_pred_ =" << P_pred_ << '\n';
    P_pred_ += weights_(i) * col * col.transpose();
  }

  // std::cout << "P_pred_ computed" << '\n';
  x_ = x_pred_;
  P_ = P_pred_;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package)
{
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  int n_z_ = 2;

  VectorXd z_ = VectorXd(n_z_);
  double px_ = meas_package.raw_measurements_(0);
  double py_ = meas_package.raw_measurements_(1);
  z_ << px_, py_;

  MatrixXd Zsig_ = Xsig_pred_.block(0, 0, n_z_, n_aug_sig_);
  VectorXd z_pred_ = Zsig_ * weights_;

  MatrixXd S_ = MatrixXd::Zero(n_z_, n_z_);
  S_ << std_laspx_ * std_laspx_, 0,
       0, std_laspy_ * std_laspy_;

  MatrixXd T_ = MatrixXd::Zero(n_x_, n_z_);

  for (int i = 0; i< n_aug_sig_; i++)
  {
      VectorXd zcol = Zsig_.col(i);
      zcol -= z_pred_;
      S_ += weights_(i) * (zcol * zcol.transpose());

      VectorXd xcol = Xsig_pred_.col(i) - x_;
      T_ += weights_(i) * xcol * zcol.transpose();
  }
  MatrixXd S_inv_ = S_.inverse();
  MatrixXd K_ = T_ * S_inv_;
  x_ += K_ * (z_ - z_pred_);
  P_ -= K_ * S_ * K_.transpose();

  VectorXd z_diff_ = z_ - z_pred_;
  double nis = z_diff_.transpose() * S_inv_ * z_diff_;
  std::cout << "Lidar NIS = " << nis << '\n';

}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  int n_z_ = 3;

  VectorXd z_ = VectorXd(n_z_);
  double rho_ = meas_package.raw_measurements_(0);
  double theta_ = meas_package.raw_measurements_(1);
  double rhodot_ = meas_package.raw_measurements_(2);
  z_ << rho_, theta_, rhodot_;

  // std::cout << "Received Measurement: " << z_.transpose() << '\n';

  MatrixXd Zsig_ = MatrixXd::Zero(n_z_, n_aug_sig_);

  for (int i = 0; i< n_aug_sig_; i++)
  {
    double rho = std::sqrt(Xsig_pred_(0, i) * Xsig_pred_(0, i) + Xsig_pred_(1, i) * Xsig_pred_(1, i));
    Zsig_(0, i) = rho;
    if (fabs(rho) > 0.001)
    {
      Zsig_(1, i) = std::atan2(Xsig_pred_(1, i), Xsig_pred_(0, i));
      Zsig_(2, i) = (Xsig_pred_(0, i) * Xsig_pred_(2, i) * std::cos(Xsig_pred_(3, i)) + Xsig_pred_(1, i) * Xsig_pred_(2, i) * std::sin(Xsig_pred_(3, i))) / rho;
    }
    else
    {
      Zsig_(1, i) = 0;
      Zsig_(2, i) = (i > 0)? Zsig_(2, i - 1): 0;
    }

  }

  // std::cout << "Zsig = " << "\n";
  // std::cout << Zsig_ << '\n';

  VectorXd z_pred_ = Zsig_ * weights_;

  MatrixXd S_ = MatrixXd::Zero(n_z_, n_z_);
  S_ << std_radr_ * std_radr_, 0, 0,
        0, std_radphi_ * std_radphi_, 0,
        0, 0, std_radrd_ * std_radrd_;

  MatrixXd T_ = MatrixXd::Zero(n_x_, n_z_);

  for (int i = 0; i< n_aug_sig_; i++)
  {
      VectorXd zcol = Zsig_.col(i);
      zcol -= z_pred_;
      S_ += weights_(i) * (zcol * zcol.transpose());

      VectorXd xcol = Xsig_pred_.col(i) - x_;
      T_ += weights_(i) * xcol * zcol.transpose();
  }

  MatrixXd S_inv_ = S_.inverse();
  MatrixXd K_ = T_ * S_inv_;
  x_ += K_ * (z_ - z_pred_);
  P_ -= K_ * S_ * K_.transpose();

  VectorXd z_diff_ = z_ - z_pred_;
  double nis = z_diff_.transpose() * S_inv_ * z_diff_;
  std::cout << "Radar NIS = " << nis << '\n';
}
