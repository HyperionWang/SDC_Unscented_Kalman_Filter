#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

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

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1;

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

  /**
  TODO - Finished

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  NIS_laser_ = 0;
  NIS_radar_ = 0;

  n_aug_ = 7;
  n_x_ = 5;

  lambda_ = 3 - n_aug_;

  weights_ = VectorXd(2 * n_aug_ + 1);
  double weight_0 = lambda_ / (lambda_ + n_aug_);
  weights_(0) = weight_0;

  for (int i = 1; i < 2 * n_aug_ + 1; i++) {
    double weight = 0.5/(n_aug_+lambda_);
    weights_(i) = weight;
  }

  x_ << 0, 0, 2, 0.3, 0;

  P_ <<   1, 0, 0, 0, 0,
          0, 1, 0, 0, 0,
          0, 0, 130, 0, 0,
          0, 0, 0, 100, 0,
          0, 0, 0, 0, 1;

  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO - Finished

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */


  if (!is_initialized_) {

    cout << "Start to initialize" << endl;

    time_us_ = meas_package.timestamp_;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {

      double rho = meas_package.raw_measurements_(0);
      double theta = meas_package.raw_measurements_(1);
      double ro_dot = meas_package.raw_measurements_(2);
      x_(0) = rho * cos(theta);
      x_(1) = rho * sin(theta);

    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      double px = meas_package.raw_measurements_(0);
      double py = meas_package.raw_measurements_(1);
      x_(0) = px;
      x_(1) = py;
    }

    is_initialized_ = true;
    cout << "End to initialize" << endl;
    cout << "x_ = " << x_ << endl;
    cout << "P_ = " << P_ << endl;
    return;
  }

  double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;

  time_us_ = meas_package.timestamp_;
  cout << "Start to Predict" << endl;
  Prediction(delta_t);
  cout << "End to Predict" << endl;
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR and use_radar_) {
    // Radar updates

    cout<< "Start to Update Radar..." <<endl;
    UpdateRadar(meas_package);
    cout<< "Finish Update Radar..." <<endl;
  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER and use_laser_) {
    // Laser updates
    cout<< "Start to Update Laser..." <<endl;
    UpdateLidar(meas_package);
    cout<< "End to Update Laser..." <<endl;
  }

  // print the output
  cout << "x_ = " << x_ << endl;
  cout << "P_ = " << P_ << endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO - Finished

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */



  VectorXd x_aug = VectorXd(n_aug_);

  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5, 5) = std_a_ * std_a_;
  P_aug(6, 6) = std_yawdd_ * std_yawdd_;

  //Create the sqrt of Augumented Matrix P_aug
  MatrixXd L = P_aug.llt().matrixL();

  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  Xsig_aug.col(0) = x_aug;

  for (int i = 0; i < n_aug_; i++) {
    Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }


  for (int i = 0; i < 2 * n_aug_ + 1; i++) {

    double px = Xsig_aug(0, i);
    double py = Xsig_aug(1, i);
    double v = Xsig_aug(2, i);
    double yaw = Xsig_aug(3, i);
    double yawd = Xsig_aug(4, i);
    double nu_a = Xsig_aug(5, i);
    double nu_yawdd = Xsig_aug(6, i);

    if (fabs(yawd) > 0.001) {
      Xsig_pred_(0, i) =
              px + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw)) + 0.5 * delta_t * delta_t * cos(yaw) * nu_a;
      Xsig_pred_(1, i) =
              py + v / yawd * (-cos(yaw + yawd * delta_t) + cos(yaw)) + 0.5 * delta_t * delta_t * sin(yaw) * nu_a;
      Xsig_pred_(2, i) = v + 0 + delta_t * nu_a;
      Xsig_pred_(3, i) = yaw + yawd * delta_t + 0.5 * delta_t * delta_t * nu_yawdd;
      Xsig_pred_(4, i) = yawd + 0 + delta_t * nu_yawdd;
    }
    else {
      Xsig_pred_(0, i) =
              px + v * delta_t*cos(yaw) + 0.5 * delta_t * delta_t * cos(yaw) * nu_a;
      Xsig_pred_(1, i) =
              py + v * delta_t*sin(yaw) + 0.5 * delta_t * delta_t * sin(yaw) * nu_a;
      Xsig_pred_(2, i) = v + 0 + delta_t * nu_a;
      Xsig_pred_(3, i) = yaw + yawd * delta_t + 0.5 * delta_t * delta_t * nu_yawdd;
      Xsig_pred_(4, i) = yawd + 0 + delta_t * nu_yawdd;
    }
  }

//  std::cout << "Xsig_pred = " << std::endl << Xsig_pred_ << std::endl;

  x_.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);

  }

  //angle normalization
  while (x_(3) > PI) x_(3) -= 2. * PI;
  while (x_(3) < -PI) x_(3) += 2. * PI;

  std::cout << "Finish x_ predict " << std::endl;
  P_.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {

    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    // To wrapp the angle for the difference

    while (x_diff(3) > PI) x_diff(3) -= 2. * PI;
    while (x_diff(3) < -PI) x_diff(3) += 2. * PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();

  }
  std::cout << "Finish P_ predict " << std::endl;
  //print result
  std::cout << "Predicted state" << std::endl;
  std::cout << x_ << std::endl;
  std::cout << "Predicted covariance matrix" << std::endl;
  std::cout << P_ << std::endl;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO - Finished

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

  int n_z = 2;
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);

    double v1 = cos(yaw) * v;
    double v2 = sin(yaw) * v;

    // measurement model
    Zsig(0, i) = p_x;
    Zsig(1, i) = p_y;
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }


  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;


    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_laspx_ * std_laspx_, 0,
          0, std_laspy_ * std_laspy_;
  S = S + R;
  //print result
//  std::cout << "z_pred: " << std::endl << z_pred << std::endl;
//  std::cout << "S: " << std::endl << S << std::endl;

  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3) > PI) x_diff(3) -= 2. * PI;
    while (x_diff(3) < -PI) x_diff(3) += 2. * PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }


  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  //angle normalization
  while (x_(3) > PI) x_(3) -= 2. * PI;
  while (x_(3) < -PI) x_(3) += 2. * PI;

  P_ = P_ - K * S * K.transpose();

  NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff; // x.050 is 5.991
  std::cout << "NIS_laser_ (5.991): " << NIS_laser_ << std::endl;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO - Finished

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  int n_z = 3;
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);

    double v1 = cos(yaw) * v;
    double v2 = sin(yaw) * v;

    // measurement model
    Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y);                        //r
    Zsig(1, i) = atan2(p_y, p_x);                                 //phi

    if (fabs(Zsig(0,i)) > 0.001) {
      Zsig(2, i) = (p_x * v1 + p_y * v2) / sqrt(p_x * p_x + p_y * p_y);
    }   //r_dot}
    else {

      Zsig(2, i) = (p_x * v1 + p_y * v2) / 0.001;
    }

  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }


  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1) > PI) z_diff(1) -= 2. * PI;
    while (z_diff(1) < -PI) z_diff(1) += 2. * PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_radr_ * std_radr_, 0, 0,
          0, std_radphi_ * std_radphi_, 0,
          0, 0, std_radrd_ * std_radrd_;
  S = S + R;
  //print result
//  std::cout << "z_pred: " << std::endl << z_pred << std::endl;
//  std::cout << "S: " << std::endl << S << std::endl;

  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1) > PI) z_diff(1) -= 2. * PI;
    while (z_diff(1) < -PI) z_diff(1) += 2. * PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3) > PI) x_diff(3) -= 2. * PI;
    while (x_diff(3) < -PI) x_diff(3) += 2. * PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }


  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred;

  //angle normalization
  while (z_diff(1) > PI) z_diff(1) -= 2. * PI;
  while (z_diff(1) < -PI) z_diff(1) += 2. * PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;

  //angle normalization
  while (x_(3) > PI) x_(3) -= 2. * PI;
  while (x_(3) < -PI) x_(3) += 2. * PI;

  P_ = P_ - K * S * K.transpose();
  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;  // x.050 is 7.815

  std::cout << "NIS_radar_ (7.815): " << NIS_radar_ << std::endl;
}
