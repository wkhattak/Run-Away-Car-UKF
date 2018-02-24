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
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector: [pos_x pos_y velocity_abs yaw_angle yaw_rate] in SI units and rad
  x_ = VectorXd(5);

  // Process noise standard deviation longitudinal acceleration in m/s^2 (aka linear acceleration)
  std_a_ = 0.75; 

  // Process noise standard deviation yaw acceleration in rad/s^2 (aka angular acceleration)
  std_yawdd_ = 0.5; 
  
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
   
  // Set to true in first call of ProcessMeasurement
  is_initialized_ = false;
  
  // Initial time
  time_us_ = 0;
  
  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // Sigma point spreading parameter
  lambda_ =  3 - n_x_;
  
  // Augmented sigma point spreading parameter
  lambda_aug_ =  3 - n_aug_;
  
  // Initial state vector 
  // (px & py can only be initialized once the first sensor measurement arrives,
  // for the other 3 variables, try different initialization values to see what works best.
  x_ << 0, 0, 0, 0, 0;

  // Initial state covariance matrix
  P_ = MatrixXd::Identity(n_x_, n_x_);

  // Predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_ + 1);
   
  // Weights of sigma points
  weights_ = VectorXd(2*n_aug_ + 1);
  
  // Noise matrices for each sensor
  R_radar_ = MatrixXd(3,3);
  R_lidar_ = MatrixXd(2,2);
  
  // Initial NIS
  nis_radar_ = 0;
  nis_lidar_ = 0;
  
  // Set zero threshold value
  zero_threshold_ = 0.001;  
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  if (!is_initialized_) {   
    // set noise matrices
    R_radar_ << std_radr_*std_radr_, 0, 0,
              0, std_radphi_*std_radphi_, 0,
              0, 0,std_radrd_*std_radrd_;

    R_lidar_ << std_laspx_*std_laspx_,0,
              0,std_laspy_*std_laspy_;
    
    // set weights
    double weight_0 = lambda_aug_/(lambda_aug_ + n_aug_);
    weights_(0) = weight_0;
    for (int i = 1; i < 2*n_aug_+1; i++) {
      double weight = 0.5/(n_aug_ + lambda_aug_);
      weights_(i) = weight;
    }
    
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert radar from polar to cartesian coordinates and initialize state.
      float rho = meas_package.raw_measurements_(0);
      float phi = meas_package.raw_measurements_(1);
      float rho_dot = meas_package.raw_measurements_(2);
      x_(0) = rho * cos(phi);     
      x_(1) = rho * sin(phi);      
      x_(3) = rho_dot * cos(phi);     
      x_(4) = rho_dot * sin(phi);      
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      x_(0) = meas_package.raw_measurements_(0);
      x_(1) = meas_package.raw_measurements_(1);
    }
    
    if (fabs(x_(0)) < zero_threshold_) x_(0) = 0;
    if (fabs(x_(1)) < zero_threshold_) x_(1) = 0;
    
    time_us_ = meas_package.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }
  
  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  float d_t = (meas_package.timestamp_ - time_us_) / 1000000.0;	//delta (seconds)
  time_us_ = meas_package.timestamp_;
   
  Prediction(d_t);
  
  /*****************************************************************************
  *  Update
  ****************************************************************************/
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
    UpdateRadar(meas_package);
  }
  
  if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
    UpdateLidar(meas_package);
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /*****************************************************************************
  *  Sigma Point Creation
  ****************************************************************************/
  // create sigma point matrix
  MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);

  // calculate square root of P
  MatrixXd A = P_.llt().matrixL();
  
  // set first column of sigma point matrix
  Xsig.col(0)  = x_;

  // set remaining sigma points
  for (int i = 0; i < n_x_; i++) {
    Xsig.col(i + 1) = x_ + sqrt(lambda_ + n_x_) * A.col(i);
    Xsig.col(i + 1 + n_x_) = x_ - sqrt(lambda_ + n_x_) * A.col(i);
  }
  
  /*****************************************************************************
  *  Sigma Point Augmentation
  ****************************************************************************/
  // create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  // create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);

  // create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  
  // create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_ * std_a_;
  P_aug(6,6) = std_yawdd_ * std_yawdd_;

  // create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  // create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug_; i++) {
    Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_aug_ + n_aug_) * L.col(i);
    Xsig_aug.col(i+ 1 + n_aug_) = x_aug - sqrt(lambda_aug_ + n_aug_) * L.col(i);
  }
  
  /*****************************************************************************
  *  Sigma Point Prediction
  ****************************************************************************/
  // predict sigma points
  for (int i = 0; i < 2*n_aug_+1; i++) { //2n+1 simga points
    // extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    // predicted state values
    double px_p, py_p;

    // avoid division by zero
    if (fabs(yawd) > zero_threshold_) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    // add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    // write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }
  
  /*****************************************************************************
  *  State Mean & Covariance Prediction
  ****************************************************************************/
  // predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++) {  //iterate over sigma points
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  // predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++) {  //iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3) -= 2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3) += 2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {  
  /*****************************************************************************
  * LiDAR Measurement Prediction
  ****************************************************************************/
  // set measurement dimension, radar can measure position x & y
  int n_z = 2;
  
  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_ + 1);
  
  // transform sigma points into measurement space
  for (int i = 0; i < 2*n_aug_+1; i++) {  //2n+1 simga points
    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);

    // measurement model
    Zsig(0,i) = p_x;                        //p_x
    Zsig(1,i) = p_y;                        //p_y
  }

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<    std_laspx_*std_laspx_, 0, 
          0, std_laspy_*std_laspy_;
  S = S + R;
  
  /*****************************************************************************
  *  LiDAR UKF Update
  ****************************************************************************/
  // get incoming LiDAR measurements
  VectorXd z = meas_package.raw_measurements_;
   
  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  
  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++) {  //2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // residual
  VectorXd z_diff = z - z_pred;

  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();
  
  /*****************************************************************************
  *  LiDAR NIS (check at 2 degrees of freedom)
  ****************************************************************************/
  nis_lidar_ = z_diff.transpose() * S.inverse() * z_diff;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {  
  /*****************************************************************************
  * Radar Measurement Prediction
  ****************************************************************************/
  // set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;
  
  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_ + 1);
  
  // transform sigma points into measurement space
  for (int i = 0; i < 2*n_aug_+1; i++) {  //2n+1 simga points
    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<    std_radr_*std_radr_, 0, 0,
          0, std_radphi_*std_radphi_, 0,
          0, 0,std_radrd_*std_radrd_;
  S = S + R;
  
  /*****************************************************************************
  *  Radar UKF Update
  ****************************************************************************/
  // get incoming radar measurements
  VectorXd z = meas_package.raw_measurements_;
   
  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  
  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++) {  //2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // residual
  VectorXd z_diff = z - z_pred;

  // angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();
  
  /*****************************************************************************
  *  Radar NIS (check at 3 degrees of freedom)
  ****************************************************************************/
  nis_radar_ = z_diff.transpose() * S.inverse() * z_diff;
}