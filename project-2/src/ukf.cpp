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

  ///* State dimension
  n_x_ = 5;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_); //5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  
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
  ///* initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  ///* time when the state is true, in us
  time_us_ = 0;

  ///* Weights of sigma points
  //VectorXd weights_;

  ///* Augmented state dimension
  n_aug_ = 7;

  ///* Sigma point spreading parameter
  lambda_ = 3 - n_x_;

  ///* predicted sigma points matrix
  //Xsig_pred_ = MatrixXd(n_x_, 2 * n_x_ + 1);
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  
  //create augmented mean vector
  x_aug_ = VectorXd(n_aug_);

  //create augmented state covariance
  P_aug_ = MatrixXd(n_aug_, n_aug_);
  
  //create augmented sigma point matrix
  Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  
  //set measurement dimension, radar can measure r, phi, and r_dot
  n_z_radar_ = 3;
  
  //set measurement dimension, laser can measure px, and py
  n_z_laser_ = 2;

  //set measurement matrix for laser
  H_ = MatrixXd(2, n_x_);
  H_ << 1, 0, 0, 0, 0,
  		0, 1, 0, 0, 0;
  // set measurement covariance matrix for laser
  R_laser_ = MatrixXd(n_z_laser_, n_z_laser_);
  R_laser_ << std_laspx_ * std_laspx_, 0,
	          0,                       std_laspy_ * std_laspy_;

  //add measurement noise covariance matrix
  R_radar_ = MatrixXd(n_z_radar_,n_z_radar_);
  R_radar_ << std_radr_ * std_radr_, 0,                         0,
              0,                     std_radphi_ * std_radphi_, 0,
              0,                     0,                         std_radrd_ * std_radrd_;

  ///* Weights of sigma points
  SetWeights();
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
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ex_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "UKF: " << endl;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      //set the state with the initial location and zero velocity
	  float rho_measured = meas_package.raw_measurements_[0]; 
	  float phi_measured = meas_package.raw_measurements_[1]; 
	  float rhodot_measured = meas_package.raw_measurements_[2];
	  float x = rho_measured*cos(phi_measured); 
	  float y = rho_measured*sin(phi_measured); 
	  
	  //cout << y << endl;
	  x_ << x, y, 0, 0, 0;

      //cout << "Radar Initialized: " << endl;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      //set the state with the initial location and zero velocity
	  x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
      
      //cout << "Laser Initialized: " << endl;
    }
    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  // Before prediction we have to compute the elapsed time between the current and previous observation.
  //compute the time elapsed between the current and previous measurements



  float delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;
  
//cout << "delta_t: " << delta_t << endl;
  
  Prediction(delta_t);

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */
  if (use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    UpdateRadar(meas_package);
  } 
  if (use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER) {
    // Laser updates
    UpdateLidar(meas_package);
  }

  // print the output
//cout << "x_ = " << x_ << endl;
//cout << "P_ = " << P_ << endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  //GenerateSigmaPoints();
  //AugmentedSigmaPoints()
  PredictSigmaPoints(delta_t);
//cout << "Predicted Sigma Points: " << endl;
  PredictMeanAndCovariance();
//cout << "Predicted Mean and Covariance: " << endl;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
//cout << "UpdateLidar: " << endl;

  // 1. predict measurement
  VectorXd z_pred = H_ * x_;

//cout << "DONE z_pred = H_ * x_: " << endl;
  
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_laser_;

//cout << "DONE MatrixXd S = H_ * P_ * Ht + R_laser_: " << endl;
  
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;

  // 2. update state
//cout << "UpdateLidar State: " << endl;
  
  VectorXd z = meas_package.raw_measurements_;//VectorXd();
  //z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1];
  
  VectorXd z_diff = z - z_pred;
  x_ = x_ + (K * z_diff);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  
  double NIS = z_diff.transpose() * Si * z_diff;
  cout << "Laser: NIS = " << NIS << endl;
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
//cout << "UpdateRadar: " << endl;
  
  //VectorXd z = meas_package.raw_measurements_;//CreateRadarMeasurement(meas_package);
  
  // 1. Predict Measurement

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_radar_, 2 * n_aug_ + 1);
  
  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

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

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_radar_);
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z_radar_,n_z_radar_);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    
//cout << "Zsig.col(i): " << Zsig.col(i) << endl;
//cout << "z_pred: " << z_pred << endl;
      
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  S = S + R_radar_;

  // 2. Update State
//cout << "UpdateRadar State: " << endl;
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_radar_);
  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

//cout << "calculating Tc: " << i << endl;

//cout << "Zsig.col(i): " << Zsig.col(i) << endl;
//cout << "z_pred: " << z_pred << endl;
    //residual
    //VectorXd z_diff = Zsig.col(i) - Xsig_pred_;
    VectorXd z_diff = Zsig.col(i) - z_pred;
    
    //angle normalization
    while (z_diff(1) >  M_PI) z_diff(1) -= 2.*M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    
    //angle normalization
    while (x_diff(3) >  M_PI) x_diff(3) -= 2.*M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  MatrixXd Si = S.inverse();
  //Kalman gain K;
  MatrixXd K = Tc * Si;

  //residual
  VectorXd z = meas_package.raw_measurements_;//CreateRadarMeasurement(meas_package);

  //VectorXd z_diff = z - Xsig_pred_;
  VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1) -= 2. * M_PI;
  while (z_diff(1)<-M_PI) z_diff(1) += 2. * M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S* K.transpose();
  
  double NIS = z_diff.transpose() * Si * z_diff;
  cout << "Radar: NIS = " << NIS << endl;
}

/*******************************************************************************
* PREDICTION functions: 
*******************************************************************************/
void UKF::PredictSigmaPoints(double delta_t){
	AugmentedSigmaPoints();
//cout << "AugmentedSigmaPoints" << endl;
	SigmaPointPrediction(delta_t);
}
void UKF::PredictMeanAndCovariance(){
  //predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  //predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.* M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.* M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }
}
/*******************************************************************************
* UPDATE functions: 
*******************************************************************************/
/*
MatrixXd UKF::PredictRadarMeasurement(){

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_radar_, 2 * n_aug_ + 1);
  
  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

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

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_radar_);
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z_radar_,n_z_radar_);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<    std_radr_ * std_radr_, 0,                         0,
          0,                     std_radphi_ * std_radphi_, 0,
          0,                     0,                         std_radrd_ * std_radrd_;
  S = S + R;
  return S;
}
// for Radar
void UKF::UpdateState(VectorXd z, MatrixXd S){
  
  //create matrix with sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_radar_, 2 * n_aug_ + 1);
  
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_radar_);
  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - Xsig_pred_;//???????????????????????????????????????
 
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - Xsig_pred_;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();
}
*/
VectorXd UKF::CreateRadarMeasurement(MeasurementPackage meas_package) {
  float rho_measured = meas_package.raw_measurements_[0]; 
  float phi_measured = meas_package.raw_measurements_[1]; 
  float rhodot_measured = meas_package.raw_measurements_[2];
  float px = rho_measured*cos(phi_measured); 
  float py = rho_measured*sin(phi_measured); 
  VectorXd x = VectorXd(n_z_radar_);
  x << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], meas_package.raw_measurements_[2]; //???????????????????????????????????????????????????
  return x;
}
VectorXd UKF::CreateLaserMeasurement(MeasurementPackage meas_package) {
  float px = meas_package.raw_measurements_[0]; 
  float py = meas_package.raw_measurements_[1];
  VectorXd x = VectorXd(n_z_laser_);
  x << px, py;
  return x;
}
/*******************************************************************************
* Initialization functions: 
*******************************************************************************/
void UKF::SetWeights() {
  //create vector for weights
  weights_ = VectorXd(2 * n_aug_ + 1);
  // set weights
  weights_(0) = lambda_/(lambda_ + n_aug_);
  for (int i = 1; i < 2 * n_aug_ + 1; i++) {  //2n+1 weights
    double weight = 0.5 / (n_aug_ + lambda_);
    weights_(i) = weight;
  }
  return;
}

/*******************************************************************************
* Sigma Points functions: 
*******************************************************************************/
/*
void UKF::GenerateSigmaPoints() {

  //create sigma point matrix
  MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);

  //calculate square root of P
  MatrixXd A = P_.llt().matrixL();

  //set first column of sigma point matrix
  Xsig.col(0)  = x_;

  //set remaining sigma points
  for (int i = 0; i < n_x_; i++)
  {
    Xsig.col(i+1)     = x_ + sqrt(lambda_ + n_x_) * A.col(i);
    Xsig.col(i+1 + n_x_) = x_ - sqrt(lambda_ + n_x_) * A.col(i);
  }

  //print result
  std::cout << "Xsig = " << std::endl << Xsig << std::endl;

}
**/
void UKF::AugmentedSigmaPoints() {

//cout << "START AugmentedSigmaPoints " << endl;

  //create augmented mean state
  x_aug_.head(5) = x_;
  x_aug_(5) = 0;
  x_aug_(6) = 0;

//cout << "created augmented covariance matrix " << endl;

  //create augmented covariance matrix
  P_aug_.fill(0.0);
  P_aug_.topLeftCorner(5,5) = P_;
  P_aug_(5,5) = std_a_  *std_a_;
  P_aug_(6,6) = std_yawdd_ * std_yawdd_;

//cout << "created augmented mean state " << endl;

  //create square root matrix
  MatrixXd L = P_aug_.llt().matrixL();

//cout << "created square root matrix " << endl;

  //create augmented sigma points
  Xsig_aug_.col(0)  = x_aug_;
  for (int i = 0; i< n_aug_; i++)
  {
//cout << "creating create augmented sigma points: " << i << endl;
    Xsig_aug_.col(i+1)       = x_aug_ + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug_.col(i+1 + n_aug_) = x_aug_ - sqrt(lambda_ + n_aug_) * L.col(i);
  }
//cout << "created create augmented sigma points " << endl;
}

void UKF::SigmaPointPrediction(double delta_t) {
//cout << "STARTBpredicting sigma points: " << 2 * n_aug_ + 1 << endl;
  //predict sigma points
  for (int i = 0; i< 2 * n_aug_ + 1; i++)
  {
//cout << "predicting sigma points: " << i << endl;
    //extract values for better readability
    double p_x = Xsig_aug_(0,i);
    double p_y = Xsig_aug_(1,i);
    double v = Xsig_aug_(2,i);
    double yaw = Xsig_aug_(3,i);
    double yawd = Xsig_aug_(4,i);
    double nu_a = Xsig_aug_(5,i);
    double nu_yawdd = Xsig_aug_(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v / yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v / yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v * delta_t * cos(yaw);
        py_p = p_y + v * delta_t * sin(yaw);
    }
//cout << "px_p: " << px_p << endl;
//cout << "py_p: " << py_p << endl;
    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p = v_p + nu_a * delta_t;

    yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p = yawd_p + nu_yawdd * delta_t;

//cout << "yaw_p: " << yaw_p << endl;
//cout << "yawd_p: " << yawd_p << endl;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
//cout << "predicted sigma points: " << i << endl;
  }
//cout << "predicted ALL the sigma points: "  << endl;
}