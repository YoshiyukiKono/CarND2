#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  // KalmanFilter
  x_ = F_*x_;
  // EKF
  // x_ = f(x_, 0) u=0
  P_ = F_*P_*F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  /*
   * KF Measurement update step
   */
  cout << "y = z - H_ * x_: "  << endl;
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  cout << "S = H_ * P_ * Ht + R_: "  << endl;
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  cout << "K =  P_ * Ht * Si: "  << endl;
  MatrixXd K =  P_ * Ht * Si;
  cout << "x_ = x_ + (K * y): "  << endl;
  //new state
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size); // Identity matrix
  cout << "P_ = (I - K * H_) * P_: "  << endl;
  P_ = (I - K * H_) * P_;
  cout << "KalmanFilter::Updated: "  << endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  
  VectorXd h = VectorXd(3);
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];
  float ro = sqrt(px*px + py*py);
  float theta = atan(py/px);
  float ro_dot = (px*vx+py*vy)/sqrt(px*px + py*py);

  h << ro, theta, ro_dot;
  cout << "y = z - h: "  << endl;
  VectorXd y = z - h;

  Tools tools;
  H_ = tools.CalculateJacobian(x_);//H_);
  MatrixXd Ht = H_.transpose();
  cout << "S = H_ * P_ * Ht + R_: "  << endl;
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  cout << "K =  P_ * Ht * Si: "  << endl;
  MatrixXd K =  P_ * Ht * Si;

  //new state
  cout << "x_ = x_ + (K * y): "  << endl;
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size); // Identity matrix
  //Tools tools;
  
  cout << "P_ = (I - K * Hj) * P_i: "  << endl;
  P_ = (I - K * H_) * P_;
}
