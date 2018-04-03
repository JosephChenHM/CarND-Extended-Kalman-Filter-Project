#include "kalman_filter.h"
// #include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &Hj_in, MatrixXd &R_in, MatrixXd &R_ekf_in,
                        MatrixXd &Q_in)
{
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  Hj_ = Hj_in;
  R_ = R_in;
  R_ekf_ = R_ekf_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  MatrixXd Ft_ = F_.transpose();

  x_ = F_ * x_;
  P_ = F_ * P_ * Ft_ + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  MatrixXd Ht_ = H_.transpose();

  VectorXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * Ht_ + R_;
  MatrixXd K = P_ * Ht_ * S.inverse();
  MatrixXd I = Eigen::MatrixXd::Identity(4,4);

  x_ = x_ + K * y;
  P_ = (I - K * H_) * P_; 
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  MatrixXd Hj(3,4);
  MatrixXd Hjt(3,4);

  Hj = tools.CalculateJacobian(x_);

  Hjt = Hj.transpose();
  //Update y with function H (nonlinear function)
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);
  if(px == 0)
  {
    cout << "px = 0, divide by zero" << endl;
    return;
  }
  double sqrtPxy_2 = sqrt(px*px+py*py);
  double theta = atan2(py, px);
  double sqrtPxy_2_dot = (px*vx+py*vy)/sqrtPxy_2;
  VectorXd foH(3);
  foH << sqrtPxy_2, theta, sqrtPxy_2_dot;

  VectorXd y = z - foH;
  if( y[1] > M_PI )
    y[1] -= 2.d*M_PI;
  if( y[1] < -M_PI )
    y[1] += 2.d*M_PI;
  MatrixXd S = Hj * P_ * Hjt + R_ekf_;
  MatrixXd K = P_ * Hjt * S.inverse();
  MatrixXd I = Eigen::MatrixXd::Identity(4,4);

  x_ = x_ + K * y;
  P_ = (I - K * Hj_) * P_; 
}
