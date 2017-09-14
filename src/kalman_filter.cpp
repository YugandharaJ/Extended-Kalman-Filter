#include "kalman_filter.h"
#include "FusionEKF.h"
#include<vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
    x_ = F_ * x_ ;
    P_ = F_ * P_ * F_.transpose() + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
    /*I_L = MatrixXd(2, 3);
    I_L << 1, 0, 0, 0,
            0, 1, 0, 0;*/
    VectorXd z_pred = H_ * x_ ;
    VectorXd y = z - z_pred;
    MatrixXd S = H_ * P_ * H_.transpose() + R_;
    MatrixXd K = P_*H_.transpose()*S.inverse();
    x_ = x_ + (K*y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
    FusionEKF fusionEKF;
    float x = fusionEKF.ekf_.x_(0);
    float y = fusionEKF.ekf_.x_(1);
    float vx = fusionEKF.ekf_.x_(2);
    float vy = fusionEKF.ekf_.x_(3);

    float ro = sqrt(x*x+y*y);
    float theta = atan2(y,x);
    float ro_dot = (x*vx+y*vy)/ro;
    VectorXd z_pred = VectorXd(3);
    z_pred << ro, theta, ro_dot;

    VectorXd y1 = z - z_pred;

    MatrixXd S = H_ * P_ * H_.transpose() + R_;
    MatrixXd K = P_*H_.transpose()*S.inverse();
    x_ = x_ + (K*y1);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K*H_)*P_;
}
