#include "ConstVelocityMotionModel.h"

ConstVelocityMotionModel::ConstVelocityMotionModel() {

  // state transition matrix
  F_ = Eigen::MatrixXd(4, 4); 

  // state transition matrix (for dt = 0)
  F_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

  // state covariance matrix
  Q_ = Eigen::MatrixXd(4, 4); 

  // process noise
  noise_ax_ = 9;
  noise_ay_ = 9;
}


void ConstVelocityMotionModel::predict(Eigen::VectorXd &x, Eigen::MatrixXd &P, double dt) {

  // adapt state covariance matrix
  calculateCovarianceMatrix(dt);

  // adapt state transition matrix 
  F_(0, 2) = dt;
  F_(1, 3) = dt;

  x = F_*x;
  P = F_*P*F_.transpose() + Q_;
}

void ConstVelocityMotionModel::calculateCovarianceMatrix(double dt) {
  double dt_2 = dt * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;

  Q_ << dt_4 / 4 * noise_ax_, 0, dt_3 / 2 * noise_ax_, 0,
        0, dt_4 / 4 * noise_ay_, 0, dt_3 / 2 * noise_ay_,
        dt_3 / 2 * noise_ax_, 0, dt_2*noise_ax_, 0,
        0, dt_3 / 2 * noise_ay_, 0, dt_2*noise_ay_;
}