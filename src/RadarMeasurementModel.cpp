#include "RadarMeasurementModel.h"

RadarMeasurementModel::RadarMeasurementModel() {
  R_ = Eigen::MatrixXd(3, 3);

  //measurement covariance matrix
  R_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;
}

Eigen::VectorXd RadarMeasurementModel::predictMeasurement(Eigen::VectorXd &x) {

  Eigen::VectorXd z_pred(3);
  
  double px = x(0);
  double py = x(1);
  double vx = x(2);
  double vy = x(3);

  // map state variables to measurement space 
  z_pred(0) = sqrt(px*px + py*py);
  z_pred(1) = atan2(py, px); //atan(py/px)  
  z_pred(2) = (px*vx + py*vy) / z_pred(0);

  return z_pred;

}

Eigen::MatrixXd RadarMeasurementModel::getJacobian(Eigen::VectorXd &x) {

  Eigen::MatrixXd Hj(3, 4);

  //recover state parameters
  float px = x(0);
  float py = x(1);
  float vx = x(2);
  float vy = x(3);

  //pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px + py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

  //check division by zero
  if (fabs(c1) < 0.0001) {
    return Hj;
  }

  //compute the Jacobian matrix
  Hj << (px / c2), (py / c2), 0, 0,
    -(py / c1), (px / c1), 0, 0,
    py*(vx*py - vy*px) / c3, px*(px*vy - py*vx) / c3, px / c2, py / c2;

  return Hj;

}