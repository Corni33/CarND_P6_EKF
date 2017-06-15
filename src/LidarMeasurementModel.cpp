#include "LidarMeasurementModel.h"

LidarMeasurementModel::LidarMeasurementModel() {
  H_ = Eigen::MatrixXd(2, 4);

  H_ << 1, 0, 0, 0,
        0, 1, 0, 0;

  R_ = Eigen::MatrixXd(2, 2);

  //measurement covariance matrix 
  R_ << 0.0225, 0,
        0, 0.0225;


}

Eigen::VectorXd LidarMeasurementModel::predictMeasurement(Eigen::VectorXd &x) {

  Eigen::VectorXd z_pred(2);

  z_pred = H_*x;

  return z_pred;
  
}