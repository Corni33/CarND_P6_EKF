#pragma once

#include "Eigen/Dense"

class LidarMeasurementModel {
public:
  Eigen::MatrixXd H_, R_;

  LidarMeasurementModel(); 
  Eigen::VectorXd predictMeasurement(Eigen::VectorXd &x);

};