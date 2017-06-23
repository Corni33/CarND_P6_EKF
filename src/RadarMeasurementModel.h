#pragma once

#include "Eigen/Dense"

class RadarMeasurementModel {
public:
  Eigen::MatrixXd R_;

  RadarMeasurementModel(); 
  Eigen::VectorXd predictMeasurement(Eigen::VectorXd &x);
  Eigen::MatrixXd getJacobian(Eigen::VectorXd &x);

};