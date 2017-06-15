#pragma once

#include "Eigen/Dense"
#include "MeasurementModel.h"

class RadarMeasurementModel : public MeasurementModel {
public:
  Eigen::MatrixXd R_;

  RadarMeasurementModel(); // Constructor
  Eigen::VectorXd predictMeasurement(Eigen::VectorXd &x);
  Eigen::MatrixXd getJacobian(Eigen::VectorXd &x);

};