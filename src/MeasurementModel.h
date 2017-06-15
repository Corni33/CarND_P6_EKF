#pragma once

#include "Eigen/Dense"

class MeasurementModel {
public:
  virtual void predictMeasurement(Eigen::VectorXd &x, Eigen::VectorXd &z_pred);
};