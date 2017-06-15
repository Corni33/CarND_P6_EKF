#pragma once

#include "Eigen/Dense"

class MotionModel {
public:
  virtual void predict(Eigen::VectorXd &x, Eigen::MatrixXd &P, double dt) = 0;
};