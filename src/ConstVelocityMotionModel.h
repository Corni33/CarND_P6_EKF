#pragma once

#include "Eigen/Dense"
#include "MotionModel.h"

class ConstVelocityMotionModel : public MotionModel {
public:
  Eigen::MatrixXd F_, Q_;
  double noise_ax_, noise_ay_;

  ConstVelocityMotionModel(); // Constructor
  void predict(Eigen::VectorXd &x, Eigen::MatrixXd &P, double dt);

private:
  void calculateCovarianceMatrix(double dt);
};