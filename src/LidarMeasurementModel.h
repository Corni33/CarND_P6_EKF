#pragma once

#include "Eigen/Dense"
#include "MeasurementModel.h"

class LidarMeasurementModel : public MeasurementModel {
public:
  Eigen::MatrixXd H_, R_;

  LidarMeasurementModel(); 
  Eigen::VectorXd predictMeasurement(Eigen::VectorXd &x);

};