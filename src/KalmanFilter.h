#pragma once

#include "Eigen/Dense"
#include "ConstVelocityMotionModel.h"
#include "RadarMeasurementModel.h"
#include "LidarMeasurementModel.h"
#include "measurement_package.h"
#include <iostream>

class KalmanFilter {
public:
  Eigen::VectorXd x_;
  Eigen::MatrixXd P_;

  ConstVelocityMotionModel motionModel_;
  RadarMeasurementModel radarMeasModel_;
  LidarMeasurementModel lidarMeasModel_;

  KalmanFilter();
  void initialize(const MeasurementPackage &measurement_pack);
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);
  void predict(double dt);
  void update(const MeasurementPackage measurement_pack);

private:
  bool is_initialized_;
  long long previous_timestamp_;

};