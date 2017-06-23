#include "KalmanFilter.h"


KalmanFilter::KalmanFilter() {

  x_ = Eigen::VectorXd::Zero(4);
  P_ = Eigen::MatrixXd::Zero(4, 4);

  is_initialized_ = false;
}

void KalmanFilter::initialize(const MeasurementPackage &measurement_pack) {

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

    std::cout << "init with RADAR" << std::endl;

    // convert radar data from polar to cartesian coordinates and initialize state
    double rho = measurement_pack.raw_measurements_(0);
    double phi = measurement_pack.raw_measurements_(1);
    double rho_dot = measurement_pack.raw_measurements_(2);

    x_(0) = rho*cos(phi);
    x_(1) = rho*sin(phi); 

    /* assumption for initializing the velocity: 
    * the orientation of the object's velocity vector is phi (i.e. the object is moving straight towards or away from us)
    * (an assumption is needed because there are 4 degrees of freedom (state variables) and only three constraints (radar measurement variables) )
    */
    x_(2) = rho*cos(rho_dot);
    x_(3) = rho*sin(rho_dot);

    // initialize covariance
    double var_rho = radarMeasModel_.R_(0, 0);

    // reasoning: 
    // rho is a distance measurement, so the variance of x- and y-position should be in the magnitude of var_rho
    // vx and vy are initialized with high uncertainty as we cannot determine vx and vy from rho_dot alone
    P_(0, 0) = var_rho;
    P_(1, 1) = var_rho;
    P_(2, 2) = 10; 
    P_(3, 3) = 10;

  }
  else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {

    std::cout << "init with LIDAR" << std::endl;

    // initialize state     
    x_(0) = measurement_pack.raw_measurements_(0);
    x_(1) = measurement_pack.raw_measurements_(1);
    x_(2) = 0;
    x_(3) = 0;

    // initialize covariance
    double sigma_x = 0.9; // TODO   R_laser_(0, 0);
    double sigma_y = 0.9; // TODO   R_laser_(1, 1);

    P_(0, 0) = sigma_x;
    P_(1, 1) = sigma_y;
    P_(2, 2) = 10; // high uncertainty, i.e. no idea about the initial velocity
    P_(3, 3) = 10;
  }

  previous_timestamp_ = measurement_pack.timestamp_;

  is_initialized_ = true;  

}

void KalmanFilter::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  if (!is_initialized_) {

    /***************************
    *  Initialize
    ****************************/
    initialize(measurement_pack);

    return; // no prediction/update necessary after initialization
  }

  // calculate time since last filter update 
  double dt = static_cast<double>(measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  /***************************
  *  Predict
  ****************************/
  predict(dt);

  /***************************
  *  Update
  ****************************/
  update(measurement_pack);


  // print the updated state and state covariance
  //std::cout << "x_ = " << x_ << std::endl;
  //std::cout << "P_ = " << P_ << std::endl;
  //std::cout << "--------------------------" << std::endl;

}

void KalmanFilter::predict(double dt) {

  motionModel_.predict(x_, P_, dt);

}

void KalmanFilter::update(const MeasurementPackage measurement_pack) {

  Eigen::MatrixXd H, S, K, I;
  Eigen::VectorXd z_pred, y;


  if (measurement_pack.sensor_type_ == MeasurementPackage::SensorType::LASER) {

    z_pred  = lidarMeasModel_.predictMeasurement(x_);
    H       = lidarMeasModel_.H_;    

    // calculate difference between sensor measurement and predicted measurement
    y = measurement_pack.raw_measurements_ - z_pred;

    // calculate Kalman gain matrix
    S = H * P_ * H.transpose() + lidarMeasModel_.R_;
    K = P_ * H.transpose() * S.inverse();

  } 
  else if (measurement_pack.sensor_type_ == MeasurementPackage::SensorType::RADAR) {

    z_pred  = radarMeasModel_.predictMeasurement(x_);
    H       = radarMeasModel_.getJacobian(x_);

    // calculate difference between sensor measurement and predicted measurement
    y = measurement_pack.raw_measurements_ - z_pred;

    // limit angle deviation to [-pi, pi]
    while (fabs(y(1)) > M_PI) {
      if (y(1) < -M_PI) {
        y(1) += 2 * M_PI;
      }
      else if (y(1) > M_PI) {
        y(1) -= 2 * M_PI;
      }
    }
   
    // calculate Kalman gain matrix
    S = H * P_ * H.transpose() + radarMeasModel_.R_;
    K = P_ * H.transpose() * S.inverse();

  }
  
  I = Eigen::MatrixXd::Identity(4, 4);

  // calculate new state estimate
  x_ = x_ + (K * y);
  P_ = (I - K * H) * P_;

}
