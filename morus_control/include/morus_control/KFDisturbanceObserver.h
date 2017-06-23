//
// Created by luka on 22.06.17..
// Kalman filter to estimate disturbances and achieve reference tracking

#ifndef PROJECT_KFDISTURBANCEOBSERVER_H
#define PROJECT_KFDISTURBANCEOBSERVER_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

namespace mav_control_attitude {



class KFDisturbanceObserver
{
 private:
  static constexpr int kStateSizeKalman = 7;
  static constexpr int kMeasurementSizeKalman = 6;

  static constexpr int kStateSize = 6;
  static constexpr int kMeasurementSize = 1;
  static constexpr int kDisturbanceSize = 1;
  static constexpr int kInputSize = 2;
  static constexpr double kGravity = 9.8066;
  
 public:
  KFDisturbanceObserver(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
  virtual ~KFDisturbanceObserver();

  //Feeding
  void reset();

  // Setting
  void setMeasuredStates(double movable_mass_0_position, double movable_mass_0_speed,
                         double movable_mass_1_position, double movable_mass_1_speed,
                         double angle, double angular_velocity)
  {
    measurements_(0) = movable_mass_0_position;
    measurements_(1) = movable_mass_0_speed;
    measurements_(2) = movable_mass_1_position;
    measurements_(3) = movable_mass_1_speed;
    measurements_(4) = angle;
    measurements_(5) = angular_velocity;
  }

  void setMovingMassCommand(Eigen::Vector2d command)
  {
    command_moving_masses_ = command;
  }

  void setInitialState(double movable_mass_0_position, double movable_mass_0_speed,
                       double movable_mass_1_position, double movable_mass_1_speed,
                       double angle, double angular_velocity)
  {
    state_(0) = movable_mass_0_position;
    state_(1) = movable_mass_0_speed;
    state_(2) = movable_mass_1_position;
    state_(3) = movable_mass_1_speed;
    state_(4) = angle;
    state_(5) = angular_velocity;
    state_(6) = 0.0; // disturbance
  }

  void setSystemMatrices(Eigen::Matrix<double, kStateSize, kStateSize> model_A,
                         Eigen::Matrix<double, kStateSize, kInputSize> model_B,
                         Eigen::Matrix<double, kStateSize, kDisturbanceSize> model_Bd);

  bool updateEstimator();

  void getEstimatedState(Eigen::VectorXd* estimated_state) const;


  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:

  typedef Eigen::Matrix<double, kStateSizeKalman, 1> StateVector;
  ros::NodeHandle nh_, private_nh_;

  StateVector state_; // [x1, dx1, x3, dx3, theta, dtheta, dist_theta] -> F is [7,7]
  Eigen::Matrix<double, kMeasurementSizeKalman, 1> measurements_;  // [x1, dx1, x3, dx3, theta, dtheta]
  Eigen::Matrix<double, kStateSizeKalman, kStateSizeKalman> state_covariance_; // P0 matrix
  Eigen::Matrix<double, kStateSizeKalman, 1> initial_state_covariance_; // P0 values
  Eigen::Matrix<double, kStateSizeKalman, kStateSizeKalman> process_noise_covariance_; // We express it as diag() later.
  Eigen::Matrix<double, kMeasurementSizeKalman, kMeasurementSizeKalman> measurement_covariance_; // We express it as diag() later.

  // Kalman filter matrices of states
  //Eigen::SparseMatrix<double> F_; // System dynamics matrix.
  Eigen::Matrix<double, kStateSizeKalman, kStateSizeKalman> F_; // System dynamics matrix.
  Eigen::Matrix<double, kStateSizeKalman, kMeasurementSizeKalman> K_; // Kalman gain matrix.
  Eigen::Matrix<double, kMeasurementSizeKalman, kStateSizeKalman> H_; // Measurement matrix.
  Eigen::Matrix<double, kStateSizeKalman, kInputSize> G_;
  //Eigen::SparseMatrix<double> H_; // Measurement matrix.

  // system model
  // Model: A, B, Bd
  // x(k+1) = A*x(k) + B*u(k) + Bd*d(k)
  Eigen::Matrix<double, kStateSize, kStateSize> model_A_;   //dynamics matrix
  Eigen::Matrix<double, kStateSize, kInputSize> model_B_;   //transfer matrix
  Eigen::Matrix<double, kStateSize, kDisturbanceSize> model_Bd_;  //Disturbance transfer  gas motor paramsmatrix

  void calculateKalmanMatrices(Eigen::Matrix<double, kStateSizeKalman, kStateSizeKalman>* F_,
                               Eigen::Matrix<double, kStateSizeKalman, kInputSize>* G_,
                               Eigen::Matrix<double, kMeasurementSizeKalman, kStateSizeKalman> *H_);

  void simulateSystem();
  Eigen::Vector2d command_moving_masses_;
  bool initialized_;

};
}


#endif //PROJECT_KFDISTURBANCEOBSERVER_H
