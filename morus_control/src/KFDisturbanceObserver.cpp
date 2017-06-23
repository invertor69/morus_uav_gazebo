//
// Created by luka on 22.06.17..
//

#include <morus_control/KFDisturbanceObserver.h>

namespace mav_control_attitude {

constexpr int KFDisturbanceObserver::kStateSizeKalman;
constexpr int KFDisturbanceObserver::kMeasurementSizeKalman;
constexpr int KFDisturbanceObserver::kStateSize;
constexpr int KFDisturbanceObserver::kMeasurementSize;
constexpr int KFDisturbanceObserver::kDisturbanceSize;
constexpr int KFDisturbanceObserver::kInputSize;
constexpr double KFDisturbanceObserver::kGravity;

KFDisturbanceObserver::KFDisturbanceObserver(const ros::NodeHandle& nh,
                                             const ros::NodeHandle& private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      initialized_(false)
{
    initial_state_covariance_(0) = 1.0;
    initial_state_covariance_(1) = 1.0;
    initial_state_covariance_(2) = 1.0;
    initial_state_covariance_(3) = 1.0;
    initial_state_covariance_(4) = 1.0;
    initial_state_covariance_(5) = 1.0;
    initial_state_covariance_(6) = 1.0;
    state_covariance_.setZero();

    process_noise_covariance_.setIdentity();
    process_noise_covariance_ *= 10.0;
    measurement_covariance_.setIdentity();

    initialized_ = true;
    ROS_INFO("Kalman Filter Initialized!");
}

void KFDisturbanceObserver::reset()
{
  state_covariance_ = initial_state_covariance_.asDiagonal();
  state_.setZero();
}

void KFDisturbanceObserver::calculateKalmanMatrices(Eigen::Matrix<double, kStateSizeKalman, kStateSizeKalman> *F_,
                                                    Eigen::Matrix<double, kStateSizeKalman, kInputSize> *G_,
                                                    Eigen::Matrix<double, kMeasurementSizeKalman, kStateSizeKalman> *H_)
{
  Eigen::MatrixXd tempF, tempG, tempH;
  tempF.resize(kStateSizeKalman, kStateSizeKalman);
  tempF << model_A_, model_Bd_,
      Eigen::MatrixXd::Zero(kDisturbanceSize, kStateSize), Eigen::MatrixXd::Identity(kDisturbanceSize, kDisturbanceSize);
  (*F_) = tempF;

  tempG.resize(kStateSizeKalman, kInputSize);
  tempG << model_B_, Eigen::MatrixXd::Zero(kDisturbanceSize, kInputSize);
  (*G_) = tempG;

  tempH.resize(kMeasurementSizeKalman, kStateSizeKalman);
  tempH.setIdentity();
  tempH(6) = 0.0;
  (*H_) = tempH;

}

void KFDisturbanceObserver::setSystemMatrices(Eigen::Matrix<double, kStateSize, kStateSize> model_A,
                       Eigen::Matrix<double, kStateSize, kInputSize> model_B,
                       Eigen::Matrix<double, kStateSize, kDisturbanceSize> model_Bd)
{
  this -> model_A_ = model_A;
  this -> model_B_ = model_B;
  this -> model_Bd_ = model_Bd;
  calculateKalmanMatrices(&F_, &G_, &H_);
}

bool KFDisturbanceObserver::updateEstimator() {
  if (!initialized_)
    return false;

  ROS_INFO_ONCE("KF is updated for first time.");
  static ros::Time t_previous = ros::Time::now();
  static bool do_once = true;
  double dt;

  if (do_once) {
    dt = 0.01;
    do_once = false;
  } else {
    ros::Time t0 = ros::Time::now();
    dt = (t0 - t_previous).toSec();
    t_previous = t0;
  }

  //check that dt is not so different from 0.01
  if (dt > 0.015) {
    dt = 0.015;
  }

  if (dt < 0.005) {
    dt = 0.005;
  }

  // P_k^- = F_(k-1) * P_(k-1)^+ * F_(k-1)^T + process_noise_covariance(Q_(k-1))
  state_covariance_ = F_ * state_covariance_ * F_.transpose();
  state_covariance_ += process_noise_covariance_;

  // update the Kalman gain
  Eigen::Matrix<double, kMeasurementSizeKalman, kMeasurementSizeKalman> tmp;
  tmp = H_ * state_covariance_ * H_.transpose() + measurement_covariance_;
  K_ = state_covariance_ * H_.transpose() * tmp.inverse();

  // update system states "state_"
  simulateSystem();

  // update with measurements
  state_ += (K_ * measurements_) - (K_ * H_ * state_);// (measurements_ - H_ * state_);

  //Update covariance
  Eigen::Matrix<double, kStateSizeKalman, kStateSizeKalman> helper_var;
  helper_var = Eigen::Matrix<double, kStateSizeKalman, kStateSizeKalman>::Identity() - K_ * H_;
  state_covariance_ = helper_var * state_covariance_ * helper_var.transpose()
      + K_ * measurement_covariance_ * K_.transpose();

  //Limits on estimated_disturbances
  if (!state_.allFinite()) {
    ROS_ERROR("The estimated state in KF Disturbance Observer has a non-finite element");
    return false;
  }

  // TODO limiting maybe
  return true;
}

void KFDisturbanceObserver::simulateSystem() {
  StateVector old_state;
  old_state = state_;
  state_ = F_ * old_state + G_ * command_moving_masses_;
}

void KFDisturbanceObserver::getEstimatedState(Eigen::VectorXd *estimated_state) const {
  assert(estimated_state);
  assert(initialized_);

  estimated_state->resize(kStateSizeKalman);
  *estimated_state = this->state_;
}

KFDisturbanceObserver::~KFDisturbanceObserver()
{
}

}