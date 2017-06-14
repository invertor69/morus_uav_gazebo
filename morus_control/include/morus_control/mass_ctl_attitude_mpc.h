//
// Created by luka on 11.06.17..
//

#ifndef PROJECT_MASS_CTL_ATTITUDE_MPC_H
#define PROJECT_MASS_CTL_ATTITUDE_MPC_H

#include <cmath>

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/unsupported/Eigen/src/MatrixFunctions/MatrixExponential.h>

namespace mav_control_attitude {
    // constants
    constexpr int kStateSize = 8;
    constexpr int kInputSize = 4;
    //static constexpr int kMeasurementSize = 6;
    constexpr int kDisturbanceSize = 1;
    constexpr int kPredictionHorizonSteps = 20;
    constexpr double kGravity = 9.80665;

    // sampling time parameters
    constexpr double sampling_time_ = 0.01;
    constexpr double prediction_sampling_time_ = 0.01;
    // TODO prebacit u YAML FILE pa da se dobije sa get_param


class MPCAttitudeController {
    public:
    MPCAttitudeController(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~MPCAttitudeController();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:

    // ros node handles
    ros::NodeHandle nh_, private_nh_;

    //initialize system
    void initializeSystem();
    bool initialized_parameters_;

    // system model
    // Model: A, B, Bd
    // x(k+1) = A*x(k) + B*u(k) + Bd*d(k)
    Eigen::Matrix<double, kStateSize, kStateSize> model_A_;   //dynamics matrix
    Eigen::Matrix<double, kStateSize, kInputSize> model_B_;   //transfer matrix
    Eigen::Matrix<double, kStateSize, kDisturbanceSize> model_Bd_;  //Disturbance transfer  gas motor paramsmatrix

    // quadrotor params with moving masses
    double mass_;        // mass of a movable mass
    double mass_quad_;   // mass of the quadrotor body (including gas motors)
    double M_;           // total mass
    double mi_;          // additional constant for equations
    double cd_;          // drag constant (translational)
    double zr_;          // added coefficient for flapping
    double beta_;        // inclination angle of the motor arms
    double beta_gm_;     // this is additional angle of the gas motor prop w.r.t. the motor arm
    double zm_;          // mass displacement iz z-axis
    double Km_;          // voltage to acceleration
    double lm_;          // mass path maximal length
    double arm_offset_;  // motor arm offset from the origin
    double l_;           // motor arm length
    double Tr_;          // transmission rate
    Eigen::Matrix3d Iq_; // moment of inertia of the quadrotor body (without masses)
    double Iyy_b_;
    double Iyy_;

    double Tgm_;  // time constant
    double w_gm_n_; // rpm to rad/s
    double F_n_;
    double b_gm_f_;
    double b_gm_m_; // lucky guess
    double w_gm_0_;
    double F0_;

    // moving mass dynamics parameters (w_mm and zeta_mm)
    double zeta_mm_;
    double tm_;
    double w_mm_;
};
}
#endif //PROJECT_MASS_CTL_ATTITUDE_MPC_H
