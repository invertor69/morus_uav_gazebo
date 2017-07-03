//
// Created by luka on 11.06.17..
//

#ifndef PROJECT_MASS_CTL_ATTITUDE_MPC_H
#define PROJECT_MASS_CTL_ATTITUDE_MPC_H

#include <morus_control/KFDisturbanceObserver.h>
#include <morus_control/steady_state_calculation.h>

#include "math.h"
#include "geometry_msgs/Vector3.h"
#include "rosgraph_msgs/Clock.h"
#include "control_msgs/JointControllerState.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int64.h"

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <unsupported/Eigen/src/MatrixFunctions/MatrixExponential.h>

// CVXGEN solver
#include <morus_control/solver.h>

namespace mav_control_attitude {
    // constants
    // CC_MPC
    /*
    constexpr int kStateSize = 8;
    constexpr int kInputSize = 4;
    constexpr int kMeasurementSize = 1;
    constexpr int kDisturbanceSize = 2;
    */

    // MM_MPC
    constexpr int kStateSize = 6;       // [x1, dx1, x3, dx3, theta, dtheta] -> A is [6,6]
    constexpr int kInputSize = 2;       // [x1_ref (m), x3_ref (m)]          -> B is [6,2]
    constexpr int kMeasurementSize = 1; // [theta] -> C is [1,6]
    constexpr int kDisturbanceSize = 6; // [theta, dtheta] -> B_d is [6,2]

    constexpr int kPredictionHorizonSteps = 20;
    constexpr double kGravity = 9.80665;

    // sampling time parameters
    constexpr double sampling_time_ = 0.1;
    constexpr double prediction_sampling_time_ = 0.1;
    // TODO prebacit u YAML FILE pa da se dobije sa get_param


class MPCAttitudeController {
 public:
    MPCAttitudeController(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~MPCAttitudeController();

    // After dynamic change update the parameters
    void applyParameters();

    // compute control input
    void calculateMovingMassesCommand(Eigen::Vector2d* moving_mass_ref);


    // setters
    void setAngleRef(double angle_sp)
    {
      angle_sp_ = angle_sp;
    }

    void setClock(rosgraph_msgs::Clock clock)
    {
      clock_read_ = clock;
    }

    void setMovingMassState(control_msgs::JointControllerState msg,
                            int number_moving_mass,
                            double gain_reading)
    {
      switch(number_moving_mass){
        case 0 : movable_mass_0_position_ = gain_reading * msg.process_value;
                 movable_mass_0_speed_ =    gain_reading * msg.process_value_dot;
                 break;
        case 1 : movable_mass_1_position_ = gain_reading * msg.process_value;
                 movable_mass_1_speed_ =    gain_reading * msg.process_value_dot;
                 break;
      }
    }

    void setAngleState(double angle)
    {
      angle_ = angle;
    }

    void setIntegratorConstantMPC(double K_I_MPC_angle)
    {
      K_I_MPC_angle_ = K_I_MPC_angle;
    }

    void setAngularVelocityState(double angular_velocity)
    {
      angular_velocity_ = angular_velocity;
    }

    void setControllerName(std::string controller_name)
    {
      controller_name_ = controller_name;
    }


    // getters
    std::string getControllerName()
    {
      return controller_name_;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
    // ros node handles
    ros::NodeHandle nh_, private_nh_;

    // publishers for debugging
    ros::Publisher target_state_pub_;
    ros::Publisher target_input_pub_;
    ros::Publisher disturbances_pub_;
    ros::Publisher MPC_solver_status_pub_;

    //initialize system
    void initializeParameters();
    bool initialized_parameters_;

    // controller variables
    double angle_sp_;
    rosgraph_msgs::Clock clock_read_;
    // states of the system
    double movable_mass_0_position_;
    double movable_mass_0_speed_;
    double movable_mass_1_position_;
    double movable_mass_1_speed_;
    double angle_;
    double angular_velocity_;

    // name
    std::string controller_name_;

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

    // steady state calculation
    SteadyStateCalculation steady_state_calculation_;

    // controller parameters
      // CC_MPC
      /*
      // state penalty
      Eigen::Vector3d q_position_;
      Eigen::Vector3d q_velocity_;

      Eigen::Vector4d q_moving_masses_;
      Eigen::Vector2d q_IC_motors_;
      Eigen::Vector2d q_attitude_;

      // control penalty
      Eigen::Vector4d r_command_; // u
      Eigen::Vector4d r_delta_command_; // du
      */

      // MM_MPC
      // state penalty
      Eigen::Vector4d q_moving_masses_;
      Eigen::Vector2d q_attitude_;

      // control penalty
      Eigen::Vector2d r_command_;
      Eigen::Vector2d r_delta_command_;

    // debug info
    bool verbose_;
    double solve_time_average_;

    // backup LQR
    Eigen::MatrixXd LQR_K_;
    Eigen::Vector2d moving_mass_ref_temp_;

    // CVXGEN solver parameters (needed if more MPC's are used)
    Params params_;
    Settings settings_;
    int solver_status_;

    // disturbance observer
    bool enable_offset_free_;
    bool enable_integrator_;
    bool initialized_observer_;
    Eigen::Matrix<double, kMeasurementSize, 1> angle_error_integration_;
    Eigen::Matrix<double, kDisturbanceSize, 1> estimated_disturbances_;
    KFDisturbanceObserver disturbance_observer_;

    // controller gains
    double K_I_MPC_angle_;
};
}
#endif //PROJECT_MASS_CTL_ATTITUDE_MPC_H
