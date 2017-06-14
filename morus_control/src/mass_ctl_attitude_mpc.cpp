// MPC control with moving masses for the MORUS project
// MPC algorithm file
// master thesis
// author: Luka Pevec

#include <morus_control/mass_ctl_attitude_mpc.h>

namespace mav_control_attitude {
    MPCAttitudeController::MPCAttitudeController(const ros::NodeHandle& nh,
                                                 const ros::NodeHandle& private_nh)
            : nh_(nh),
              private_nh_(private_nh),
              initialized_parameters_(false)  // after call of the "initializedSystem" it gets to "true"
    {
        initializeSystem(); // init the system and its parameters
    }

    MPCAttitudeController::~MPCAttitudeController() { }

    void MPCAttitudeController::initializeSystem() {
        mass_ = 1.0;
        mass_quad_ = 30.8;
        M_ = mass_quad_ + 4 * mass_;
        mi_ = mass_ / mass_quad_;
        cd_ = 1.5;
        zr_ = 0.2;
        beta_ = 0;
        beta_gm_ = 0;
        zm_ = 0.05;
        Km_ = 1;

        lm_ = 0.6;
        arm_offset_ = 0.1;
        l_ = lm_ + arm_offset_;
        Tr_ = 100;
        double Iq_xx = 5.5268 + 0.2;
        double Iq_yy = 5.5268 + 0.2;
        double Iq_zz = 6.8854 + 0.4;
        Iq_ = Eigen::Matrix3d::Zero(3,3);
        Iq_(0,0) = Iq_xx;
        Iq_(1,1) = Iq_yy;
        Iq_(2,2) = Iq_zz;
        Iyy_b_ = 8.3;
        Iyy_ = Iyy_b_ + 2*mass_*pow(lm_/2, 2);

        Tgm_ = 0.2;
        w_gm_n_ = 7000 / 60 * 2*M_PI;
        F_n_ = 25 * kGravity;
        b_gm_f_ = F_n_ / (pow(w_gm_n_,2));
        b_gm_m_ = 0.01;

        w_gm_0_ = sqrt(M_ * kGravity / 4.0 / b_gm_f_);
        F0_ = b_gm_f_ * pow(w_gm_0_,2);

        zeta_mm_ = 0.6551;
        w_mm_ = 14.8508;

        // construct model matrices
        Eigen::MatrixXd A_continous_time(kStateSize, kStateSize);
        A_continous_time = Eigen::MatrixXd::Zero(kStateSize, kStateSize);
        Eigen::MatrixXd B_continous_time(kStateSize, kInputSize);
        B_continous_time = Eigen::MatrixXd::Zero(kStateSize, kInputSize);
        Eigen::MatrixXd Bd_continous_time(kStateSize, kDisturbanceSize);
        Bd_continous_time = Eigen::MatrixXd::Zero(kStateSize, kDisturbanceSize);

        // states: (x)
        // [x1, dx1, x3, dx3, F1, F3, theta, dtheta] -> A is [8,8]
        // input signals: (u)
        // ['x1_ref (m)'; 'x3_ref (m)'; 'd_omega1 (rad/s)'; 'd_omega3 (rad/s)'] -> B is [8,4]
        A_continous_time(0,1) = 1.0;
        A_continous_time(1,0) = -pow(w_mm_,2);
        A_continous_time(1,1) = -2.0*zeta_mm_*w_mm_;
        A_continous_time(2,3) = 1.0;
        A_continous_time(3,2) = -pow(w_mm_,2);
        A_continous_time(3,3) = -2.0*zeta_mm_*w_mm_;
        A_continous_time(4,4) = -1.0/Tgm_;
        A_continous_time(5,5) = -1.0/Tgm_;
        A_continous_time(6,7) = 1.0;
        A_continous_time(7,0) = mass_ / Iyy_ * (kGravity + ((1.0-mi_)*zm_*pow(w_mm_,2)));
        A_continous_time(7,1) = 2.0 *mass_*(1.0-mi_)*zm_*zeta_mm_*w_mm_/Iyy_;
        A_continous_time(7,2) = mass_ / Iyy_ * (kGravity + ((1.0-mi_)*zm_*pow(w_mm_,2)));
        A_continous_time(7,3) = 2.0 *mass_*(1.0-mi_)*zm_*zeta_mm_*w_mm_/Iyy_;
        A_continous_time(7,4) = 0.92 / (0.92 * Iyy_);
        A_continous_time(7,5) = -0.92 / (0.92 * Iyy_);

        B_continous_time(1,0) = pow(w_mm_,2);
        B_continous_time(3,1) = pow(w_mm_,2);
        B_continous_time(4,2) =  2.0 * b_gm_f_ * w_gm_0_ / Tgm_;
        B_continous_time(5,3) =  2.0 * b_gm_f_ * w_gm_0_ / Tgm_;
        B_continous_time(7,0) = -mass_ * (1.0-mi_)*zm_*pow(w_mm_,2) / Iyy_;
        B_continous_time(7,1) = -mass_ * (1.0-mi_)*zm_*pow(w_mm_,2) / Iyy_;

        // disturbance on angular speed speed [dtheta]
        Bd_continous_time(6, 0) = 1.0;

        // discretization of matrix A
        model_A_ = (prediction_sampling_time_ * A_continous_time).exp();

        // calculate equation (42) from
        // "Model predictive control for Trajectory Tracking of Unmanned Aerial Vehicles Using Robot Operating System"
        Eigen::MatrixXd integral_exp_A;
        integral_exp_A = Eigen::MatrixXd::Zero(kStateSize, kStateSize);
        const int count_integral_A = 100;

        // discrete integration
        for (int i = 0; i < count_integral_A; i++) {
            integral_exp_A += (A_continous_time * prediction_sampling_time_ * i / count_integral_A).exp()
                              * prediction_sampling_time_ / count_integral_A;
        }

        // making the discrete matrices B and C
        model_B_ = integral_exp_A * B_continous_time;
        model_Bd_ = integral_exp_A * Bd_continous_time;

        // TODO init the solver and steady state calculation
        //steady_state_calculation_.initialize(model_A_, model_B_, model_Bd_);

        /*
        if (verbose_) {
            ROS_INFO_STREAM("A: \n" << model_A_);
            ROS_INFO_STREAM("B: \n" << model_B_);
            ROS_INFO_STREAM("B_d: \n" << model_Bd_);
        }
        */

        //Solver initialization
        //set_defaults();
        //setup_indexing();

        //Solver settings
        //settings.verbose = 0;

        //Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.A), kStateSize, kStateSize) = model_A_;
        //Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.B), kStateSize, kInputSize) = model_B_;
        //Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.Bd), kStateSize, kDisturbanceSize) =
        //        model_Bd_;

        initialized_parameters_ = true;

        ROS_INFO("Linear MPC attitude controller: initialized correctly");
    }

}
