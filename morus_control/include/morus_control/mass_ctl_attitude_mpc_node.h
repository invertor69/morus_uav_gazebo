#ifndef PROJECT_MASS_CTL_ATTITUDE_MPC_NODE_H
#define PROJECT_MASS_CTL_ATTITUDE_MPC_NODE_H

#include <math.h>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Imu.h"
#include "rosgraph_msgs/Clock.h"
#include "control_msgs/JointControllerState.h" // for moving masses states
#include "morus_msgs/CommandMovingMasses.h"
#include "morus_msgs/AngleAndAngularVelocity.h"

#include "morus_control/mass_ctl_attitude_mpc.h"

namespace mav_control_attitude {
    class MPCAttitudeControllerNode{

    public:
        MPCAttitudeControllerNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
        ~MPCAttitudeControllerNode();
        void run();

    private:
        ros::NodeHandle nh_, private_nh_;

        MPCAttitudeController linear_mpc_roll_, linear_mpc_pitch_;

        // calculation of the future input signals
        virtual bool calculateMovingMassesCommand(Eigen::Matrix<double, 2, 1>* moving_masses_command,
                                                  MPCAttitudeController* linear_mpc_commanded_angle);

        // publishers
        ros::Publisher pub_mass0_;
        ros::Publisher pub_mass1_;
        ros::Publisher pub_mass2_;
        ros::Publisher pub_mass3_;

        void publishCommands();
        // debugging publisher
        ros::Publisher pub_angle_state_;
        // variables to hold the setpoints for the moving masses
        Eigen::Matrix<double, 2, 1> mass_roll_commands_;
        Eigen::Matrix<double, 2, 1> mass_pitch_commands_;
        bool start_flag_;

        // subscribers
        ros::Subscriber imu_subscriber_;
            void AhrsCallback(const sensor_msgs::Imu& msg);
            struct euler_mv {
                double x;
                double y;
                double z;
            } euler_mv_;
            struct euler_rate_mv {
                double x;
                double y;
                double z;
            } euler_rate_mv_;
            bool imu_received_; // received msg from imu

        ros::Subscriber mot_vel_ref_subscriber_;
            void MotVelRefCallback(const std_msgs::Float32& msg);
            float w_sp_;

        ros::Subscriber euler_ref_subscriber_;
            void EulerRefCallback(const geometry_msgs::Vector3& msg);
            geometry_msgs::Vector3 euler_sp_;

        ros::Subscriber clock_subscriber_;
            void ClockCallback(const rosgraph_msgs::Clock& msg);
            rosgraph_msgs::Clock clock_read_;

        ros::Subscriber movable_mass_0_state_subscriber_;
            void MovingMass0Callback(const control_msgs::JointControllerState& msg);
            double movable_mass_0_position_;
            double movable_mass_0_speed_;
            bool movable_mass_0_state_received_;

        ros::Subscriber movable_mass_1_state_subscriber_;
            void MovingMass1Callback(const control_msgs::JointControllerState& msg);
            double movable_mass_1_position_;
            double movable_mass_1_speed_;
            bool movable_mass_1_state_received_;

        ros::Subscriber movable_mass_2_state_subscriber_;
            void MovingMass2Callback(const control_msgs::JointControllerState& msg);
            double movable_mass_2_position_;
            double movable_mass_2_speed_;
            bool movable_mass_2_state_received_;

        ros::Subscriber movable_mass_3_state_subscriber_;
            void MovingMass3Callback(const control_msgs::JointControllerState& msg);
            double movable_mass_3_position_;
            double movable_mass_3_speed_;
            bool movable_mass_3_state_received_;

        // debug info
        bool verbose_;
    };
}
#endif //PROJECT_MASS_CTL_ATTITUDE_MPC_NODE_H
