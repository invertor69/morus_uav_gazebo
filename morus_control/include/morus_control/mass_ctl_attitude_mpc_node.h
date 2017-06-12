#ifndef PROJECT_MASS_CTL_ATTITUDE_MPC_NODE_H
#define PROJECT_MASS_CTL_ATTITUDE_MPC_NODE_H

#include <math.h>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Imu.h"
#include "rosgraph_msgs/Clock.h"

namespace mav_control_attitude {
    class MPCAttitudeControllerNode{
    public:
        MPCAttitudeControllerNode(const ros::NodeHandle& nh, const ros::NodeHandle private_nh);
        ~MPCAttitudeControllerNode();
        void Publish();

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        double count_;
        bool start_flag_;

        // publishers
        ros::Publisher pub_mass0_;
        ros::Publisher pub_mass1_;
        ros::Publisher pub_mass2_;
        ros::Publisher pub_mass3_;

        // subscribers
        ros::Subscriber imu_;
            void ahrs_cb        (const sensor_msgs::Imu& msg);
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

        ros::Subscriber mot_vel_ref_;
            void mot_vel_ref_cb (const std_msgs::Float32& msg);
            float w_sp_;

        ros::Subscriber euler_ref_;
            void euler_ref_cb   (const geometry_msgs::Vector3& msg);
            geometry_msgs::Vector3 euler_sp_;

        ros::Subscriber clock_;
            void clock_cb       (const rosgraph_msgs::Clock& msg);
            rosgraph_msgs::Clock clock_read_;
    };
}
#endif //PROJECT_MASS_CTL_ATTITUDE_MPC_NODE_H
