// MPC control with moving masses for the MORUS project
// ROS node init
// master thesis
// autor: Luka Pevec

#include <morus_uav_ros_msgs>
#include <morus_control/mass_ctl_attitude_mpc_node.h>

// specific msg definition
#include <sensor_msgs/Imu.h>

namespace mav_control_mpc {

    MPCAttitudeControllerNode::MPCAttitudeControllerNode(const ros::NodeHandle& nh,
                                                         const ros::NodeHandle private_nh)
            : nh_(nh),
              private_nh_(private_nh),
              got_first_attitude_command_(false),
              MPC_attitude_controller_(nh, private_nh)
    {

        // subscribers
        imu_sub_ = nh.subscribe('imu', 1, &MPCAttitudeControllerNode::ahrs_cb, this);
        /*
         * python subscribers
         rospy.Subscriber('imu', Imu, self.ahrs_cb)
            rospy.Subscriber('mot_vel_ref', Float32, self.mot_vel_ref_cb)
            rospy.Subscriber('euler_ref', Vector3, self.euler_ref_cb)
            rospy.Subscriber('/clock', Clock, self.clock_cb)
         */
        // publishers
        motor_velocity_reference_pub_ = nh_.advertise<mav_msgs::Actuators>(
                mav_msgs::default_topics::COMMAND_ACTUATORS, 1);
    }

    /*
    AHRS callback. Used to extract roll, pitch, yaw and their rates.
    We used the following order of rotation - 1)yaw, 2) pitch, 3) roll
        :param msg: Type sensor_msgs/Imu
    */ // TODO !!!!!!!!!!!!!!!!!!!!!!!!!
    /*
    void MPCAttitudeControllerNode::ahrs_cb(const sensor_msgs::Imu& msg)
    {

    }
    */
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mass_ctl_attitude_mpc");  // init the node
    ros::NodeHandle nh, private_nh("~");
    // create an instance of a node
    mav_control_mpc::MPCAttitudeControllerNode MPC_attitude_controller(nh, private_nh);
    ros::spin();  // start ROS
    return 0;
}
