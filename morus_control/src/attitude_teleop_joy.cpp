#include <morus_control/attitude_teleop_joy.h>

namespace mav_control_attitude {

  AttitudeJoy::AttitudeJoy(const ros::NodeHandle &nh,
                           const ros::NodeHandle &private_nh)
            :nh_(nh),
             private_nh_(private_nh)
  {
    vel_pub_ = nh_.advertise<geometry_msgs::Vector3>("euler_ref", 1);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 10, &AttitudeJoy::joyCallback, this);
  }

  AttitudeJoy::~AttitudeJoy()
  {
  }

  void AttitudeJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
  {
    geometry_msgs::Vector3 euler_ref_msg;
    euler_ref_msg.x = -0.1*joy->axes[0];
    euler_ref_msg.y =  0.1*joy->axes[1];
    vel_pub_.publish(euler_ref_msg);
  }
}
