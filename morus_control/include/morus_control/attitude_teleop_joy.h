#ifndef PROJECT_ATTITUDE_TELEOP_JOY_H
#define PROJECT_ATTITUDE_TELEOP_JOY_H

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h> // publishing msg
#include <sensor_msgs/Joy.h>       // receiving msg

namespace mav_control_attitude {
  class AttitudeJoy{

   public:
       AttitudeJoy(const ros::NodeHandle& nh,
                   const ros::NodeHandle& private_nh);
       ~AttitudeJoy();

   private:
       ros::NodeHandle nh_, private_nh_;

       ros::Publisher vel_pub_;
       ros::Subscriber joy_sub_;

       void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  };
}

#endif //PROJECT_ATTITUDE_TELEOP_JOY_H
