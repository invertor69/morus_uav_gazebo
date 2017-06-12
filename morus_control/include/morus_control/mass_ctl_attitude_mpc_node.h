#ifndef PROJECT_MASS_CTL_ATTITUDE_MPC_NODE_H
#define PROJECT_MASS_CTL_ATTITUDE_MPC_NODE_H

#include "ros/ros.h"
#include "std_msgs/Float64.h"

namespace mav_control_attitude {
    class MPCAttitudeControllerNode{
    public:
        MPCAttitudeControllerNode(const ros::NodeHandle& nh, const ros::NodeHandle private_nh);
        ~MPCAttitudeControllerNode();
        void Publish();

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        // publishers
        ros::Publisher pub_mass0_;
        ros::Publisher pub_mass1_;
        ros::Publisher pub_mass2_;
        ros::Publisher pub_mass3_;

        float count;
    };
}
#endif //PROJECT_MASS_CTL_ATTITUDE_MPC_NODE_H
