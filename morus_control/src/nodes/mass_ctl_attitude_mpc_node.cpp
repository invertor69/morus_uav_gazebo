#include <morus_control/mass_ctl_attitude_mpc_node.h>

namespace mav_control_attitude {
MPCAttitudeControllerNode::MPCAttitudeControllerNode(const ros::NodeHandle& nh,
                                                     const ros::NodeHandle private_nh)
        : nh_(nh),
          private_nh_(private_nh)
{
    // Publishers ( nh -> )
    pub_mass0_ = nh_.advertise<std_msgs::Float64>("movable_mass_0_position_controller/command", 1);
    pub_mass1_ = nh_.advertise<std_msgs::Float64>("movable_mass_1_position_controller/command", 1);
    pub_mass2_ = nh_.advertise<std_msgs::Float64>("movable_mass_2_position_controller/command", 1);
    pub_mass3_ = nh_.advertise<std_msgs::Float64>("movable_mass_3_position_controller/command", 1);

    count = 0.0;

    // Subscribers ( nh <- )
    //ros::Subscriber sub = nh.subscribe("imu", 1, chatterCallback);
}

MPCAttitudeControllerNode::~MPCAttitudeControllerNode()
{
}

void MPCAttitudeControllerNode::Publish() {
    // MAIN ALGORITHM THAT RUNS FOREVER !
    std_msgs::Float64 msg;
    msg.data = count;

    // publish the right control actions
    pub_mass0_.publish(msg);
    pub_mass1_.publish(msg);
    pub_mass2_.publish(msg);
    pub_mass3_.publish(msg);

    count+= 1.0;
}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mav_attitude_ctl_mpc");

    // fully initialize the node
    ros::NodeHandle nh, private_nh("~");

    mav_control_attitude::MPCAttitudeControllerNode MPC_attitude_controller(nh, private_nh);

    ros::Rate loop_rate(100); // 100 Hz -> Ts = 0.01 s
    while (ros::ok()){
        ros::spinOnce(); // get all msg-s from subscribers
        MPC_attitude_controller.Publish();
        loop_rate.sleep(); //
    }


    return 0;
}