#include <morus_control/mass_ctl_attitude_mpc_node.h>

namespace mav_control_attitude {
MPCAttitudeControllerNode::MPCAttitudeControllerNode(const ros::NodeHandle& nh,
                                                     const ros::NodeHandle private_nh)
        : nh_(nh),
          private_nh_(private_nh)
{
    count_ = 0.0; // temp variable for the publishers
    start_flag_ = false;            // flag indicates if the first measurement is received

    // Publishers  ( nh -> )
    pub_mass0_ = nh_.advertise<std_msgs::Float64>("movable_mass_0_position_controller/command", 1);
    pub_mass1_ = nh_.advertise<std_msgs::Float64>("movable_mass_1_position_controller/command", 1);
    pub_mass2_ = nh_.advertise<std_msgs::Float64>("movable_mass_2_position_controller/command", 1);
    pub_mass3_ = nh_.advertise<std_msgs::Float64>("movable_mass_3_position_controller/command", 1);

    // Subscribers ( nh <- )
    imu_ = nh_.subscribe("imu", 1, &MPCAttitudeControllerNode::ahrs_cb, this);
    mot_vel_ref_ = nh_.subscribe("mot_vel_ref", 1, &MPCAttitudeControllerNode::mot_vel_ref_cb, this);
    euler_ref_ = nh_.subscribe("euler_ref", 1, &MPCAttitudeControllerNode::euler_ref_cb, this);
    clock_ = nh_.subscribe("/clock", 1, &MPCAttitudeControllerNode::clock_cb, this);
}

MPCAttitudeControllerNode::~MPCAttitudeControllerNode() {
}

void MPCAttitudeControllerNode::ahrs_cb(const sensor_msgs::Imu &msg) {
    /**
    AHRS callback. Used to extract roll, pitch, yaw and their rates.
            We used the following order of rotation - 1)yaw, 2) pitch, 3) roll
    :param msg: Type sensor_msgs/Imu
    */
    if (!start_flag_){
        start_flag_ = true;
    }

    // read the msg
    double qx = msg.orientation.x;
    double qy = msg.orientation.y;
    double qz = msg.orientation.z;
    double qw = msg.orientation.w;

    // conversion quaternion to euler (yaw - pitch - roll)
    euler_mv_.x = atan2(2 * (qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz);
    euler_mv_.y =  asin(2 * (qw * qy - qx * qz));
    euler_mv_.z = atan2(2 * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz);

    // gyro measurements (p,q,r)
    double p = msg.angular_velocity.x;
    double q = msg.angular_velocity.y;
    double r = msg.angular_velocity.z;

    double sx = sin(euler_mv_.x); // sin(roll)
    double cx = cos(euler_mv_.x); // cos(roll)
    double cy = cos(euler_mv_.y); // cos(pitch)
    double ty = tan(euler_mv_.y); // cos(pitch)

    // conversion gyro measurements to roll_rate, pitch_rate, yaw_rate
    euler_rate_mv_.x = p + sx * ty * q + cx * ty * r;
    euler_rate_mv_.y = cx * q - sx * r;
    euler_rate_mv_.z = sx / cy * q + cx / cy * r;
}

void MPCAttitudeControllerNode::mot_vel_ref_cb(const std_msgs::Float32 &msg) {
    /**
    Referent motor velocity callback. (This should be published by height controller).
    :param msg: Type Float32
    */
    w_sp_ = msg.data;

}

void MPCAttitudeControllerNode::euler_ref_cb(const geometry_msgs::Vector3 &msg) {
    /**
    Euler ref values callback.
    :param msg: Type Vector3 (x-roll, y-pitch, z-yaw)
    */
    euler_sp_ = msg;
}

void MPCAttitudeControllerNode::clock_cb(const rosgraph_msgs::Clock& msg) {
    clock_read_ = msg;
}

void MPCAttitudeControllerNode::Publish() {
    // MAIN ALGORITHM THAT RUNS FOREVER !
    std_msgs::Float64 msg;
    msg.data = count_;

    // publish the right control actions
    pub_mass0_.publish(msg);
    pub_mass1_.publish(msg);
    pub_mass2_.publish(msg);
    pub_mass3_.publish(msg);
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
        loop_rate.sleep(); // watch that sampling time is correct
    }

    return 0;
}