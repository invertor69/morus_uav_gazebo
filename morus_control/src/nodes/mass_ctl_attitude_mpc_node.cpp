#include <morus_control/mass_ctl_attitude_mpc_node.h>

namespace mav_control_attitude {
    MPCAttitudeControllerNode::MPCAttitudeControllerNode(const ros::NodeHandle& nh,
                                                         const ros::NodeHandle& private_nh)
            : nh_(nh),
              private_nh_(private_nh),
              linear_mpc_roll_(nh_, private_nh_),
              linear_mpc_pitch_(nh_, private_nh_),
              start_flag_(false),  // flag for the first measurement
              verbose_(false)
    {

        // init the readings od moving mass sensors
        movable_mass_0_position_ = 0.0;
        movable_mass_1_position_ = 0.0;
        movable_mass_2_position_ = 0.0;
        movable_mass_3_position_ = 0.0;
        movable_mass_0_speed_ = 0.0;
        movable_mass_1_speed_ = 0.0;
        movable_mass_2_speed_ = 0.0;
        movable_mass_3_speed_ = 0.0;

        euler_sp_.x = 0.0;
        euler_sp_.y = 0.0;
        euler_sp_.z = 0.0;

        linear_mpc_roll_.applyParameters();
        linear_mpc_roll_.setControllerName("Roll controller");

        linear_mpc_pitch_.applyParameters();
        linear_mpc_pitch_.setControllerName("Pitch controller");

        // Publishers  ( nh -> )
        pub_mass0_ = nh_.advertise<std_msgs::Float64>("movable_mass_0_position_controller/command", 1);
        pub_mass1_ = nh_.advertise<std_msgs::Float64>("movable_mass_1_position_controller/command", 1);
        pub_mass2_ = nh_.advertise<std_msgs::Float64>("movable_mass_2_position_controller/command", 1);
        pub_mass3_ = nh_.advertise<std_msgs::Float64>("movable_mass_3_position_controller/command", 1);
        pub_angle_state_ = nh_.advertise<morus_msgs::AngleAndAngularVelocity>("angles", 1);

        // Subscribers ( nh <- )
        imu_subscriber_ = nh_.subscribe("imu", 1, &MPCAttitudeControllerNode::AhrsCallback, this); // measured values info
        mot_vel_ref_subscriber_ = nh_.subscribe("mot_vel_ref", 1, &MPCAttitudeControllerNode::MotVelRefCallback, this);
        euler_ref_subscriber_ = nh_.subscribe("euler_ref", 1, &MPCAttitudeControllerNode::EulerRefCallback, this); // reference for the angles
        clock_subscriber_ = nh_.subscribe("/clock", 1, &MPCAttitudeControllerNode::ClockCallback, this);  // internal clock variable
        // position of mass 0
        movable_mass_0_state_subscriber_= nh_.subscribe("movable_mass_0_position_controller/state", 1, &MPCAttitudeControllerNode::MovingMass0Callback, this);
        // position of mass 1
        movable_mass_1_state_subscriber_= nh_.subscribe("movable_mass_1_position_controller/state", 1, &MPCAttitudeControllerNode::MovingMass1Callback, this);
        // position of mass 2
        movable_mass_2_state_subscriber_= nh_.subscribe("movable_mass_2_position_controller/state", 1, &MPCAttitudeControllerNode::MovingMass2Callback, this);
        // position of mass 3
        movable_mass_3_state_subscriber_= nh_.subscribe("movable_mass_3_position_controller/state", 1, &MPCAttitudeControllerNode::MovingMass3Callback, this);

    }

    MPCAttitudeControllerNode::~MPCAttitudeControllerNode() {
    }

    void MPCAttitudeControllerNode::AhrsCallback(const sensor_msgs::Imu &msg) {
        /// @details AHRS callback. Used to extract roll, pitch, yaw and their rates.
        /// We used the following order of rotation - 1)yaw, 2) pitch, 3) roll
        /// @param msg: Type sensor_msgs::Imu

        ROS_INFO_ONCE("MPCAttitudeController got first odometry message.");

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
        double ty = tan(euler_mv_.y); // tan(pitch)

        // conversion gyro measurements to roll_rate, pitch_rate, yaw_rate
        euler_rate_mv_.x = p + sx * ty * q + cx * ty * r;
        euler_rate_mv_.y = cx * q - sx * r;
        euler_rate_mv_.z = sx / cy * q + cx / cy * r;

        // publish states in roll,pitch,jaw format
        morus_msgs::AngleAndAngularVelocity angles_velocities;
        angles_velocities.roll = (float)  euler_mv_.x;
        angles_velocities.pitch = (float) euler_mv_.y;
        angles_velocities.jaw = (float)   euler_mv_.z;

        angles_velocities.roll_dot = (float)  euler_rate_mv_.x;
        angles_velocities.pitch_dot = (float) euler_rate_mv_.y;
        angles_velocities.jaw_dot = (float)   euler_rate_mv_.z;
        angles_velocities.header.stamp = ros::Time::now();

        pub_angle_state_.publish(angles_velocities);

        // publish to check if calculation
        if (verbose_) {
          ROS_INFO_STREAM("angles: \n roll: " << euler_mv_.x <<
                                 "\n pitch: " << euler_mv_.y <<
                                   "\n jaw: " << euler_mv_.z);
        }

        // Calculation of the output

        // set the data to the controllers
        linear_mpc_roll_.setAngleState(euler_mv_.x);
        linear_mpc_roll_.setAngularVelocityState(euler_rate_mv_.x);

        linear_mpc_pitch_.setAngleState(euler_mv_.y);
        linear_mpc_pitch_.setAngularVelocityState(euler_rate_mv_.y);

        // calculate the control signals - MAIN ALGORITHM !!!!!
        calculateMovingMassesCommand(&mass_roll_commands_, &linear_mpc_roll_);
        calculateMovingMassesCommand(&mass_pitch_commands_, &linear_mpc_pitch_);

        std_msgs::Float64 mass0_command_msg, mass1_command_msg, mass2_command_msg, mass3_command_msg;
        mass0_command_msg.data =  mass_pitch_commands_(0);
        mass1_command_msg.data = -mass_roll_commands_(0);
        mass2_command_msg.data = -mass_pitch_commands_(1);
        mass3_command_msg.data =  mass_roll_commands_(1);

        // publish the new references for the masses
        pub_mass0_.publish(mass0_command_msg);
        pub_mass1_.publish(mass1_command_msg);
        pub_mass2_.publish(mass2_command_msg);
        pub_mass3_.publish(mass3_command_msg);
    }

    void MPCAttitudeControllerNode::MotVelRefCallback(const std_msgs::Float32 &msg) {
        /// @details Referent motor velocity callback. (This should be published by height controller).
        /// @param msg: Type std_msgs::Float32
        w_sp_ = msg.data;

    }

    void MPCAttitudeControllerNode::EulerRefCallback(const geometry_msgs::Vector3 &msg) {
        /// @details Euler ref values callback.
        /// @param msg: Type geometry_msgs::Vector3 (x-roll, y-pitch, z-yaw)
        euler_sp_ = msg;
        linear_mpc_roll_.setAngleRef(msg.x);
        linear_mpc_pitch_.setAngleRef(msg.y);
    }

    void MPCAttitudeControllerNode::ClockCallback(const rosgraph_msgs::Clock &msg) {
        /// @param msg
        clock_read_ = msg;
        linear_mpc_roll_.setClock(msg);
        linear_mpc_pitch_.setClock(msg);
    }

    void MPCAttitudeControllerNode::MovingMass0Callback(const control_msgs::JointControllerState& msg) {
      movable_mass_0_position_ = msg.process_value;
      movable_mass_0_speed_ = msg.process_value_dot;
      linear_mpc_pitch_.setMovingMassState(msg, 0, +1.0);
    }

    void MPCAttitudeControllerNode::MovingMass1Callback(const control_msgs::JointControllerState& msg) {
      movable_mass_1_position_ = msg.process_value;
      movable_mass_1_speed_ = msg.process_value_dot;
      linear_mpc_roll_.setMovingMassState(msg, 0, -1.0);
    }

    void MPCAttitudeControllerNode::MovingMass2Callback(const control_msgs::JointControllerState& msg) {
      movable_mass_2_position_ = msg.process_value;
      movable_mass_2_speed_ = msg.process_value_dot;
      linear_mpc_pitch_.setMovingMassState(msg, 1, -1.0);
    }

    void MPCAttitudeControllerNode::MovingMass3Callback(const control_msgs::JointControllerState& msg) {
      movable_mass_3_position_ = msg.process_value;
      movable_mass_3_speed_ = msg.process_value_dot;
      linear_mpc_roll_.setMovingMassState(msg, 1, +1.0);
    }

    bool MPCAttitudeControllerNode::calculateMovingMassesCommand(Eigen::Matrix<double, 2, 1>* moving_masses_command,
                                                                 MPCAttitudeController* linear_mpc_commanded_angle) {
      Eigen::Matrix<double, 2, 1> moving_mass_ref;
      (*linear_mpc_commanded_angle).calculateMovingMassesCommand(&moving_mass_ref);
      *moving_masses_command = moving_mass_ref;
      return true;
    }

    void MPCAttitudeControllerNode::run() {
      // define sampling time
      ros::Rate loop_rate(10); // 100 Hz -> Ts = 0.01 s
      while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
      }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mav_attitude_ctl_mpc");

    // fully initialize the node
    ros::NodeHandle nh, private_nh("~");

    mav_control_attitude::MPCAttitudeControllerNode MPC_attitude_controller_node(nh, private_nh);

    MPC_attitude_controller_node.run();

    return 0;
}