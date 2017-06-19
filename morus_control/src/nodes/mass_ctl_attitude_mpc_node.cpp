#include <morus_control/mass_ctl_attitude_mpc_node.h>

namespace mav_control_attitude {
    MPCAttitudeControllerNode::MPCAttitudeControllerNode(const ros::NodeHandle& nh,
                                                         const ros::NodeHandle& private_nh)
            : nh_(nh),
              private_nh_(private_nh),
              linear_mpc_(nh_, private_nh_),
              start_flag_(false),  // flag for the first measurement
              verbose_(false)
    {

        // temp variables for the publishers
        mass_0_reff_ = 0.0;
        mass_1_reff_ = -0.0;
        mass_2_reff_ = -0.0;
        mass_3_reff_ = 0.0;

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

        linear_mpc_.apllyParameters(); // aplly after the dynamic change! // TODO

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

        // publish to check if calculation
        if (verbose_) {
            ROS_INFO_STREAM("angles: \n roll: " << euler_mv_.x <<
                                   "\n pitch: " << euler_mv_.y <<
                                     "\n jaw: " << euler_mv_.z);
        }

        morus_msgs::AngleAndAngularVelocity angles_velocities;
        angles_velocities.roll = (float)  euler_mv_.x;
        angles_velocities.pitch = (float) euler_mv_.y;
        angles_velocities.jaw = (float)   euler_mv_.z;

        angles_velocities.roll_dot = (float)  euler_rate_mv_.x;
        angles_velocities.pitch_dot = (float) euler_rate_mv_.y;
        angles_velocities.jaw_dot = (float)   euler_rate_mv_.z;
        angles_velocities.header.stamp = ros::Time::now();

        pub_angle_state_.publish(angles_velocities);

        // Calculation of the output
        // input variables "euler_mv_"(angle) and "euler_rate_mv_"(angular velocity)

        Eigen::Matrix<double, kStateSize, 1> target_state;
        Eigen::Matrix<double, kStateSize, 1> current_state;

        // roll
        target_state(0,0) = 0.0;
        target_state(1,0) = 0.0;
        target_state(2,0) = 0.0;
        target_state(3,0) = 0.0;
        target_state(4,0) = euler_sp_.x;
        target_state(5,0) = 0.0;

        current_state(0,0) = movable_mass_1_position_;
        current_state(1,0) = movable_mass_1_speed_;
        current_state(2,0) = movable_mass_3_position_;
        current_state(3,0) = movable_mass_3_speed_;
        current_state(4,0) = euler_mv_.x;
        current_state(5,0) = euler_rate_mv_.x;

        mass_x_commands = linear_mpc_.LQR_K_ * (target_state - current_state);
        // min limits
        Eigen::Vector2d lower_limits_roll;
        lower_limits_roll << -0.29, -0.29;
        mass_x_commands = mass_x_commands.cwiseMax(lower_limits_roll);
        // max limits
        Eigen::Vector2d upper_limits_roll;
        upper_limits_roll << 0.29, 0.29;
        mass_x_commands = mass_x_commands.cwiseMin(upper_limits_roll);
        mass_1_reff_ = mass_x_commands(0);
        mass_3_reff_ = mass_x_commands(1);

        // pitch
        target_state(0,0) = 0.0;
        target_state(1,0) = 0.0;
        target_state(2,0) = 0.0;
        target_state(3,0) = 0.0;
        target_state(4,0) = euler_sp_.y;
        target_state(5,0) = 0.0;

        current_state(0,0) = movable_mass_0_position_;
        current_state(1,0) = movable_mass_0_speed_;
        current_state(2,0) = movable_mass_2_position_;
        current_state(3,0) = movable_mass_2_speed_;
        current_state(4,0) = euler_mv_.y;
        current_state(5,0) = euler_rate_mv_.y;

        mass_y_commands = linear_mpc_.LQR_K_ * (target_state - current_state);
        // min limits
        Eigen::Vector2d lower_limits_pitch;
        lower_limits_pitch << -0.29, -0.29;
        mass_y_commands = mass_y_commands.cwiseMax(lower_limits_pitch);
        // max limits
        Eigen::Vector2d upper_limits_pitch;
        upper_limits_pitch << 0.29, 0.29;
        mass_y_commands = mass_y_commands.cwiseMin(upper_limits_pitch);
        mass_0_reff_ = mass_y_commands(0);
        mass_2_reff_ = mass_y_commands(1);

        // TODO calculate the feedback and form the selected structure !!!! line 114 in "linear_mpc_node.cpp"

        // TODO calculate the outputs for the masses using the current "euler_mv_" and "euler_rate_mv_"
        // TODO init the solver
        // TODO express the outputs
        std_msgs::Float64 mass0_command_msg, mass1_command_msg, mass2_command_msg, mass3_command_msg;
        mass0_command_msg.data = mass_0_reff_;
        mass1_command_msg.data = -mass_1_reff_;
        mass2_command_msg.data = -mass_2_reff_;
        mass3_command_msg.data = mass_3_reff_;

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
    }

    void MPCAttitudeControllerNode::ClockCallback(const rosgraph_msgs::Clock &msg) {
        /// @param msg
        clock_read_ = msg;
    }

    void MPCAttitudeControllerNode::MovingMass0Callback(const control_msgs::JointControllerState& msg) {
      movable_mass_0_position_ = msg.process_value;
      movable_mass_0_speed_ = msg.process_value_dot;
    }

    void MPCAttitudeControllerNode::MovingMass1Callback(const control_msgs::JointControllerState& msg) {
      movable_mass_1_position_ = msg.process_value;
      movable_mass_1_speed_ = msg.process_value_dot;
    }

    void MPCAttitudeControllerNode::MovingMass2Callback(const control_msgs::JointControllerState& msg) {
      movable_mass_2_position_ = msg.process_value;
      movable_mass_2_speed_ = msg.process_value_dot;
    }

    void MPCAttitudeControllerNode::MovingMass3Callback(const control_msgs::JointControllerState& msg) {
      movable_mass_3_position_ = msg.process_value;
      movable_mass_3_speed_ = msg.process_value_dot;
    }

    bool MPCAttitudeControllerNode::calculateMovingMassesCommand(morus_msgs::CommandMovingMasses* moving_masses_command) {
      Eigen::Vector4d moving_mass_ref;
      linear_mpc_.calculateMovingMassesCommand(&moving_mass_ref);
      moving_masses_command->moving_mass_0_setpoint = (float)moving_mass_ref(0);
      moving_masses_command->moving_mass_1_setpoint = (float)moving_mass_ref(1);
      moving_masses_command->moving_mass_2_setpoint = (float)moving_mass_ref(2);
      moving_masses_command->moving_mass_3_setpoint = (float)moving_mass_ref(3);
      return true;
    }

    void MPCAttitudeControllerNode::run() {
      ros::Rate loop_rate(100); // 100 Hz -> Ts = 0.01 s
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