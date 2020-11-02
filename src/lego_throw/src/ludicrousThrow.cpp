#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <math.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include "std_msgs/String.h"
#include "std_msgs/UInt16.h"
#include "geometry_msgs/Vector3.h"
#include "ros/callback_queue.h"
#include <actionlib/server/simple_action_server.h>
#include <lego_throw/throwingAction.h>

// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

double goal_position[3] = {0.1, 1.3, 0.05}; // goal position in meters from base frame
double throwing_angle = (M_PI*3.0)/16; // The angle to throw in radians
double rotation_velocity = 2.0; // radians/sec rotation of object when throwing velocity is 1.0
double acceleration_time = 1.5; // Time it takes to accelerate from joint_position_start to joint_position_throw when throwing velocity = 1.0
double deceleration_time = 4.0; // Time it takes to decelerate from joint_position_throw to joint_position_end when throwing velocity = 1.0
int acceleration_waypoints = 100; // Number of waypoints in the acceleration phase
int deceleration_waypoints = 100; // Number of waypoints in the deceleration phase
double return_to_start_acceleration_scale = 0.1;

// Start, throw and end positions in joint space. radians
std::vector<double> joint_position_start{0.0, -(M_PI*13)/16, -(M_PI*6)/16, -(M_PI*2)/16, M_PI/2, 0.0};
std::vector<double> joint_position_throw{0.0, -(M_PI*10)/16, -(M_PI*3)/16, (M_PI*1)/16, M_PI/2, 0.0};
std::vector<double> joint_position_end{0.0, -(M_PI*10)/16, -(M_PI*3)/16, (M_PI*1)/16, M_PI/2, 0.0};

//double offset = 0.109492; // end-effector offset from x-axis when J0 = 0. used in get_joint_one_angle()
double offset = 0.1053; // end-effector offset from x-axis when J0 = 0. used in get_joint_one_angle()
double g = -9.816;
std::vector<double> zero_velocity_vector{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
const std::string PLANNING_GROUP = "manipulator";
int serial_port = 0;


// Finds the angle of joint one so that the goal position intersects with the throwing plane
double get_joint_one_angle(double x, double y) {
    double distToPoint = sqrt(pow(x, 2) + pow(y, 2));
    double theta1 = asin(offset/distToPoint);
    double theta2 = atan2(y, x);
    return theta2 - theta1;
}


// Perform forward kinematics given some joint values
Eigen::Isometry3d forward_kinematics(std::vector<double> joint_values) {
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(PLANNING_GROUP);
    kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
    const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("tcp");
    return end_effector_state;
}


// Calculates the throwing velocity
double get_throwing_velocity(double height, double length, double theta) {
    return -(sqrt(2) * length * sqrt(g * cos(theta) * (height * cos(theta) - length * sin(theta)))) / (2 * (height * pow(cos(theta), 2) - length * cos(theta) * sin(theta)));
}


// Convert a velocity scalar into a velocity vector given some angle
std::vector<double> vectorize_throwing_velocity(double velocity, double angle) {
    std::vector<double> velocity_vector {
        cos(angle)*velocity,
        0.0,
        sin(angle)*velocity
    };
    return velocity_vector;
}


// Using inverse jacobian to calculate joint velocities
std::vector<double> get_joint_velocities(std::vector<double> end_effector_velocities, std::vector<double> joint_angles) {
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    moveit::core::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(PLANNING_GROUP);
    kinematic_state->setJointGroupPositions(joint_model_group, joint_angles);
    Eigen::MatrixXd jacobian;
    jacobian = kinematic_state->getJacobian(joint_model_group);
    Eigen::MatrixXd inverse_jacobian(6, 6);
    inverse_jacobian = jacobian.inverse();
    Eigen::MatrixXd end_effector_velocities_matrix(6, 1);
    end_effector_velocities_matrix << end_effector_velocities[0], end_effector_velocities[1], end_effector_velocities[2], end_effector_velocities[3], end_effector_velocities[4], end_effector_velocities[5];
    Eigen::MatrixXd joint_velocities_matrix = inverse_jacobian*end_effector_velocities_matrix;
    std::vector<double> joint_velocities;
    joint_velocities.push_back(joint_velocities_matrix(0, 0));
    joint_velocities.push_back(joint_velocities_matrix(1, 0));
    joint_velocities.push_back(joint_velocities_matrix(2, 0));
    joint_velocities.push_back(joint_velocities_matrix(3, 0));
    joint_velocities.push_back(joint_velocities_matrix(4, 0));
    joint_velocities.push_back(joint_velocities_matrix(5, 0));
    return joint_velocities;
}


// Adds to a trajectory object using third degree polynomials
moveit_msgs::RobotTrajectory add_to_a_trajectory(moveit_msgs::RobotTrajectory trajectory, std::vector<double> theta_start, std::vector<double> theta_end, std::vector<double> theta_dot_start, std::vector<double> theta_dot_end, double travel_time, int steps, bool is_first_step, double start_time) {
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    //move_group.setEndEffectorLink("tcp");
    std::vector<std::string> jointNames = move_group.getJointNames();

    trajectory.joint_trajectory.joint_names = jointNames;

    double a0j1 = theta_start[0];
    double a0j2 = theta_start[1];
    double a0j3 = theta_start[2];
    double a0j4 = theta_start[3];
    double a0j5 = theta_start[4];
    double a0j6 = theta_start[5];

    double a1j1 = theta_dot_start[0];
    double a1j2 = theta_dot_start[1];
    double a1j3 = theta_dot_start[2];
    double a1j4 = theta_dot_start[3];
    double a1j5 = theta_dot_start[4];
    double a1j6 = theta_dot_start[5];

    double a2j1 = (3/pow(travel_time, 2)) * (theta_end[0] - theta_start[0]) - (2/travel_time) * theta_dot_start[0] - (1/travel_time) * theta_dot_end[0];
    double a2j2 = (3/pow(travel_time, 2)) * (theta_end[1] - theta_start[1]) - (2/travel_time) * theta_dot_start[1] - (1/travel_time) * theta_dot_end[1];
    double a2j3 = (3/pow(travel_time, 2)) * (theta_end[2] - theta_start[2]) - (2/travel_time) * theta_dot_start[2] - (1/travel_time) * theta_dot_end[2];
    double a2j4 = (3/pow(travel_time, 2)) * (theta_end[3] - theta_start[3]) - (2/travel_time) * theta_dot_start[3] - (1/travel_time) * theta_dot_end[3];
    double a2j5 = (3/pow(travel_time, 2)) * (theta_end[4] - theta_start[4]) - (2/travel_time) * theta_dot_start[4] - (1/travel_time) * theta_dot_end[4];
    double a2j6 = (3/pow(travel_time, 2)) * (theta_end[5] - theta_start[5]) - (2/travel_time) * theta_dot_start[5] - (1/travel_time) * theta_dot_end[5];

    double a3j1 = -(2/pow(travel_time, 3)) * (theta_end[0] - theta_start[0]) + (1/pow(travel_time, 2)) * (theta_dot_end[0] + theta_dot_start[0]);
    double a3j2 = -(2/pow(travel_time, 3)) * (theta_end[1] - theta_start[1]) + (1/pow(travel_time, 2)) * (theta_dot_end[1] + theta_dot_start[1]);
    double a3j3 = -(2/pow(travel_time, 3)) * (theta_end[2] - theta_start[2]) + (1/pow(travel_time, 2)) * (theta_dot_end[2] + theta_dot_start[2]);
    double a3j4 = -(2/pow(travel_time, 3)) * (theta_end[3] - theta_start[3]) + (1/pow(travel_time, 2)) * (theta_dot_end[3] + theta_dot_start[3]);
    double a3j5 = -(2/pow(travel_time, 3)) * (theta_end[4] - theta_start[4]) + (1/pow(travel_time, 2)) * (theta_dot_end[4] + theta_dot_start[4]);
    double a3j6 = -(2/pow(travel_time, 3)) * (theta_end[5] - theta_start[5]) + (1/pow(travel_time, 2)) * (theta_dot_end[5] + theta_dot_start[5]);

    int k = 0;
    if (!is_first_step) {
        k = 1;
    }

    for (int i = k; i <= steps; i++) {
        trajectory_msgs::JointTrajectoryPoint point;

        double time = (travel_time/steps)*i;

        std::vector<double> positions;
        positions.push_back(a0j1 + a1j1*time + a2j1*pow(time, 2) + a3j1*pow(time, 3));
        positions.push_back(a0j2 + a1j2*time + a2j2*pow(time, 2) + a3j2*pow(time, 3));
        positions.push_back(a0j3 + a1j3*time + a2j3*pow(time, 2) + a3j3*pow(time, 3));
        positions.push_back(a0j4 + a1j4*time + a2j4*pow(time, 2) + a3j4*pow(time, 3));
        positions.push_back(a0j5 + a1j5*time + a2j5*pow(time, 2) + a3j5*pow(time, 3));
        positions.push_back(a0j6 + a1j6*time + a2j6*pow(time, 2) + a3j6*pow(time, 3));

        std::vector<double> velocities;
        velocities.push_back(a1j1 + 2*a2j1*time + 3*a3j1*pow(time, 2));
        velocities.push_back(a1j2 + 2*a2j2*time + 3*a3j2*pow(time, 2));
        velocities.push_back(a1j3 + 2*a2j3*time + 3*a3j3*pow(time, 2));
        velocities.push_back(a1j4 + 2*a2j4*time + 3*a3j4*pow(time, 2));
        velocities.push_back(a1j5 + 2*a2j5*time + 3*a3j5*pow(time, 2));
        velocities.push_back(a1j6 + 2*a2j6*time + 3*a3j6*pow(time, 2));

        std::vector<double> accelerations;
        accelerations.push_back(2*a2j1 + 6*a3j1*time);
        accelerations.push_back(2*a2j2 + 6*a3j2*time);
        accelerations.push_back(2*a2j3 + 6*a3j3*time);
        accelerations.push_back(2*a2j4 + 6*a3j4*time);
        accelerations.push_back(2*a2j5 + 6*a3j5*time);
        accelerations.push_back(2*a2j6 + 6*a3j6*time);
        
        point.positions = positions;
        point.velocities = velocities;
        point.accelerations = accelerations;

        ros::Duration seconds(time + start_time);
        point.time_from_start = seconds;

        trajectory.joint_trajectory.points.push_back(point);
    }

    return trajectory;
}


// Scales a trajectory to change the throwing velocity from 1.0 to a given value
moveit_msgs::RobotTrajectory scale_trajectory(moveit_msgs::RobotTrajectory trajectory, double velocity, double degrees) {
    for (int i = 0; i < trajectory.joint_trajectory.points.size(); i++) {
        for (int j = 0; j < 6; j++) {
            trajectory.joint_trajectory.points[i].velocities[j] = trajectory.joint_trajectory.points[i].velocities[j] * velocity;
            trajectory.joint_trajectory.points[i].accelerations[j] = trajectory.joint_trajectory.points[i].accelerations[j] * velocity;
        }
        ros::Duration seconds(trajectory.joint_trajectory.points[i].time_from_start.toSec() / velocity);
        trajectory.joint_trajectory.points[i].time_from_start = seconds;
        trajectory.joint_trajectory.points[i].positions[0] = degrees;
    }

    return trajectory;
}


// Go to some joint position
void go_to_joint_position(std::vector<double> joint_goal) {
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    //move_group.setEndEffectorLink("tcp");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    move_group.setMaxAccelerationScalingFactor(return_to_start_acceleration_scale);
    move_group.setJointValueTarget(joint_goal);
    move_group.setNumPlanningAttempts(10);
    move_group.plan(plan);
    move_group.execute(plan);
}


void configure_serial_port() {
    // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
    serial_port = open("/dev/ttyACM0", O_RDWR);

    // Create new termios struc, we call it 'tty' for convention
    struct termios tty;

    // Read in existing settings, and handle any error
    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 115200
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }

    ros::Duration(3.0).sleep();
}


// Change the state of the gripper
void set_gripper_state(bool open) {
    if (open) {
        write(serial_port, "1", 2);
    } else {
        write(serial_port, "0", 2);
    }
    /*
    std_msgs::UInt16 button_msg;
    if (open) {
        button_msg.data = 1;
    } else {
        button_msg.data = 0;
    }
    gripper_pub.publish(button_msg);
    */
}


// Takes a cartesian positian as input and tries to throw to that position.
void throw_to(double position[3]) {
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    //move_group.setEndEffectorLink("tcp");

    double joint_one_angle = get_joint_one_angle(goal_position[0], goal_position[1]); // Calculating the angle of joint one in radians

    std::vector<double> turned_joint_position_throw = joint_position_throw;
    turned_joint_position_throw[0] = joint_one_angle;
    Eigen::Isometry3d cartesian_throw_pose = forward_kinematics(turned_joint_position_throw); // Finding the cartesian position of the throwing position

    double height = goal_position[2] - cartesian_throw_pose.translation().z(); // Calculate the change in height from throwing position to goal position
    double length = sqrt(pow((goal_position[0]-cartesian_throw_pose.translation().x()), 2) + pow((goal_position[1]-cartesian_throw_pose.translation().y()), 2)); // Calculate the euclidean distance from the throwing position to goal position in the x, y-plane
    double velocity = get_throwing_velocity(height, length, throwing_angle); // Calculate the initial throwing velocity

    if (velocity > 4.9) {
        throw "too fast";
    }

    std::vector<double> throwing_velocity_vector = vectorize_throwing_velocity(1.0, throwing_angle); // Calculating a velocity vector with a speed of 1.0
    throwing_velocity_vector.push_back(0.0);
    throwing_velocity_vector.push_back(rotation_velocity);
    throwing_velocity_vector.push_back(0.0);

    std::vector<double> joint_velocities_vector = get_joint_velocities(throwing_velocity_vector, joint_position_throw); // Calculating joint velocities

    // Instantiating a trajectory and adding an acceleration and deceleration phase to it
    moveit_msgs::RobotTrajectory trajectory;
    trajectory = add_to_a_trajectory(trajectory, joint_position_start, joint_position_throw, zero_velocity_vector, joint_velocities_vector, acceleration_time, acceleration_waypoints, true, 0.0);
    trajectory = add_to_a_trajectory(trajectory, joint_position_throw, joint_position_end, joint_velocities_vector, zero_velocity_vector, deceleration_time, deceleration_waypoints, false, acceleration_time);

    trajectory = scale_trajectory(trajectory, velocity, joint_one_angle); // Scale the trajectory to the right velocity

    // Go to the start position
    std::vector<double> turned_joint_position_start = joint_position_start;
    turned_joint_position_start[0] = joint_one_angle;
    go_to_joint_position(turned_joint_position_start);

    // throw the object 
    move_group.asyncExecute(trajectory);

    //ros::Duration((acceleration_time / velocity) * 1.08).sleep();
    ros::Duration((acceleration_time / velocity) - 0.02).sleep();

    set_gripper_state(true);

    ROS_INFO("time: %f", (acceleration_time / velocity));
    ROS_INFO("velocity: %f", velocity);

    std::vector<double> joint_values = move_group.getCurrentJointValues();

    ros::spinOnce();
}

void goal_position_callback(const geometry_msgs::Vector3::ConstPtr& msg) {
    goal_position[0] = msg->x;
    goal_position[1] = msg->y;
    goal_position[2] = msg->z;

    ROS_INFO("pos x: %f", goal_position[0]);
    ROS_INFO("pos y: %f", goal_position[1]);
    ROS_INFO("pos z: %f", goal_position[2]);

    try {
        throw_to(goal_position);
        
        ros::Duration(3.0).sleep();
        
        set_gripper_state(false);
    } catch(std::string error) {
        ROS_ERROR("error: %s", error.c_str());
    }
}


void execute_throw(const lego_throw::throwingGoalConstPtr& goal, actionlib::SimpleActionServer<lego_throw::throwingAction>* action_server) {
  // Do lots of awesome groundbreaking robot stuff here

    goal_position[0] = goal->x;
    goal_position[1] = goal->y;
    goal_position[2] = goal->z;

    ROS_INFO("pos x: %f", goal_position[0]);
    ROS_INFO("pos y: %f", goal_position[1]);
    ROS_INFO("pos z: %f", goal_position[2]);

    try {
        throw_to(goal_position);
        
        ros::Duration(3.0).sleep();
        
        set_gripper_state(false);
    } catch(std::string error) {
        ROS_ERROR("error: %s", error.c_str());
    }
  ROS_INFO("something happend. SUCCESS!!!!!");
  action_server->setSucceeded();


}


int main(int argc, char** argv) {
    ros::init(argc, argv, "ludicrous_throw");
    ros::AsyncSpinner spinner(4);

    ros::NodeHandle node_handle;

    actionlib::SimpleActionServer<lego_throw::throwingAction> server(node_handle, "throwing", boost::bind(&execute_throw, _1, &server), false);
    server.start();

    ros::Subscriber box_position_subscriber = node_handle.subscribe("box_position", 1, goal_position_callback);
    spinner.start();

    configure_serial_port();

/*
    try {
        throw_to(goal_position);
        
        ros::Duration(3.0).sleep();
        
        set_gripper_state(false);
    } catch(std::string error) {
        ROS_ERROR("error: %s", error.c_str());
    }
    */

    ros::waitForShutdown();

    close(serial_port);

    return 0;
}