#include <ros/ros.h>
#include <lego_throw/pick_optionAction.h>
#include <actionlib/server/simple_action_server.h>
#include <iostream>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <math.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include "std_msgs/String.h"
#include "std_msgs/UInt16.h"
#include "geometry_msgs/Vector3.h"

#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

bool pick_bag(std::string option);
ros::Publisher button_state_pub;

void execute(const lego_throw::pick_optionGoalConstPtr &goal, actionlib::SimpleActionServer<lego_throw::pick_optionAction> *as)
{
    std::string option = goal->option;

    std::cout << "The following option was requested: " << option << std::endl;
    pick_bag(option);
    as->setSucceeded();
}

double cnvert_deg_to_rad(double degree) 
{ 
    double pi = 3.14159265359; 
    return (degree * (pi / 180)); 
} 

bool pick_bag(std::string option)
{
    // The robot:
    moveit::planning_interface::MoveGroupInterface group("manipulator");
    std_msgs::UInt16 state;
    double pi = 3.14159265359; 
    //Positions for the LEGO bags and intial pick position
    //std::vector<double> initial_pick_position{(203.84)*(pi/180), (-96.76)*(pi/180) , (-86.5)*(pi/180), (-48.3)*(pi/180), (88.65)*(pi/180), (-69.54) *(pi/180)};
    
    std::vector<double> red_bag_init{(203.84)*(pi/180), (-96.76)*(pi/180) , (-86.5)*(pi/180), (-48.3)*(pi/180), (88.65)*(pi/180), (-69.54) *(pi/180)};
    std::vector<double> green_bag_init{(201.7)*(pi/180), (-111.2) *(pi/180), (-73.6)*(pi/180), (-85.83) *(pi/180), (86.52)*(pi/180), (-71.42)*(pi/180)};
    //std::vector<double> blue_bag_init{(204.05)*(pi/180), (-102.96) *(pi/180), (-100.26)*(pi/180), (-67.16) *(pi/180), (88.65)*(pi/180), (-69.54)*(pi/180)};
    
    
    std::vector<double> red_bag{(204.05)*(pi/180), (-102.96) *(pi/180), (-100.26)*(pi/180), (-67.16) *(pi/180), (88.65)*(pi/180), (-69.54)*(pi/180)};
    std::vector<double> green_bag{(201.7)*(pi/180), (-114.05) *(pi/180), (-84.56)*(pi/180), (-71.55) *(pi/180), (86.59)*(pi/180), (-71.59)*(pi/180)};
    //std::vector<double> blue_bag{(204.05)*(pi/180), (-102.96) *(pi/180), (-100.26)*(pi/180), (-67.16) *(pi/180), (88.65)*(pi/180), (-69.54)*(pi/180)};

    //Moving to inital pick position before opening gripper
    ROS_INFO("MOVING TO INITIAL PICK POSITION");
    if (option == "R")
        group.setJointValueTarget(red_bag_init);
    else if (option == "G")
        group.setJointValueTarget(green_bag_init);
    else if (option == "B")
        group.setJointValueTarget(green_bag_init);
    group.move();

    ROS_INFO("OPENING GRIPPER");
    state.data = 1;
    button_state_pub.publish(state);

    //Depening on the bag option chosen the manipulator will choose the joint values to move to
    ROS_INFO("PICKING OPTION %s", option.c_str());
    if (option == "R")
        group.setJointValueTarget(red_bag);
    else if (option == "G")
        group.setJointValueTarget(green_bag);
    else if (option == "B")
        group.setJointValueTarget(green_bag);
    group.move();

    //When the manipulator has reached the bag, close the gripper
    state.data = 0;
    button_state_pub.publish(state);

    //Move back to initial pick position before initation throw
    ROS_INFO("MOVING BACK TO INITIAL PICK POSITION");
    if (option == "R")
        group.setJointValueTarget(red_bag_init);
    else if (option == "G")
        group.setJointValueTarget(green_bag_init);
    else if (option == "B")
        group.setJointValueTarget(green_bag_init);
    group.move();

    ROS_INFO("READY TO THROW");

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "picking_node");
    ros::NodeHandle node_handle;
    button_state_pub = node_handle.advertise<std_msgs::UInt16>("gripper_state", 100);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    //add_object();
    // Action server:
    actionlib::SimpleActionServer<lego_throw::pick_optionAction> server(node_handle, "pick_option", boost::bind(&execute, _1, &server), false);
    server.start();

    ros::waitForShutdown();

    return 0;
}

/*void add_object()
{
    const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
   
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
   
    Eigen::Vector3d scaler(0.01, 0.01, 0.01);
   

    moveit_msgs::CollisionObject collision_object;
    collision_object.id = "wall";
    shapes::Mesh *m = shapes::createMeshFromResource("file:///home/egil/Downloads/Bulbasaur.stl", scaler);
    ROS_INFO("STL loaded");

    shape_msgs::Mesh mesh;
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m, mesh_msg);
    mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

    collision_object.meshes.resize(1);
    collision_object.mesh_poses.resize(1);
    collision_object.meshes[0] = mesh;
    collision_object.header.frame_id = move_group.getPlanningFrame();
    collision_object.mesh_poses[0].position.x = 3.0;
    collision_object.mesh_poses[0].position.y = 0.0;
    collision_object.mesh_poses[0].position.z = 0.0;
    collision_object.mesh_poses[0].orientation.w=  0.0;
    collision_object.mesh_poses[0].orientation.x = 0.0;
    collision_object.mesh_poses[0].orientation.y = 0.0;
    collision_object.mesh_poses[0].orientation.z = -3.14;

    collision_object.meshes.push_back(mesh);
    collision_object.mesh_poses.push_back(collision_object.mesh_poses[0]);
    collision_object.operation = collision_object.ADD;
    std::vector<moveit_msgs::CollisionObject> collision_vector;
    collision_vector.push_back(collision_object);

    planning_scene_interface.applyCollisionObjects(collision_vector);
    ROS_INFO("STL added into the world");
    //move_group.attachObject(collision_object.id);
    sleep(5.0);
}
*/