#include <ros/ros.h>
#include <ros/package.h>
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
#include <tf2/LinearMath/Quaternion.h>

#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
// The robot:
moveit::planning_interface::MoveGroupInterface *group;


void execute(const lego_throw::pick_optionGoalConstPtr &goal, actionlib::SimpleActionServer<lego_throw::pick_optionAction> *as);
bool pick_bag(std::string option);
void add_object();
ros::Publisher button_state_pub;



int main(int argc, char **argv)
{
    ros::init(argc, argv, "picking_node");
    ros::NodeHandle node_handle;
    button_state_pub = node_handle.advertise<std_msgs::UInt16>("gripper_state", 100);
        // The robot:
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    group = &move_group;

    ros::AsyncSpinner spinner(0);
    spinner.start();

    add_object();
    // Action server:
    actionlib::SimpleActionServer<lego_throw::pick_optionAction> server(node_handle, "pick_option", boost::bind(&execute, _1, &server), false);
    server.start();

    ros::waitForShutdown();

    return 0;
}



void execute(const lego_throw::pick_optionGoalConstPtr &goal, actionlib::SimpleActionServer<lego_throw::pick_optionAction> *as)
{
    std::string option = goal->option;

    std::cout << "The following option was requested: " << option << std::endl;
    pick_bag(option);
    as->setSucceeded();
}


bool pick_bag(std::string option)
{

    std_msgs::UInt16 state;


    std::vector<double> red_bag_init{3.544858932495117, -1.6782949606524866, -1.5043681303607386, -1.4927943388568323, 1.6183748245239258, -1.1857765356646937};
    std::vector<double> red_bag{3.5458896160125732, -1.747371021901266, -1.7807148138629358, -1.1476643721209925, 1.62007737159729, -1.1898101011859339};

    //Moving to inital pick position before opening gripper
    ROS_INFO("MOVING TO INITIAL PICK POSITION");
    if (option == "R")
        group->setJointValueTarget(red_bag_init);
    group->move();

    ROS_INFO("OPENING GRIPPER");
    state.data = 1;
    button_state_pub.publish(state);

    //Depening on the bag option chosen the manipulator will choose the joint values to move to
    ROS_INFO("PICKING OPTION %s", option.c_str());
    if (option == "R")
        group->setJointValueTarget(red_bag);
    group->move();
    //When the manipulator has reached the bag, close the gripper
    state.data = 0;
    button_state_pub.publish(state);

    //Move back to initial pick position before initation throw
    ROS_INFO("MOVING BACK TO INITIAL PICK POSITION");
    if (option == "R")
        group->setJointValueTarget(red_bag_init);
    group->move();
    ROS_INFO("READY TO THROW");

    return true;
}


void add_object()
{
    const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
   
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
   
    Eigen::Vector3d scaler(0.001, 0.001, 0.001);
   

    moveit_msgs::CollisionObject collision_object;
    collision_object.id = "wall";
    shapes::Mesh *m = shapes::createMeshFromResource("file://"+ros::package::getPath("lego_throw")+"/mesh/worktable.stl", scaler);
    ROS_INFO("STL loaded");

    shape_msgs::Mesh mesh;
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m, mesh_msg);
    mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

    collision_object.meshes.resize(1);
    collision_object.mesh_poses.resize(1);
    collision_object.meshes[0] = mesh;
    collision_object.header.frame_id = move_group.getPlanningFrame();
    collision_object.mesh_poses[0].position.x = 0.0;
    collision_object.mesh_poses[0].position.y = 0.0;
    collision_object.mesh_poses[0].position.z = 0.0;


    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0, 0, 1.58);
    collision_object.mesh_poses[0].orientation.w=  myQuaternion.getW();
    collision_object.mesh_poses[0].orientation.x = myQuaternion.getX();
    collision_object.mesh_poses[0].orientation.y = myQuaternion.getY();
    collision_object.mesh_poses[0].orientation.z = myQuaternion.getZ();


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
