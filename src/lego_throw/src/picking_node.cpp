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


    std::vector<double> red_bag_init{3.5562777519226074, -1.711090389882223, -1.3679917494403284, -1.632956329976217, 1.555716872215271, -1.1562846342669886};
    std::vector<double> red_bag{ 3.5576555728912354, -1.7895548979388636, -1.7646234671222132, -1.1579397360431116, 1.557623028755188, -1.1618183294879358};

    std::vector<double> green_bag_init{3.907362699508667, -1.9283598105060022,-1.0925691763507288,  -1.6913679281817835, 1.555501103401184, -0.8042801062213343};
    std::vector<double> green_bag{3.9084770679473877, -1.9780305067645472, -1.4986127058612269, -1.2353242079364222, 1.5568796396255493, -0.8099706808673304};

    std::vector<double> blue_bag_init{3.7789478302001953, -2.348231617604391, -0.42161256471742803, -1.9422147909747522, 1.5556808710098267, -0.9288480917560022};
    std::vector<double> blue_bag{3.7800257205963135, -2.2803214232074183, -1.016848389302389,  -1.4148395697223108, 1.5556808710098267, -0.9358471075641077};

    std::vector<double> yellow_bag_init{3.483186960220337, -2.0752437750445765, -0.8785517851458948,  -1.758641544972555, 1.5553812980651855, -1.2274039427386683};
    std::vector<double> yellow_bag{3.4843251705169678, -2.0998533407794397, -1.3224299589740198, -1.2898944059955042, 1.556448221206665, -1.2334168593036097};

    std::vector<double> via_point{3.1341092586517334, -1.7107909361468714, -1.3683159987079065, -1.633087460194723, 1.5557048320770264, -1.5785301367389124};
    
    //Moving to inital pick position before opening gripper
    ROS_INFO("MOVING TO INITIAL PICK POSITION");
    if (option == "1")
        group->setJointValueTarget(red_bag_init);
    if (option == "2")
        group->setJointValueTarget(green_bag_init);
    if (option == "3")
        group->setJointValueTarget(blue_bag_init);
    if (option == "4")
        group->setJointValueTarget(yellow_bag_init);
    group->move();

    ROS_INFO("OPENING GRIPPER");
    state.data = 1;
    button_state_pub.publish(state);

    //Depening on the bag option chosen the manipulator will choose the joint values to move to
    ROS_INFO("PICKING OPTION %s", option.c_str());
    if (option == "1")
        group->setJointValueTarget(red_bag);
    if (option == "2")
        group->setJointValueTarget(green_bag);
    if (option == "3")
        group->setJointValueTarget(blue_bag);
    if (option == "4")
        group->setJointValueTarget(yellow_bag);
    group->move();
    //When the manipulator has reached the bag, close the gripper
    state.data = 0;
    button_state_pub.publish(state);

    //Move back to initial pick position before initation throw
    ROS_INFO("MOVING BACK TO INITIAL PICK POSITION");
    if (option == "1")
        group->setJointValueTarget(red_bag_init);
    if (option == "2")
        group->setJointValueTarget(green_bag_init);
    if (option == "3")
        group->setJointValueTarget(blue_bag_init);
    if (option == "4")
        group->setJointValueTarget(yellow_bag_init);
    group->move();


    //Move to via point before throw:
    //group->setJointValueTarget(via_point);
    //group->move();

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
