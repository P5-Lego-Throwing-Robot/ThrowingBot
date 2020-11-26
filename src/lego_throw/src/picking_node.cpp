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

    group->setMaxAccelerationScalingFactor(0.5);

    std_msgs::UInt16 state;


    std::vector<double> red_bag_init{3.5599920749664307, -1.7112701574908655, -1.423314396535055, -1.577430550252096, 1.555716872215271, -1.1531155745135706};
    std::vector<double> red_bag{3.5609023571014404, -1.7871363798724573, -1.7532079855548304, -1.1713998953448694, 1.5575990676879883, -1.1581695715533655};

    std::vector<double> green_bag_init{3.480203628540039, -2.070430103932516, -0.940134350453512, -1.7013548056231897, 1.5555490255355835, -1.2309325377093714};
    std::vector<double> green_bag{3.481137990951538, -2.1002610365497034, -1.307598892842428, -1.3039773146258753, 1.556448221206665, -1.2360461393939417};

    std::vector<double> blue_bag_init{3.7776176929473877, -2.341201130543844, -0.44892722765077764, -1.9218815008746546, 1.5555131435394287, -0.930084530507223};
    std::vector<double> blue_bag{3.7786123752593994, -2.2802379767047327, -1.0063703695880335, -1.4252694288836878, 1.5556329488754272, -0.9367716948138636};

    std::vector<double> yellow_bag_init{3.901876211166382, -1.927366081868307, -1.0972688833819788, -1.6873567740069788, 1.555489182472229, -0.8095262686358851};
    std::vector<double> yellow_bag{3.9030141830444336, -1.9740431944476526, -1.492054287587301, -1.2458742300616663, 1.5568077564239502, -0.8150365988360804};

    std::vector<double> via_point{3.295738458633423, -1.8790972868548792, -1.1627658049212855, -1.6700294653521937, 1.5557048320770264, -1.4159873167621058};
    
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
    group->setJointValueTarget(via_point);
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
