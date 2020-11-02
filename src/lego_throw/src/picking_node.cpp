#include <ros/ros.h>
#include <lego_throw/pick_optionAction.h>
#include <actionlib/server/simple_action_server.h>
#include <iostream>

void execute(const lego_throw::pick_optionGoalConstPtr& goal, actionlib::SimpleActionServer<lego_throw::pick_optionAction>* as) 
{
    std::string option = goal->option;

    std::cout << "The following option was requested: " << option << std::endl;

    as->setSucceeded();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "picking_node");
    ros::NodeHandle node_handle;

    // Action server:
    actionlib::SimpleActionServer<lego_throw::pick_optionAction> server(node_handle, "pick_option", boost::bind(&execute, _1, &server), false);
    server.start();

    ros::spin();
    
    return 0;
}
