#include <ros/ros.h>
#include <lego_throw/pick_option.h>
#include <lego_throw/pick_optionAction.h>
#include <actionlib/server/simple_action_server.h>
#include <iostream>

bool service_callback(lego_throw::pick_option::Request &req, lego_throw::pick_option::Response &res)
{
    std::cout << "Requesting the following item to be picked up: " << req.option << std::endl;

    ros::Duration(5.0).sleep();

    res.state = true;

    return true;
}

void execute(const lego_throw::pick_optionGoalConstPtr& goal, actionlib::SimpleActionServer<lego_throw::pick_optionAction>* as) 
{
    std::string option = goal->option;

    std::cout << "The following option was requested: " << option << std::endl;

    ros::Duration(5.0).sleep();

    as->setSucceeded();
}



int main(int argc, char** argv)
{

    ros::init(argc, argv, "picking_node");
    ros::NodeHandle node_handle;

    //ros::ServiceServer service = node_handle.advertiseService("pick_option", service_callback);

    actionlib::SimpleActionServer<lego_throw::pick_optionAction> server(node_handle, "pick_option", boost::bind(&execute, _1, &server), false);
    server.start();

    ros::spin();
    
    return 0;
}