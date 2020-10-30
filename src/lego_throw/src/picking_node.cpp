#include <ros/ros.h>
#include <lego_throw/pick_option.h>
#include <iostream>

bool service_callback(lego_throw::pick_option::Request &req, lego_throw::pick_option::Response &res)
{
    std::cout << "Requesting the following item to be picked up: " << req.option << std::endl;

    ros::Duration(5.0).sleep();

    res.state = true;

    return true;
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "picking_node");
    ros::NodeHandle node_handle;

    ros::ServiceServer service = node_handle.advertiseService("pick_option", service_callback);

    ros::spin();
    
    return 0;
}