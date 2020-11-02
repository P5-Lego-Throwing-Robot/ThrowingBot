#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <lego_throw/pick_option.h>
#include <lego_throw/pick_optionAction.h>
#include <actionlib/client/simple_action_client.h>

// Global variables: 
nlohmann::json order;
std::vector<std::string> box_id_queue; 

// Callback function:
void box_id_callback(const std_msgs::String::ConstPtr& msg){
        
        std::string ID = msg->data;

        if(order.contains(ID)){
            std::cout << ID << " was added to the processing queue." << "\n";
            box_id_queue.push_back(ID);
        }
        else {
            std::cout << ID <<" was not found in the database." << "\n";
        }
}


bool load_json(std::string path, nlohmann::json &json_object)
    {
        // Load file:
        std::fstream file(path);

        // Check if file loaded successfully:
        if(file.fail()) 
        {
            std::cout << "Failed to open file (" << path << ")";
            return false;
        }

        // Parsing file to json object:
        file >> order;
        
        return true;
    }


int main(int argc, char **argv)
{
    ros::init(argc, argv, "master_node");
    ros::NodeHandle node_handle;

    // Picking action client:
    actionlib::SimpleActionClient<lego_throw::pick_optionAction> picking_client("pick_option", true);
    picking_client.waitForServer();

    // Subscriber:
    ros::Subscriber box_info = node_handle.subscribe("box_info", 1, box_id_callback);
    
    // Loading orders from file:
    load_json(ros::package::getPath("lego_throw") + ("/orders/orders.json"), order);

    // Variables for controlling the flow of the code:
    bool processing = false;
    bool picking_goal_sent = false;
    bool throwing_goal_sent = false;
    bool picking_done = false;
    bool throwing_done = false;
    lego_throw::pick_optionGoal pick_option_goal;

    while(ros::ok()) 
    {
        if(box_id_queue.size() > 0)
        {

            // Setting new goals:
            if (processing == false) 
            {
                processing = true;

                pick_option_goal.option = order[box_id_queue[0]][0];
            }
            
            // Sending picking goal if one has not been set before:
            if (picking_goal_sent == false) 
            {
                picking_client.sendGoal(pick_option_goal);
                picking_goal_sent = true;
            }

            // Checking to see if set picking goal is completed:
            if (picking_goal_sent == true && picking_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                picking_done = true;

                // DELETE:
                // Done processing:
                std::cout << "Done processing " << box_id_queue[0] << std::endl;
                box_id_queue.erase(box_id_queue.begin());
                processing = false;
                picking_goal_sent = false;
                throwing_goal_sent = false;
                picking_done = false;
                throwing_done = false;
            }

            /*
            // If picking goal is complete a throwing goal is sent:
            if (picking_done == true && throwing_goal_sent == false) 
            {
                throwing_goal_sent == true;
            }

            
             // Checking to see if set picking goal is completed:
            if (throwing_goal_sent == true && throwing_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                picking_done = true;
            }

            if (picking_done == true && throwing_done == true)
            {
                // Done processing:
                std::cout << "Done processing " << box_id_queue[0] << std::endl;
                box_id_queue.erase(box_id_queue.begin());
                processing = false;
                picking_goal_sent = false;
                throwing_goal_sent = false;
                picking_done = false;
                throwing_done = false;
            }
            */
        }
   
        ROS_INFO("TEST");
        
        ros::spinOnce();
        ros::Rate(10).sleep();
    }
    

    return 0;
}
