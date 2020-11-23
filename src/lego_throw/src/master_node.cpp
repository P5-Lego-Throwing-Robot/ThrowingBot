#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <lego_throw/pick_optionAction.h>
#include <lego_throw/throwingAction.h>
#include <actionlib/client/simple_action_client.h>
#include <lego_throw/camera.h>

struct box_id {
    std::string box_id;
    float x;
    float y;
    float z;

    std::string option;
};

// Global variables: 
nlohmann::json order;
std::vector<box_id> box_id_queue; 


// Callback function:
bool box_id_callback(lego_throw::camera::Request  &req,
                     lego_throw::camera::Response &res){
        for (int i =0 ; i<4; i++){
        box_id temp_box;

        temp_box.box_id = req.data;
        temp_box.x = req.x;
        temp_box.y = req.y;
        temp_box.z = req.z;

        temp_box.option = order[req.data][i];

        if(order.contains(temp_box.box_id))
        {
            std::cout << temp_box.box_id << " was added to the processing queue." << "\n";
            box_id_queue.push_back(temp_box);
        }
        
        else 
        {
            std::cout << temp_box.box_id <<" was not found in the database." << "\n";
        }
        }
        res.status = 0;
        return true;
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

    // Service server:
    ros::ServiceServer service = node_handle.advertiseService("camera", box_id_callback);

    // Action clients:
    actionlib::SimpleActionClient<lego_throw::pick_optionAction> picking_client("pick_option", true);
    actionlib::SimpleActionClient<lego_throw::throwingAction> throwing_client("throwing", true);

    // Waiting for action servers:
    picking_client.waitForServer();
    throwing_client.waitForServer();
    
    // Loading orders from file:
    load_json(ros::package::getPath("lego_throw") + ("/orders/orders.json"), order);

    // State varaible:
    int state = 0;

    // Action goals:
    lego_throw::pick_optionGoal pick_option_goal;
    lego_throw::throwingGoal throwing_goal;

    while(ros::ok()) 
    {
        if(box_id_queue.size() > 0)
        {

            // Setting new goals:
            if (state == 0) 
            {
                std::cout << "---------------------\nSTARTING TO PROCESS -> " << box_id_queue[0].box_id << std::endl;

                pick_option_goal.option = box_id_queue[0].option;
                throwing_goal.x = box_id_queue[0].x;
                throwing_goal.y = box_id_queue[0].y;
                throwing_goal.z = box_id_queue[0].z;

                std::cout << "Order: " << pick_option_goal.option << ", X: " << throwing_goal.x << " , Y: " << throwing_goal.y << " , Z: " << throwing_goal.z << std::endl;

                state = 1;
            }
            
            // Sending picking goal if one has not been set before:
            if (state == 1) 
            {
                ROS_INFO("SENDING PICKING GOAL!");
                picking_client.sendGoal(pick_option_goal);

                state = 2;
            }

            // Checking to see if set picking goal is completed:
            if (state == 2 && picking_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("PICKING DONE!");
                
                state = 3;
            }

            // If picking goal is complete a throwing goal is sent:
            if (state == 3) 
            {
                ROS_INFO("SENDING THROWING GOAL!");
                throwing_client.sendGoal(throwing_goal);
                
                state = 4;
            }

             // Checking to see if set picking goal is completed:
            if (state == 4 && throwing_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("THROWING DONE!");
                
                state = 5;
            }

            // If both picking and throwing is done, the order has been packed:
            if (state == 5)
            {
 
                // Done processing:
                std::cout << "DONE PROCESSING -> " << box_id_queue[0].box_id << "\n---------------------" << std::endl;

                // Erasing first element in box_id:
                box_id_queue.erase(box_id_queue.begin());

                // Resetting state:
                state = 0;
            }
        }
   
        //ROS_INFO("TEST");

        ros::spinOnce();
        ros::Rate(10).sleep();
    }
    

    return 0;
}
