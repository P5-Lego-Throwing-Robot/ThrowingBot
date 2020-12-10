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
#include <moveit_msgs/ExecuteTrajectoryAction.h>

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
int trajectory_status = 3;


// Callback function:
bool box_id_callback(lego_throw::camera::Request  &req,
                     lego_throw::camera::Response &res){

        if(order.contains(req.data))
        {

            // Creating box:
            box_id temp_box;

            // Box center coordinates:
            float x_c = req.x;
            float y_c = req.y;
            float z_c = req.z;

            for (int i = 0; i < order[req.data]["bags"]; i++) 
            {
                
                // Corner offset:
                float x_offset = float(order[req.data]["item"][i]["OffsetX"]);
                float y_offset = float(order[req.data]["item"][i]["OffsetY"]);
                float z_offset = float(order[req.data]["item"][i]["OffsetZ"]);


                // Box id:
                temp_box.box_id = req.data + ", bag: " + std::to_string(i + 1);

                // Throw coordinate:
                temp_box.x = x_c + x_offset;
                temp_box.y = y_c + y_offset;
                temp_box.z = z_c + z_offset;

                // Throw option:
                temp_box.option = order[req.data]["item"][i]["option"];

                box_id_queue.push_back(temp_box);
            }

            std::cout << req.data << " ("<< order[req.data]["name"] << ", "  << order[req.data]["bags"] << " bags, XYZ: (" << x_c << ", " << y_c << ", " << z_c << ")) was added to the processing queue." << "\n";
        }
        else 
        {
            std::cout << req.data <<" was not found in the database." << "\n";
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

// Update trajectory status to avoid overlapping trajectories:
void execute_trajectory_callback(const moveit_msgs::ExecuteTrajectoryActionFeedbackConstPtr &msg)
{
    actionlib_msgs::GoalStatus feedback = msg->status;
    trajectory_status = int(feedback.status);
    //std::cout << "New trajectory status: " << trajectory_status << std::endl;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "master_node");
    ros::NodeHandle node_handle;

    // Service server:
    ros::ServiceServer service = node_handle.advertiseService("camera", box_id_callback);

    // Subscribe to feedback from trajectory executioner to avoid overlapping trajectories:
    ros::Subscriber execute_sub = node_handle.subscribe("/execute_trajectory/feedback", 1, execute_trajectory_callback);

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

    // Loop rate:
    ros::Rate loop_rate(10);

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
            
            // Sending picking goal if one has not been set before and the throwing node is finished (trajectory_status == 3):
            if (state == 1 && trajectory_status == 3) 
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
   
        ros::spinOnce();
        loop_rate.sleep();
    }
    

    return 0;
}
