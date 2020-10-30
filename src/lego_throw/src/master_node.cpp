#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <lego_throw/pick_option.h>

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

    // Service client request: 
    ros::ServiceClientOptions ops;
    ops.init ("pick_options",true);
    ops.allow_concurrent_callbacks = true;
    ros::ServiceClient = node_handle.ros::ServiceClient(ops);

    
    //ros::ServiceClient client = node_handle.serviceClient<lego_throw::pick_option>("pick_option");

    // Subscriber:
    ros::Subscriber box_info = node_handle.subscribe("box_info", 1, box_id_callback);
    
    // AsyncSpinner:
    ros::AsyncSpinner spinner(0);
    spinner.start();

    // Loading orders from file:
    load_json(ros::package::getPath("lego_throw") + ("/orders/orders.json"), order);

    bool processing = false;

    while(ros::ok()) 
    {
        if(box_id_queue.size() > 0 && processing == false)
        {
            processing = true;
            
            lego_throw::pick_option srv;
            srv.request.option = order[box_id_queue[0]][0];
            
            if (client.call(srv))
            {
                // Done processing:
                std::cout << "Done processing " << box_id_queue[0] << std::endl;
                box_id_queue.erase(box_id_queue.begin());
                processing = false;
            }
        }
   
        std::cout<<"PIK"<<std::endl;
        
        ros::spinOnce();
        ros::Rate(10).sleep();
    }
    

    return 0;
}