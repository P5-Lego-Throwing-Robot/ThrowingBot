
#include <ros/ros.h>
#include <ros/package.h>
#include <nlohmann/json.hpp>
#include <fstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "master_node");
    std::string order_path = ros::package::getPath("lego_throw")+ ("/orders/orders.json");
    std::fstream file(order_path);

    if(file.fail()) 
    {
        std::cout << "Failed to open file (" << order_path << ")";
        return 0;
    }
    nlohmann::json orders;
    file >> orders;

    std::cout<< orders.dump(); 


    return 0;
}