#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Vector3>("box_position", 1000);

  ros::Rate loop_rate(1);

  while (ros::ok()) {
    geometry_msgs::Vector3 msg;

    msg.x = 1.1;
    msg.y = 1.2;
    msg.z = 1.3;

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}