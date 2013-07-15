/*
Parker Conroy
Parcon Robotics

This code samples a JSON tag from a url
*/
#include <ros/ros.h>
#include <jansson.h>
/*
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Vector3.h>
#include <ardrone_autonomy/Navdata.h>
*/	

void merge_new_mgs(void){
	
	}

int main(int argc, char** argv)
{
	ros::init(argc, argv,"JSON_TAG");
	ros::NodeHandle node;
	ros::Rate loop_rate(50);
	ros::Publisher pub_twist;
	/*
	ros::Publisher pub_empty_reset;
	ros::Subscriber nav_sub;

	pub_twist = node.advertise<geometry_msgs::Twist>("cmd_vel", 1); 
	joy_sub = node.subscribe("joy", 1, joy_callback);
	*/
    ROS_INFO("Json tag Node");
 	while (ros::ok()) {
	merge_new_mgs();
	
		}//ros::ok
ROS_ERROR("ROS::ok failed- Node Closing");
}//main
