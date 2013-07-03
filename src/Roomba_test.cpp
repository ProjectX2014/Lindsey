/*
Parker Conroy

    
    This node runs the turtle bot for the extended LQR experiments.
*/

#include <ros/ros.h>				// ROS resources
#include "tf/transform_listener.h"		// coordinate system transforms
#include <geometry_msgs/Vector3.h>		// defines Vector3
#include "geometry_msgs/TransformStamped.h"	// transform with time stamp
#include "ros/time.h"				// headers to manipulate time
#include <std_msgs/Int16.h>			// 16bit int
//#include <Eigen/Dense>			// linear algebra library
#include <sensor_msgs/Joy.h>			// joy stick message
#include "Tank.h"				// tank services is how to command Rumba
#include <std_msgs/Float32.h>			// float 32
#include <fstream>
#include <string>
#include <sstream>

/*
#define yaw 0
#define pitch 1
#define roll 2
#define thrust 3

#define u_dim 2
#define x_dim 3
*/

/*
// LQR stuff
using namespace Eigen;

// Flag to enable/disable mocap 
//bool tf_flag = false;
bool tf_flag = true;

// Flag to enable/disable debugging
//bool debug_matrix = false;
bool debug_matrix = true;

// Flag to start running controller
bool start_flag = false;

// LQR matrices
std::vector<MatrixXf> Lt(60);
std::vector<VectorXf> ellt(60);

// State vector
VectorXf x(x_dim);

// Control input
Vector2f u;
*/


// Mo-Cap position & yaw
geometry_msgs::Vector3 curr_pos;
std_msgs::Float32 yaw_angle;
geometry_msgs::Vector3 u_roomba;


// Counter variable
int t = 0;

// Joystick stuff for test drives
double joy_x_,joy_y_;			// raw data
double joy_x, joy_y;			// stable data
int joy_a_,joy_b_;			// raw a and b buttons 
int joy_a, joy_b;
geometry_msgs::Vector3  output;

// Joystick callback
void joy_callback(const sensor_msgs::Joy& joy_msg_in)
{
	joy_x_ = joy_msg_in.axes[1];
	joy_y_ = joy_msg_in.axes[0];
	joy_a_ = joy_msg_in.buttons[0];
	joy_b_ = joy_msg_in.buttons[1];
}

// Merge new joystick messages
void merge_new_msgs(void)
{
	joy_x = joy_x_;
	joy_y = joy_y_;
	joy_a = joy_a_;
	joy_b = joy_b_;
}

// Simple tank mixer
geometry_msgs::Vector3 mixer(double joy_x, double joy_y)
{
	double max = 500.0;
	double min = -500.0;
	double left, right;
	double joy_dead = 0.1;
	if (fabs(joy_x)<joy_dead) {joy_x =0;}
	if (fabs(joy_y)<joy_dead) {joy_y =0;}
	left = 500.0*joy_x - 500.0*joy_y;		// differential drive mixer
	right = 500.0*joy_x + 500.0*joy_y;
	if (left > max) {left = max;}
	if (left < min) {left = min;}
	if (right > max) {right = max;}
	if (right < min) {right = min;}
	geometry_msgs::Vector3 out;
	out.x = (int) left;
	out.y = (int) right;
	out.z = 0;
	return out;
}

// Main loop
int main(int argc, char** argv)
{
	// ROS Initialization
	ros::init(argc, argv, "Roomba_Driver");		// creates ROS node
	ros::NodeHandle node;				// creates the node handle
	ros::Rate loop_rate(100); 			// testing rate (Hz)
	
	// ROS subscribers
	ros::Subscriber joy_sub;			// create subscriber to listen to joystick
	joy_sub = node.subscribe("joy",1,joy_callback); // this tells our node what to subscribe to, 1 is buffer size of 1

	// ROS publishers
/*	
	ros::Publisher pos_pub = node.advertise<geometry_msgs::Vector3>("position",1);
	ros::Publisher yaw_pub = node.advertise<std_msgs::Float32>("yaw",1);
	ros::Publisher u_pub = node.advertise<geometry_msgs::Vector3>("control_input",1);
*/
	// ROS Service client
	ros::ServiceClient client = node.serviceClient<irobot_create_2_1::Tank>("tank");

	// Initialize service
	irobot_create_2_1::Tank srv;

	// loop until time or ros not ok
	while(ros::ok())
	{
		ros::spinOnce();	// Receive callbacks	
		merge_new_msgs();   	// Merge joysticks
/*		
		if(joy_a) // Start if A is pressed
			start_flag = true;
		if(joy_b) // Stop when B is pressed
			start_flag = false;

		if(start_flag) // If A has been pressed, run main loop
				{
*/
			output = mixer(joy_x, joy_y);
			
			// set srv values
			srv.request.left = output.x;
			srv.request.right= output.y;
			srv.request.clear= 1;
			// send out value to robot
			if(client.call(srv))
			{
				ROS_INFO("Sending stuff");
				ROS_INFO("joy %d %d",output.x,output.y);
				ROS_INFO("service %d %d",srv.request.left,srv.request.right);
				if (srv.response.success)
					ROS_INFO("Response");
				else
					{ROS_INFO("No Response");}
			}
			else
				ROS_INFO("NOT Sending stuff");
	}

}

