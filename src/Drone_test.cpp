/*
Parker Conroy
Algorithmic Robotics Lab @ University of Utah


This code actuates the ARdrone from a generic joystick message. It is open loop.
It is intended as a simple example for those starting with the AR Drone platform.
*/
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Vector3.h>
#include <ardrone_autonomy/Navdata.h>

double max_speed = 1.0; //[m/s]
double max_speed_yaw = 0.5; //[m/s2]
double Kp= 1.0;
double Kd= 0.5;
double des_altd= 1.0;

double joy_x_,joy_y_,joy_z_,joy_yaw_;
int joy_a_,joy_b_,joy_xbox_,joy_yb_;
double joy_x,joy_y,joy_z,joy_yaw;
int joy_a,joy_b,joy_xbox,joy_yb;

double drone_vx_, drone_vy_ , drone_vz_;
double drone_ax_, drone_ay_ , drone_az_, drone_altd_;
double drone_vx, drone_vy , drone_vz;
double drone_ax, drone_ay , drone_az, drone_altd;

double tag_x=0.0;
double tag_y=0.0;
double tag_z=0.0;

double cmd_x,cmd_y,cmd_z,cmd_yaw;
int new_msg=0;
int drone_state =0; 
// state: {0 is failure, 2 is landed, 3 is flying, 4 is hovering, 6 taking off, 8 landing}
float forget =0.99;
//double joy_x_old,joy_y_old,joy_z_old;
geometry_msgs::Twist twist_msg;
std_msgs::Empty emp_msg;
geometry_msgs::Vector3 v3_msg; //[x, y,z]
sensor_msgs::Joy joy_msg_in;


void joy_callback(const sensor_msgs::Joy& joy_msg_in)
{
	//Take in xbox controller
	joy_x_=joy_msg_in.axes[1]; //left stick up-down
	joy_y_=joy_msg_in.axes[0]; //left stick left-right

	joy_yaw_=joy_msg_in.axes[3]; //right stick left-right

	joy_z_=joy_msg_in.axes[4]; //right stick up-down
	joy_a_=joy_msg_in.buttons[0]; //a button
	joy_b_=joy_msg_in.buttons[1]; //b button
	joy_yb_=joy_msg_in.buttons[3]; //Y button
	joy_xbox_=joy_msg_in.buttons[8]; //xbox button

	//Take in time
	//msg_time=(double)ros::Time::now().toNSec();
    new_msg=1;
}

void nav_callback(const ardrone_autonomy::Navdata& msg_in)
{
	//Take in state of ardrone	
	drone_vx_=msg_in.vx*0.001; //[mm/s] to [m/s]
	drone_vy_=msg_in.vy*0.001;	
	drone_vz_=msg_in.vz*0.001;
	drone_altd_=msg_in.altd*0.001;

	drone_ax_=msg_in.ax*9.8; //[g] to [m/s2]
	drone_ay_=msg_in.ay*9.8;	
	drone_az_=msg_in.az*9.8;

	drone_state=msg_in.state;	
	//ROS_INFO("getting sensor reading");	
}


void test_controller(double vx_des,double vy_des,double altd_des,double Kp, double Kd)
{
		geometry_msgs::Twist twist_msg_gen;

		cmd_x=Kp*(vx_des-drone_vx); //-Kd *drone_vx_	; //{-1 to 1}=K*( m/s - m/s)
		cmd_y=Kp*(vy_des-drone_vy); 
		//cmd_z=Kp*(vz_des-drone_vz);
		cmd_z=Kp*(altd_des-drone_altd);
		twist_msg_gen.angular.x=1.0; 
		twist_msg_gen.angular.y=1.0;
		twist_msg_gen.angular.z=0.0;

		if (cmd_z > 1.0)
			{cmd_z=1.0;}
		if (cmd_z < 0.0)
			{cmd_z=0.0;}
}		

double map(double value, double in_min, double in_max, double out_min, double out_max) {
  return (double)((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}	

void merge_new_mgs(void){
		joy_x=joy_x_;
		joy_y=joy_y_;
		joy_z=joy_z_;
		joy_yaw=joy_yaw_;
		joy_a=joy_a_;
		joy_b=joy_b_;
		joy_yb=joy_yb_;
		joy_xbox=joy_xbox_;
		drone_vx=drone_vx_;
		drone_vy=drone_vy_;
		drone_vz=drone_vz_;
		drone_ax=drone_ax_;
		drone_ay=drone_ay_;
		drone_az=drone_az_;
		drone_altd=drone_altd_;
	}

void tracking_callback(const geometry_msgs::Vector3& tag_in)
{
	tag_x=tag_in.x; //in m/sec
	tag_y=tag_in.y; //in m/sec
	tag_z=tag_in.z; //in m/sec
}


int main(int argc, char** argv)
{
	ros::init(argc, argv,"ARDrone_fly_from_joy");
    	ros::NodeHandle node;
   	ros::Rate loop_rate(50);
	ros::Publisher pub_twist;
	ros::Publisher pub_empty_reset;
	ros::Publisher pub_empty_land;
	ros::Publisher pub_empty_takeoff;
	ros::Publisher pub_v3;
	ros::Subscriber joy_sub;
	ros::Subscriber nav_sub;

    pub_twist = node.advertise<geometry_msgs::Twist>("cmd_vel", 1); 
    pub_v3 = node.advertise<geometry_msgs::Vector3>("joy_vel", 1); 
	joy_sub = node.subscribe("joy", 1, joy_callback);
	nav_sub = node.subscribe("ardrone/navdata", 1, nav_callback);	
	pub_empty_reset = node.advertise<std_msgs::Empty>("ardrone/reset", 1);
	pub_empty_takeoff = node.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
	pub_empty_land = node.advertise<std_msgs::Empty>("ardrone/land", 1);
	ros::Subscriber track_sub = node.subscribe("/UAV_Trackee_RelPos", 1, tracking_callback);


    ROS_INFO("Starting Test Node, /cmd_vel = f(joy)");
 	while (ros::ok()) {
	merge_new_mgs();
		//commands to change state of drone
		if (joy_a){
			while (drone_state ==2){
				ROS_INFO("Launching drone");
				pub_empty_takeoff.publish(emp_msg); //launches the drone
				ros::spinOnce();
				loop_rate.sleep();
			}//drone take off
		}	
		if (joy_b){
			while (drone_state ==3 || drone_state ==4){
				ROS_INFO("landing drone");
				pub_empty_land.publish(emp_msg); //launches the drone
				ros::spinOnce();
				loop_rate.sleep();
			}//drone land
		}
		if (joy_xbox){
			double time_start=(double)ros::Time::now().toSec();
			while (drone_state ==0 ){
				ROS_INFO("resetting drone");
				pub_empty_reset.publish(emp_msg); //resets the drone
				ros::spinOnce();
				loop_rate.sleep();
				if((double)ros::Time::now().toSec()> time_start+3.0){ 					
					ROS_ERROR("Time limit reached, unable reset ardrone");
					break; //exit loop
				}
			}//drone take off	
		}
		if (joy_yb){

			
			ROS_INFO("Auto Drone x: %f y: %f z: %f",tag_x,tag_y,tag_z);
		twist_msg.linear.x= tag_x;
		twist_msg.linear.y= tag_y;
		twist_msg.linear.z= tag_z +1.0; //plus meters 
		twist_msg.angular.z=0.0;
		//pub_twist.publish(twist_msg); //enable later

		}
else{ //control via joystick

		if (fabs(joy_x)<0.1) {joy_x =0;}
		//else {joy_x=joy_x*forget+joy_x_old*(1-forget);} //smoothing via forget

		if (fabs(joy_y)<0.1) {joy_y =0;}
		//else {joy_y=joy_y*forget+joy_y_old*(1-forget);}

		if (fabs(joy_z)<0.1) {joy_z =0;}
		//else {joy_z=joy_z*forget+joy_z_old*(1-forget);} 

		if (fabs(joy_yaw)<0.1) {joy_yaw =0;}
		//else {joy_z=joy_z*forget+joy_z_old*(1-forget);} 

		cmd_x= joy_x*max_speed;
		cmd_y= joy_y*max_speed;
		cmd_z= joy_z*max_speed;
		cmd_yaw= joy_yaw*max_speed_yaw;
		ROS_INFO("Before x: %f y: %f z: %f",cmd_x,cmd_y,cmd_z);
		//test_controller(cmd_x,cmd_y,cmd_z,Kp,Kd); //modifies cmd_x,cmd_y,cmd_z proportinal to quad_speed
		//test_controller(cmd_x,cmd_y,des_altd,Kp,Kd); //with pos on z
		ROS_INFO("After x: %f y: %f z: %f",cmd_x,cmd_y,cmd_z);
		twist_msg.linear.x=cmd_x;
		twist_msg.linear.y=cmd_y;	
		twist_msg.linear.z=cmd_z;
		twist_msg.angular.z=cmd_yaw;	

		v3_msg.x=cmd_x;
		v3_msg.y=cmd_y;
		v3_msg.z=cmd_z;

		new_msg=0;
		pub_v3.publish(v3_msg);
		pub_twist.publish(twist_msg);

		ros::spinOnce();
		loop_rate.sleep();
	}
		}//ros::ok
ROS_ERROR("ROS::ok failed- Node Closing");
}//main
