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
#include <ros/time.h>
#define ROSHZ 200

ros::Time Time_old;
int measure_time =1;
ros::Duration step;
geometry_msgs::Vector3 time_step;

double max_speed = 1.0; //[m/s]
double max_speed_yaw = 0.5; //[m/s2]
double Kp= 1.0;
double Kd= 0.5;
double des_altd= 1.0;

double joy_x_,joy_y_,joy_z_,joy_yaw_;
int joy_a_,joy_b_,joy_xbox_,joy_yb_,joy_xb_;
double joy_x,joy_y,joy_z,joy_yaw;
int joy_a,joy_b,joy_xbox,joy_yb,joy_xb;

double drone_vx_, drone_vy_ , drone_vz_;
double drone_ax_, drone_ay_ , drone_az_, drone_altd_;
double drone_vx, drone_vy , drone_vz;
double drone_ax, drone_ay , drone_az, drone_altd;

float tag[] = {0.0,0.0,0.0};
float tag_[] = {0.0,0.0,0.0};

float next_tag[] = {0.0,0.0,0.0};
float next_tag_[] = {0.0,0.0,0.0};

double cmd_x,cmd_y,cmd_z,cmd_yaw;
int new_msg=0;
int drone_state =0; 
// state: {0 is failure, 2 is landed, 3 is flying, 4 is hovering, 6 taking off, 8 landing}
float forget =0.99;
float kp=.2;
float kd= .15;
float max_speed_twist =0.4;
float tag_x_old =0;
float tag_y_old =0;
float tag_z_old =0;
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
	joy_xb_=joy_msg_in.buttons[2]; //b button
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
		joy_xb=joy_xb_;
		joy_xbox=joy_xbox_;
		drone_vx=drone_vx_;
		drone_vy=drone_vy_;
		drone_vz=drone_vz_;
		drone_ax=drone_ax_;
		drone_ay=drone_ay_;
		drone_az=drone_az_;
		drone_altd=drone_altd_;
		tag[0]=tag_[0];
		tag[1]=tag_[1];
		tag[2]=tag_[2];
		next_tag[0]=next_tag_[0];
		next_tag[1]=next_tag_[1];
		next_tag[2]=next_tag_[2];
	}

void tracking_callback(const geometry_msgs::Vector3& msg)
{
	tag_[0]=msg.x;
	tag_[1]=msg.y;
	tag_[2]=msg.z; //meters
}

void next_tracking_callback(const geometry_msgs::Vector3& msg)
{
	next_tag_[0]=msg.x;
	next_tag_[1]=msg.y;
	next_tag_[2]=msg.z; //meters
}


int main(int argc, char** argv)
{
	ros::init(argc, argv,"ARDrone_fly_from_joy");
    	ros::NodeHandle node;
   	ros::Rate loop_rate(ROSHZ);
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
	ros::Subscriber track_sub = node.subscribe("Point2Point", 1, tracking_callback);
	ros::Subscriber next_track_sub = node.subscribe("NextPoint", 1, next_tracking_callback);
	ros::Publisher time_msg = node.advertise<geometry_msgs::Vector3>("Drone_timestep", 1);
	geometry_msgs::Vector3 p_term;
	geometry_msgs::Vector3 d_term;

    ROS_INFO("Starting Test Node, /cmd_vel = f(joy)");
 	while (ros::ok()) {
		Time_old= ros::Time::now();
		merge_new_mgs();
		//commands to change state of drone
		while (joy_a){
			while (drone_state ==2){
				ROS_INFO("Launching drone");
				pub_empty_takeoff.publish(emp_msg); //launches the drone
				ros::spinOnce();
				loop_rate.sleep();
			}//drone take off
		}	
		while (joy_b){
			while (drone_state ==3 || drone_state ==4){
				ROS_INFO("landing drone");
				pub_empty_land.publish(emp_msg); //launches the drone
				ros::spinOnce();
				loop_rate.sleep();
			}//drone land
		}
		while (joy_xbox){
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
		
		while (joy_yb){
			//position hold
			merge_new_mgs();

			ROS_INFO("Error x: %f y: %f z: %f",tag[0],tag[1],tag[2]);	

			p_term.x=kp*tag[1];
			p_term.y=kp*-tag[0];
			p_term.z=kp*tag[2];			
			d_term.x=kd*((tag[1]-tag_x_old)*ROSHZ);
			d_term.y=-kd*((tag[0]-tag_y_old)*ROSHZ);
			d_term.z=kd*((tag[2]-tag_z_old)*ROSHZ);
			
			ROS_INFO("Cont px: %f py: %f",p_term.x,p_term.y);
			ROS_INFO("Cont dx: %f dy: %f",d_term.x,d_term.y);

			twist_msg.linear.x=p_term.x+d_term.x;
			twist_msg.linear.y=p_term.y+d_term.y;
			twist_msg.linear.z= p_term.z; 
			//twist_msg.angular.z=(kp*-next_tag[0]);
			twist_msg.angular.z=0.0;

			if(twist_msg.linear.x>max_speed_twist) {twist_msg.linear.x=max_speed_twist;}
			if(twist_msg.linear.x<-max_speed_twist) {twist_msg.linear.x=-max_speed_twist;}
			if(twist_msg.linear.y>max_speed_twist) {twist_msg.linear.y=max_speed_twist;}
			if(twist_msg.linear.y<-max_speed_twist) {twist_msg.linear.y=-max_speed_twist;}
			if(twist_msg.angular.z>2*max_speed_twist) {twist_msg.angular.z=2*max_speed_twist;}
			if(twist_msg.angular.z<-2*max_speed_twist) {twist_msg.angular.z=-2*max_speed_twist;}

			ROS_INFO("Auto Drone commands x: %f y: %f z: %f",twist_msg.linear.x,twist_msg.linear.y,twist_msg.angular.z);

			v3_msg.x=twist_msg.linear.x;
			v3_msg.y=twist_msg.linear.y;
			v3_msg.z=twist_msg.linear.z;
			pub_v3.publish(v3_msg);
		
			pub_twist.publish(twist_msg); 
			tag_x_old=tag[1];
			tag_y_old=tag[0]; 
			tag_z_old=tag[2]; 
			ros::spinOnce();
			loop_rate.sleep();

		}

		while (joy_xb){
			//Track point via yaw
			merge_new_mgs();

			ROS_INFO("Error x: %f y: %f z: %f",tag[0],tag[1],tag[2]);	
			ROS_INFO("Rot commands z: %f",twist_msg.angular.z);
			p_term.x=kp*tag[1];
			p_term.y=kp*-tag[0];
			d_term.x=kd*drone_vx;
			d_term.y=kd*drone_vy;
			
			twist_msg.linear.x= 0.0; 
			twist_msg.linear.y= 0.0; 
			twist_msg.linear.z= 0.0; 
			twist_msg.angular.z=(kp*-next_tag[0]);
			pub_twist.publish(twist_msg);
			ROS_INFO("ROT z: %f",twist_msg.angular.z);
			
			ros::spinOnce();
			loop_rate.sleep();

		}

//else{ //control via joystick

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
		ROS_INFO("Drone Command x: %f y: %f z: %f",cmd_x,cmd_y,cmd_z);
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
	//}
if(measure_time)
{
				step=ros::Time::now()-Time_old;
				time_step.x=step.toSec();
				time_msg.publish(time_step);
}

		}//ros::ok
ROS_ERROR("ROS::ok failed- Node Closing");
}//main
