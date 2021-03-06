/*
Parker Conroy
ARLab @ University of Utah


This is sample Kalman filter code.

x(t)= A*x(t-1) +B*u(t-1) + w(t-1)
z(t)=Hx(t)+v(t)

x vector is n x 1
u vector is l x 1
z vector is m x 1

A matrix is n x n
B matrix is n x l
H matrix is m x n
Kalman Gain K matrix is m x n  
w is process white noise ~ N(0,Q)
v is measurement white noise ~ N(0,R)
*/

#include <ros/ros.h>
#include <Eigen/Dense> //needs to be eigen for groovy, Eigen for fuerte?
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <ardrone_autonomy/Navdata.h>
#include <tf/transform_listener.h>
#include "kalman.h"
#include <math.h>
#include <sstream>


std_msgs::Float32MultiArray x_msg;
const float deg2rad= 0.0174532925;
int had_message_1 =0; //state callback from ardrone

float vel_x=0.0;
float vel_y=0.0;
float vel_z=0.0;
double tag_x=0.0;
double tag_y=0.0;
double tag_z=0.0;
double tag_x_old =0.0;
double tag_y_old =0.0;
double tag_z_old =0.0;
float ux=0.0;
float uy=0.0;
float uz=0.0;

void state_callback(const ardrone_autonomy::Navdata& msg_in)
{
	had_message_1=1;

	vel_x=msg_in.vx*0.001; //  mm/s to m/s
	vel_y=msg_in.vy*0.001; //  mm/s to m/s
	vel_z=msg_in.vz*0.001; //  mm/s to m/s

//	ROS_INFO("KALMAN vx: %4f vy: %4f, %4f",vel_x,vel_y,vel_z);
}

void cmd_callback(const geometry_msgs::Vector3& cmd_in)
{
	//Take in commands for u in ax+bu

	ux=cmd_in.x; //in m/sec
	uy=cmd_in.y; //in m/sec
	uz=cmd_in.z; //in m/sec
	u1_old<< ux,uy,uz;
}

void tag_callback(const geometry_msgs::Vector3& tag_in)
{
	tag_x=tag_in.x; //in m/sec
	tag_y=tag_in.y; //in m/sec
	tag_z=tag_in.z; //in m/sec
}

void get_new_residual_and_H(void){
	//UPDATE Z
	//z = [p12_x p12_y p12_z v1x v1y v1z]T
	//geometry_msgs::Vector3 tag_pos=transform_update();
	z(0)=tag_x; 
	z(1)=tag_y; 
	z(2)=tag_z;
	z(3)=vel_x;
	z(4)=vel_y;
	z(5)=vel_z;

	ROS_INFO("Observation; TAG %f %f %f  VEL %f %f %f",z(0),z(1),z(2),z(3),z(4),z(5));

	//Make the residual
	y=z-H*x_minus;

}

void get_new_residual_and_H_wo_tag(void){
//UPDATE Z
	//z = [v1x v1y v1z]T
	z_wo_tag(0)=vel_x; 
	z_wo_tag(1)=vel_y; 
	z_wo_tag(2)=vel_z;

	ROS_INFO("Observation; VEL %f %f %f",z(0),z(1),z(2));

	//Make the residual
	y_wo_tag=z_wo_tag-H_wo_tag*x_minus;
}


int main(int argc, char** argv)
{
	//ROS stuff
	ROS_INFO("Starting Kalman");
	ros::init(argc, argv,"Kalman_filter");
   	ros::NodeHandle node;
	ros::Rate loop_rate(ROS_HZ);
	ros::Subscriber nav_sub;
	ros::Subscriber imu_sub;
	ros::Subscriber tag_sub;
	ros::Subscriber cmd_sub;
	ros::Publisher state_pub;
	//ros::Publisher _pub;
	tf::TransformListener listener;
	std::ostringstream OUT;

	state_pub = node.advertise<std_msgs::Float32MultiArray> ("state_post_KF", 1);
	nav_sub = node.subscribe("ardrone/navdata", 1, state_callback);
	tag_sub = node.subscribe("Transformed_tag_v3", 1, tag_callback);
	cmd_sub = node.subscribe("cmd_vel_u", 1, cmd_callback);
	
	init_matrix(); //Setup Matrices 

	ROS_INFO("Starting Kalman loop");
	bool saidMsg =false;
	while ( (had_message_1 ==0) )
	{
		if (!saidMsg){
		ROS_INFO("Kalman waiting for message");
		saidMsg=true;
		}
		ros::spinOnce();
		loop_rate.sleep();
}

	while (ros::ok() && had_message_1 ){

		bool kalman_with_tag;

		if (tag_x ==tag_x_old && tag_y ==tag_y_old &&tag_z ==tag_z_old)
			{kalman_with_tag = false;}
		else
			{kalman_with_tag = true;}

		ROS_INFO("Kalman Tag %f %f %f",tag_x,tag_y,tag_z);

		//Prediction Step
		x_minus=A*x_old+B*u1_old;
		P_minus=A*P_old*A.transpose() + Q;

if (kalman_with_tag)
{
		get_new_residual_and_H();
		//Correction Step
		O=H*P_minus*H.transpose()+R;
		K=P_minus*H.transpose()*O.inverse();
		x=x_minus+K*y;
		P=(I-K*H)*P_minus;

		/*
		std::ostringstream OUT11;
		OUT11 << B;
		ROS_INFO("Kalman_w_tag B: %s",OUT11.str().c_str());
		
		std::ostringstream OUT10;
		OUT10 << A;
		ROS_INFO("Kalman_w_tag A: %s",OUT10.str().c_str());
		*/

}
else
{
		get_new_residual_and_H_wo_tag();
		O_wo_tag=H_wo_tag*P_minus*H_wo_tag.transpose()+R_wo_tag;
		K_wo_tag=P_minus*H_wo_tag.transpose()*O_wo_tag.inverse();
		x=x_minus+K_wo_tag*y_wo_tag;
		P=(I-K_wo_tag*H_wo_tag)*P_minus;
}

 
	 	x_msg.data.clear(); //clear data
		float move =0.0;
	    for (long int i=0; i<dimention_n; i++)
			{
			move=x[i];
			x_msg.data.push_back(move);//fill msg
			}

		ROS_INFO("Kalman State; Tag %f %f %f vel1 %f %f %f vel2 %f %f %f  ",x(0),x(1),x(2),x(3),x(4),x(5),x(6),x(7),x(8));

		state_pub.publish(x_msg); //publish message
	//set old variable
		x_old=x;
		P_old=P;
		tag_x_old=tag_x;
		tag_y_old=tag_y;
		tag_z_old=tag_z;
		ros::spinOnce();
		loop_rate.sleep();
		}//while ros ok

	ROS_ERROR("Kalman has exited");
}//main
