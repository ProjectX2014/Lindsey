
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32MultiArray.h>
#include <ardrone_autonomy/Navdata.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <Lindsey/Tag.h>
#include <ros/time.h>
ros::Time Time_old;
int measure_time =1;
ros::Duration step;
geometry_msgs::Vector3 time_step;

/*Tag.h
int16 NumTags
int16[] Id
float32[] PosX
float32[] PosY
float32[] PosZ
float32[] SmoPosX
float32[] SmoPosY
float32[] SmoPosZ
*/

//#include <tf/StampedTransform.h>

const int dimention_n =3;
typedef Eigen::Matrix<float, dimention_n, dimention_n> rotationMatrix;
typedef Eigen::Matrix<float,dimention_n,1> State_vector;

int had_message_1=0; //Quuppa tags
const float deg2rad= 0.0174532925;
geometry_msgs::Vector3 tag_in;
float x_old[]= {0.0,0.0,0.0,0.0};
float y_old[]= {0.0,0.0,0.0,0.0};
float z_old[]= {0.0,0.0,0.0,0.0};
//double rotation_roll =0.0;
//double rotation_pitch =0.0;
//double rotation_yaw =0.0;
Lindsey::Tag QTags;
int number_tags =0;
int number_tags_old =0;
int change_tag_number =0;
float RotX=0.0;
float RotY=0.0;
float RotZ=0.0;
//other params
float max =0.01;//kinda AR.drone
//float max =0.005; //max of 500 mm/s Roomba

//TAGS WE CARE ABOUT
std::string uav_name = "001830ecf431";
const int cov_uav =1; //meters

//std::string trackee_name = "78c5e56f39ea";
std::string trackee_name = "001830ecf432";
const int cov_trackee =1; //meters
//END TAGS WE CARE ABOUT



/*
QTags.NumTags = number_tags;

QTags.Id[number_tags];
QTags.Name[number_tags];
QTags.PosX[number_tags];
QTags.PosY[number_tags];
QTags.PosZ[number_tags];
QTags.SmoPosX[number_tags];
QTags.SmoPosY[number_tags];
QTags.SmoPosZ[number_tags];
*/

float lpf(float x, float x_old, float max)
{
float out;
float change =x-x_old;

if(change>max){
	out=x_old +max;
}
else if (change<-max)
{
	out=x_old -max;
}
else 
{
	out=x;
}
	return out;
}

void nav_callback(const ardrone_autonomy::Navdata& msg_in)
{
	//Take in state of ardrone	
	RotX=msg_in.rotX*deg2rad; //[mm/s] to [m/s]
	RotY=msg_in.rotY*deg2rad;	
	RotZ=msg_in.rotZ*deg2rad;
/*
	drone_ax_=msg_in.ax*9.8; //[g] to [m/s2]
	drone_ay_=msg_in.ay*9.8;	
	drone_az_=msg_in.az*9.8;
*/
	//ROS_INFO("getting sensor reading");	
}

void tag_callback(const Lindsey::Tag& tag_in_)
	{
	number_tags=tag_in_.NumTags;
	if (number_tags-number_tags_old){ //if numbers are the same

	QTags.Id[number_tags];
	QTags.Name[number_tags];
	QTags.PosX[number_tags];
	QTags.PosY[number_tags];
	QTags.PosZ[number_tags];
	QTags.SmoPosX[number_tags];
	QTags.SmoPosY[number_tags];
	QTags.SmoPosZ[number_tags];
	number_tags_old=number_tags;
	change_tag_number =1;
}
else	{change_tag_number =0;}
	
	QTags=tag_in_;
	had_message_1=1;


	}


int main(int argc, char** argv){

ros::init(argc, argv, "Quuppa_decode");
ros::NodeHandle node;
ros::Rate loop_rate(200);
nav_msgs::Odometry odom_data;
sensor_msgs::Imu imu_data;
geometry_msgs::Vector3 rotated_tag;
Lindsey::Tag QTags_local;
tf::TransformBroadcaster br;

bool saidMsg = false;
int i=0;//for the for loop
ros::Subscriber tag_sub = node.subscribe("Quuppa_raw", 1, tag_callback);
ros::Subscriber nav_sub = node.subscribe("ardrone/navdata", 1, nav_callback);	

ros::Publisher time_msg = node.advertise<geometry_msgs::Vector3>("Quuppa_timestep", 1);
ros::Publisher tag_pub = node.advertise<geometry_msgs::Vector3> ("Quuppa_analy", 1);
ros::Publisher odom_uav_pub = node.advertise<nav_msgs::Odometry> ("UAV/vo", 1);
ros::Publisher odom_trackee_pub = node.advertise<nav_msgs::Odometry> ("Trackee/vo", 1);
//ros::Publisher pub_v3 = node.advertise<geometry_msgs::Vector3>("UAV_Pos", 1);

tf::Transform transform;

std::ostringstream os;
nav_msgs::Odometry odom_uav_data;
nav_msgs::Odometry odom_trackee_data;
geometry_msgs::Vector3 inspect;
geometry_msgs::Vector3 est_location;

while ( (had_message_1 ==0) )
	{
		if (!saidMsg){
		ROS_INFO("Quuppa_decode waiting for message");
		saidMsg=true;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

while(ros::ok() && had_message_1){

Time_old= ros::Time::now();
		for(i = 0; i < QTags.NumTags; i++)
		
		
			{
				
			if (0==uav_name.compare( QTags.Id[i].c_str() ) )
				{//track with ekf
				//ROS_INFO("UAV on tag %i",i);
/*
				odom_uav_data.header.stamp= ros::Time::now();
				odom_uav_data.header.frame_id = "UAV";   // the tracked robot frame
		 		odom_uav_data.pose.pose.position.x = QTags.SmoPosX[i];         // x measurement Quuppa.
		 		odom_uav_data.pose.pose.position.y = QTags.SmoPosY[i]; // y measurement Quuppa.
		 		odom_uav_data.pose.pose.position.z = QTags.SmoPosZ[i];  // z measurement Quuppa.
		 		odom_uav_data.pose.pose.orientation.x = 1;   // identity quaternion
		 		odom_uav_data.pose.pose.orientation.y = 0;   // identity quaternion
		 		odom_uav_data.pose.pose.orientation.z = 0;   // identity quaternion
		 		odom_uav_data.pose.pose.orientation.w = 0;   // identity quaternion
				//1m accuracy in position, no accuracy in orientation (since we dont measure it yet)
				odom_uav_data.pose.covariance[0] =1.0;
				odom_uav_data.pose.covariance[7] =1.0;
				odom_uav_data.pose.covariance[14] =1.0;
				odom_uav_data.pose.covariance[21] =9999.0;
				odom_uav_data.pose.covariance[28] =9999.0;
				odom_uav_data.pose.covariance[35] =9999.0;
				//send message
				//odom_uav_pub.publish(odom_uav_data);

				//create tf smo
				transform.setOrigin( tf::Vector3(QTags.SmoPosX[i],QTags.SmoPosY[i],QTags.SmoPosZ[i]) );
				
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "UAV_Smo"));
				//create tf raw
				transform.setOrigin( tf::Vector3(QTags.PosX[i],QTags.PosY[i],QTags.PosZ[i]) );
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "UAV_Raw"));
				//output data straight to controller
	*/			
				//super smooth
				
				float x_lpf=lpf(QTags.PosX[i],x_old[0],max);
				float y_lpf=lpf(QTags.PosY[i],y_old[0],max);
				float z_lpf=lpf(QTags.PosZ[i],z_old[0],max);
				x_old[0]=x_lpf;
				y_old[0]=y_lpf;
				z_old[0]=z_lpf;

				//est_location.x=x_lpf;
				//est_location.y=y_lpf;
				//est_location.z=z_lpf;
				//pub_v3.publish(est_location);

				transform.setRotation( tf::Quaternion(0,0,RotZ) );
				transform.setOrigin( tf::Vector3(x_lpf,y_lpf,z_lpf) );
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "UAV_LPF"));

				}//end uav

			 if (0==trackee_name.compare( QTags.Id[i].c_str() ) )
				{//track with ekf
				//ROS_INFO("Trackee on tag %i",i);
/*
				odom_trackee_data.header.stamp= ros::Time::now();
				odom_trackee_data.header.frame_id = "Trackee";   // the tracked robot frame
		 		//odom_trackee_data.pose.pose.position.x = QTags.PosX[i];         // x measurement Quuppa.
		 		//odom_trackee_data.pose.pose.position.y = QTags.PosY[i]; // y measurement Quuppa.
		 		//odom_trackee_data.pose.pose.position.z = QTags.PosZ[i];  // z measurement Quuppa.
		 		//odom_trackee_data.pose.pose.position.x = QTags.SmoPosX[i];         // x measurement Quuppa.
		 		//odom_trackee_data.pose.pose.position.y = QTags.SmoPosY[i]; // y measurement Quuppa.
		 		//odom_trackee_data.pose.pose.position.z = QTags.SmoPosZ[i];  // z measurement Quuppa.

				odom_trackee_data.pose.pose.orientation.x = 1;   // identity quaternion
		 		odom_trackee_data.pose.pose.orientation.y = 0;   // identity quaternion
		 		odom_trackee_data.pose.pose.orientation.z = 0;   // identity quaternion
		 		odom_trackee_data.pose.pose.orientation.w = 0;   // identity quaternion
				//1m accuracy in position, no accuracy in orientation (since we dont measure it yet)
		 		odom_trackee_data.pose.covariance[0] =1.0;
				odom_trackee_data.pose.covariance[7] =1.0;
				odom_trackee_data.pose.covariance[14] =1.0;
				odom_trackee_data.pose.covariance[21] =9999.0;
				odom_trackee_data.pose.covariance[28] =9999.0;
				odom_trackee_data.pose.covariance[35] =9999.0;
*/
				//send trackee data				
					//odom_trackee_pub.publish(odom_trackee_data);
				//create tf
				//smo

//				transform.setOrigin( tf::Vector3(QTags.SmoPosX[i],QTags.SmoPosY[i],QTags.SmoPosZ[i]) );
//				transform.setRotation( tf::Quaternion(0,0,0) );
//				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "Trackee_Smo"));
			/*	//raw
				transform.setOrigin( tf::Vector3(QTags.PosX[i],QTags.PosY[i],QTags.PosZ[i]) );
				transform.setRotation( tf::Quaternion(0,0,0) );
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "Trackee_Raw"));
*/				
//super smooth
				float x_lpf=lpf(QTags.PosX[i],x_old[1],max);
				float y_lpf=lpf(QTags.PosY[i],y_old[1],max);
				float z_lpf=lpf(QTags.PosZ[i],z_old[1],max);
				x_old[1]=x_lpf;
				y_old[1]=y_lpf;
				z_old[1]=z_lpf;
				odom_trackee_data.pose.pose.position.x=x_lpf;
				odom_trackee_data.pose.pose.position.y=y_lpf;
				odom_trackee_data.pose.pose.position.z=z_lpf;
				odom_trackee_pub.publish(odom_trackee_data);

				transform.setOrigin( tf::Vector3(x_lpf,y_lpf,z_lpf) );
				transform.setRotation( tf::Quaternion(0,0,0) );
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "Trackee_LPF"));

				}//end trackee

			}// end if
ros::spinOnce();
		
if(measure_time)
{
				step=ros::Time::now()-Time_old;
				time_step.x=step.toSec();
				time_msg.publish(time_step);
}
loop_rate.sleep();

		
} //end ros::ok
return 0;
} //end main
