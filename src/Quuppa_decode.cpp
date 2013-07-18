
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
float x=0.0;
float y=0.0;
float z=0.0;
double rotation_roll =0.0;
double rotation_pitch =0.0;
double rotation_yaw =0.0;
Lindsey::Tag QTags;
int number_tags =0;
int number_tags_old =0;
int change_tag_number =0;

//TAGS WE CARE ABOUT
std::string uav_name = "3304-0249-";
const int cov_uav =1; //meters

std::string trackee_name = "3304-0249-";
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

bool saidMsg = false;
int i=0;//for the for loop
ros::Subscriber tag_sub = node.subscribe("Quuppa_raw", 1, tag_callback);
ros::Publisher tag_pub = node.advertise<geometry_msgs::Vector3> ("Quuppa_relative_clean", 1);

ros::Publisher odom_uav_pub = node.advertise<nav_msgs::Odometry> ("uav_vo", 1);
ros::Publisher odom_trackee_pub = node.advertise<nav_msgs::Odometry> ("trackee_vo", 1);

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

/*
		//merge messages (may be really slow.... so not doing it now)
		
		
		number_tags=tag_in_.NumTags;
		if (change_tag_number){ //if numbers are the same

			QTags_local.Id[number_tags];
			QTags_local.Name[number_tags];
			QTags_local.PosX[number_tags];
			QTags_local.PosY[number_tags];
			QTags_local.PosZ[number_tags];
			QTags_local.SmoPosX[number_tags];
			QTags_local.SmoPosY[number_tags];
			QTags_local.SmoPosZ[number_tags];
			number_tags_old=number_tags;

		//
		QTags_local=QTags;
*/
		nav_msgs::Odometry odom_uav_data;
		nav_msgs::Odometry odom_trackee_data;
		
		
		/*   Do math to convert:
		 raw tag codes to relative uav + tracked 
		Set course markers 
		Extract velocity from tracked target */
		
		for(i = 0; i < QTags.NumTags; i++)
			{
			if (uav_name.compare(QTags.Name[i]))
				{//track with ekf
				odom_uav_data.header.stamp= ros::Time::now();
				odom_uav_data.header.frame_id = "UAV";   // the tracked robot frame
		 		odom_uav_data.pose.pose.position.x = QTags.PosX[i];         // x measurement GPS.
		 		odom_uav_data.pose.pose.position.y = QTags.PosY[i]; // y measurement GPS.
		 		odom_uav_data.pose.pose.position.z = QTags.PosZ[i];  // z measurement GPS.
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
				odom_uav_pub.publish(odom_uav_data);
				}

			if (trackee_name.compare(QTags.Name[i]))
				{//track with ekf
				odom_trackee_data.header.stamp= ros::Time::now();
				odom_trackee_data.header.frame_id = "trackee";   // the tracked robot frame
		 		odom_trackee_data.pose.pose.position.x = QTags.PosX[i];         // x measurement GPS.
		 		odom_trackee_data.pose.pose.position.y = QTags.PosY[i]; // y measurement GPS.
		 		odom_trackee_data.pose.pose.position.z = QTags.PosZ[i];  // z measurement GPS.
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
				odom_trackee_pub.publish(odom_trackee_data);
				}


/*
			//Display points in 3D space via tfs 
			tf::Transform transform;
			std::ostringstream os;

			transform.setOrigin( tf::Vector3(QTags.SmoPosX[i],QTags.SmoPosY[i],QTags.SmoPosZ[i]) );
			transform.setRotation( tf::Quaternion(0,0,0) );

			os << "Quuppa_Smooth_" << i;
			std::string Quuppa_tag_string_name = os.str();
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", Quuppa_tag_string_name));
*/						
			}// end if

		ros::spinOnce();
		loop_rate.sleep();
} //end ros::ok
return 0;
} //end main
