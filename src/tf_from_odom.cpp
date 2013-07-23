
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>
//#include <tf/StampedTransform.h>


int had_message_1=0;
int had_message_2=0;

float uav_x_=0.0;
float uav_y_=0.0;
float uav_z_=0.0;


float trackee_x_=0.0;
float trackee_y_=0.0;
float trackee_z_=0.0;

double rotation_roll =0.0;
double rotation_pitch =0.0;
double rotation_yaw =0.0;

void odom_callback_trackee(const geometry_msgs::PoseWithCovarianceStamped& msg_in)
{
	had_message_1=1;
	//Take in states of robot that we are interested in from the navdata topic
	trackee_x_=msg_in.pose.pose.position.x;
	trackee_y_=msg_in.pose.pose.position.y;
	trackee_z_=msg_in.pose.pose.position.z;
}


void odom_callback_uav(const geometry_msgs::PoseWithCovarianceStamped& msg_in)
{
	had_message_2=1;
	//Take in states of robot that we are interested in from the navdata topic
	uav_x_=msg_in.pose.pose.position.x;
	uav_y_=msg_in.pose.pose.position.y;
	uav_z_=msg_in.pose.pose.position.z;
}


int main(int argc, char** argv){

ros::init(argc, argv, "TF_From_Odom");
ros::NodeHandle node;
ros::Rate loop_rate(100);

tf::TransformBroadcaster br;
tf::TransformListener listener;

ros::Subscriber nav_sub_trackee = node.subscribe("/Trackee/robot_pose_ekf_Trackee/odom", 1, odom_callback_trackee);
ros::Subscriber nav_sub_uav = node.subscribe("/UAV/robot_pose_ekf_UAV/odom", 1, odom_callback_uav);

//ros::Publisher pub_v3 = node.advertise<geometry_msgs::Vector3>("UAV_Trackee_RelPos", 1);

tf::Transform transform;
transform.setRotation( tf::Quaternion(0,0,0) );
//geometry_msgs::Vector3 est_location;

ROS_INFO("tf to odom: waiting for message");
while ( (had_message_1 ==0) || (had_message_2 ==0))
	{
		
		ros::spinOnce();
		loop_rate.sleep();
	}
ROS_INFO("tf to odom: waiting starting");
while(ros::ok() && (had_message_1==1) && (had_message_2==1)){
				//merge messages
				float uav_x=uav_x_;
				float uav_y=uav_y_;
				float uav_z=uav_z_;
				float trackee_x=trackee_x_;
				float trackee_y=trackee_y_;
				float trackee_z=trackee_z_;
				//Do math to deturmine relative position
				/*
				est_location.x=uav_x;
				est_location.y=uav_y;
				est_location.z=uav_z;
				pub_v3.publish(est_location);
*/				
				//send transforms
								

				transform.setOrigin( tf::Vector3(trackee_x,trackee_y,trackee_z) );
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "Trackee_EKF"));

				transform.setOrigin( tf::Vector3(uav_x,uav_y,uav_z) );
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "UAV_EKF"));

				//SPIN		
				ros::spinOnce();
				loop_rate.sleep();
}
return 0;
}
