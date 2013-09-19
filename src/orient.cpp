
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>
#include <ros/time.h>

#define QUADTF "vicon/Quad/Quad"
#define TRACKEETF "vicon/Roombu/Roomba"
ros::Time Time_old;
int measure_time =1;
ros::Duration step;
geometry_msgs::Vector3 time_step;
const float pi=3.14;
int had_message_1=0;
int had_message_2=0;
double roll, pitch, yaw;

float tag_[] = {0.0,0.0,0.0};
float tag[] = {0.0,0.0,0.0};
float cur_tag[3];
int main(int argc, char** argv){

ros::init(argc, argv, "Orient_Node");
ros::NodeHandle node;
ros::Rate loop_rate(100);

geometry_msgs::Vector3 next_point;
geometry_msgs::Vector3 error;
tf::TransformBroadcaster br;
tf::TransformListener listener;
tf::Transform transform;
tf::StampedTransform err_transform;
tf::StampedTransform init_transform;
tf::StampedTransform rot_transform;
transform.setRotation( tf::Quaternion(0,0,0) );
tf::Quaternion init_rotation;
tf::Quaternion cur_rotation;
int point =0;
int it_worked =0;
while(it_worked==0){
try{
listener.lookupTransform("world", QUADTF, ros::Time(0), init_transform); //could also be ekf_output_UAV
it_worked=1;
}
catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
	it_worked=0;
     }
}
init_rotation=init_transform.getRotation();
ROS_INFO("orient: starting");

while(ros::ok() ){
//transform.setRotation( tf::Quaternion(0,0,RotZ) );
			//Measure error between desired point and uav
try{
				listener.lookupTransform("world", QUADTF, ros::Time(0), err_transform); //could also be ekf_output_UAV
				listener.lookupTransform(QUADTF,"world", ros::Time(0), rot_transform); //could also be ekf_output_UAV
}
catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
     }
				
//ROS_INFO("Quad: %f %f %f",roll,pitch,yaw);

				
				err_transform.getBasis().getRPY(roll, pitch, yaw);
				
				transform.setRotation( tf::Quaternion(-roll+pi/2.1,-pitch,-yaw+pi/2.3) );
				
				transform.setOrigin( tf::Vector3(err_transform.getOrigin().x(),err_transform.getOrigin().y(),err_transform.getOrigin().z()) );
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "Quad_Err"));

				rot_transform.getBasis().getRPY(roll, pitch,yaw);

//				roll=-roll+pi/2.1;
//				pitch=-pitch;
//				yaw=-yaw+pi/2.3;
				ROS_INFO("Movement: %f %f %f",roll,pitch,yaw);
				cur_rotation=rot_transform.getRotation();
				transform.setRotation( tf::Quaternion(roll,yaw,pitch) );
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "Quad_rot"));
	
				
				cur_rotation=cur_rotation-init_rotation;
				rot_transform.getBasis().getRPY(roll, pitch, yaw);
				transform.setRotation( tf::Quaternion(roll,pitch,yaw) );
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "Quad_inv"));

try{
				listener.lookupTransform("Quad_inv", "world", ros::Time(0), err_transform); //could also be ekf_output_UAV
}
catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
     }
				err_transform.getBasis().getRPY(roll, pitch, yaw);
//ROS_INFO("Error: %f %f %f",roll,pitch,yaw);
/*
				transform.setRotation( tf::Quaternion(-roll,-pitch,-yaw) );
				transform.setOrigin( tf::Vector3(err_transform.getOrigin().x(),err_transform.getOrigin().y(),err_transform.getOrigin().z()) );
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "Trackee"));

*/
				//SPIN		
				ros::spinOnce();
				loop_rate.sleep();
}
return 0;
}
