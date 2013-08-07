
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

int had_message_1=0;
int had_message_2=0;

float tag_[] = {0.0,0.0,0.0};
float tag[] = {0.0,0.0,0.0};
float cur_tag[3];

float init[] ={0.0,0.0,0.0};
float mid[]={-2.0,0.0,0.0};
float mid1[]={-3.0,-0.0,0.0};
float mid2[] ={-3.0,-1.0,0.0};
float mid3[]={-3.0,1.0,0.0};
float mid4[]={-3.0,0.0,0.0};

void tracking_callback(const geometry_msgs::Vector3& msg)
{
	had_message_1 =1;
	tag_[0]=msg.x;
	tag_[1]=msg.y;
	tag_[2]=msg.z; //meters
}

int main(int argc, char** argv){

ros::init(argc, argv, "Point2Point_Node");
ros::NodeHandle node;
ros::Rate loop_rate(250);

//ros::Subscriber track_sub = node.subscribe("UAV_Pos", 1, tracking_callback);
ros::Publisher pub_v3 = node.advertise<geometry_msgs::Vector3>("Point2Point", 1);
ros::Publisher next_pub_v3 = node.advertise<geometry_msgs::Vector3>("NextPoint", 1);
ros::Publisher time_msg = node.advertise<geometry_msgs::Vector3>("P2P_timestep", 1);

geometry_msgs::Vector3 next_point;
geometry_msgs::Vector3 error;
tf::TransformBroadcaster br;
tf::TransformListener listener;
tf::Transform transform;
tf::StampedTransform err_transform;
tf::StampedTransform away_transform;
tf::StampedTransform yaw_transform;
transform.setRotation( tf::Quaternion(0,0,0) );
int point =0;

double timer = 9999999;
ROS_INFO("P2P: waiting for message");
/*
while ( (timer<.5))
	{
		step=ros::Time::now()-Time_old;
		timer=step.toSec();
		ros::spinOnce();
		loop_rate.sleep();
	}
*/
ros::Time Time_start= ros::Time::now();
ROS_INFO("P2P: starting");
while(ros::ok() ){
	
	
	
				if (point ==0)      {cur_tag[0]=init[0]; cur_tag[1]=init[1]; cur_tag[2]=init[2];}//init
				else if (point ==1) {cur_tag[0]=mid[0]; cur_tag[1]=mid[1]; cur_tag[2]=mid[2];} //mid
				else if (point ==2) {cur_tag[0]=mid1[0]; cur_tag[1]=mid1[1]; cur_tag[2]=mid1[2];}//mid1
				else if (point ==3) {cur_tag[0]=mid2[0]; cur_tag[1]=mid2[1]; cur_tag[2]=mid2[2];}//mid2
				else if (point ==4) {cur_tag[0]=mid3[0]; cur_tag[1]=mid3[1]; cur_tag[2]=mid3[2];}//mid3
				else if (point ==5) {cur_tag[0]=mid4[0]; cur_tag[1]=mid4[1]; cur_tag[2]=mid4[2];}//mid4
				else if (point ==6) {cur_tag[0]=mid[0]; cur_tag[1]=mid[1]; cur_tag[2]=mid[2];}//mid
				else if (point ==7) {cur_tag[0]=init[0]; cur_tag[1]=init[1]; cur_tag[2]=init[2];}//init
				else if (point > 7) {point=0;}
				
				//Place desired Point in space
				transform.setOrigin( tf::Vector3(cur_tag[0],cur_tag[1],cur_tag[2]) );
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "Des_Pos"));
				
				//Measure error between desired point and uav
try{
				//listener.waitForTransform(QUADTF,"Des_Pos", ros::Time(0), ros::Duration(.5) );
				listener.lookupTransform(QUADTF, "Des_Pos", ros::Time(0), err_transform); 
}
catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
     }
				error.x=err_transform.getOrigin().x();
				error.y=err_transform.getOrigin().y();
				error.z=err_transform.getOrigin().z();
				pub_v3.publish(error);

				//Change desired point in terms of where the UAV is 
				float away= sqrt(error.x*error.x+error.y*error.y); //l2 norm
						
				step=ros::Time::now()-Time_start;
				timer=step.toSec();
				//ROS_INFO("P2P timer: %f",timer);
				if (timer >.2){
						ROS_INFO("P2P point: %i",point);
							if (away <0.25){ 
								point++; 
								ROS_INFO("P2P Changing Desired Point");
								ROS_INFO("P2P away: %f",away);
								ROS_INFO("P2P point: %i",point);
								Time_start= ros::Time::now();}
						
						}
				//measure transform between uav and trackee for yaw
/*
try{
				listener.lookupTransform(QUADTF,TRACKEETF, ros::Time(0), yaw_transform); //could also be ekf_output_UAV
}
catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
     }
				next_point.x=yaw_transform.getOrigin().x();
				next_point.y=yaw_transform.getOrigin().y();
				next_point.z=yaw_transform.getOrigin().z();
				next_pub_v3.publish(next_point);

*/
if(measure_time)
{
				step=ros::Time::now()-Time_old;
				time_step.x=step.toSec();
				time_msg.publish(time_step);
				std::cout<< "things " <<"\n";
}
				//SPIN		
				ros::spinOnce();
				loop_rate.sleep();
}
return 0;
}
