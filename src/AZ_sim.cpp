
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>
#include <ros/time.h>
#include <Lindsey/AZTag.h>

#define LOCATORTF "vicon/QuuppaLocator/Locator"
#define TRACKEETF "vicon/QuuppaWand/Wand"



geometry_msgs::Vector3 time_step;
Lindsey::AZTag Tags;
const float rad2deg=57.29577;
int message=0;

void tag_callback(const Lindsey::AZTag& msg)
{
	Tags=msg;
	message=1;
}

int main(int argc, char** argv){

ros::init(argc, argv, "AZ_Sim");
ros::NodeHandle node;
ros::Rate loop_rate(50);

ros::Subscriber tag_sub = node.subscribe("Quuppa_AZ", 1, tag_callback);
ros::Publisher v3_msg = node.advertise<geometry_msgs::Vector3>("AZM_error", 1); 
ros::Publisher v3_msg2 = node.advertise<geometry_msgs::Vector3>("AZM_real", 1);

geometry_msgs::Vector3 tag;
tf::TransformBroadcaster br;
tf::TransformListener listener;
geometry_msgs::Vector3 error; 
geometry_msgs::Vector3 real; 


tf::StampedTransform err_transform;

int point =0;
float azm =0.0; //like a clock
float zen = 0.0; // from the normal
float opo =0.0;
float mes_azm= 0.0;
float mes_zen= 0.0;


ROS_INFO("Az sim: starting");
while(ros::ok() ){
	
		try{
		listener.lookupTransform(LOCATORTF, TRACKEETF, ros::Time(0), err_transform); 	
		}
		catch (tf::TransformException ex){
	        ROS_ERROR("%s",ex.what());
		}
		

		tag.z=err_transform.getOrigin().z();
		tag.y=err_transform.getOrigin().x();
		tag.x=-err_transform.getOrigin().y();

		ROS_INFO("DEF x %f y %f z %f l2 %f",tag.x,tag.y,tag.z,sqrt(pow(tag.x,2)+pow(tag.y,2)+pow(tag.z,2)));

		opo=sqrt(tag.y*tag.y+tag.x*tag.x);
		zen=abs(atan(opo/tag.z)*rad2deg);
		azm=atan2(tag.y,tag.x)*rad2deg;
	

		ROS_INFO("VICON AZM %f ZEN %f",azm,zen);
if (message)
{
		mes_azm=Tags.azm[0];
	//	if (mes_azm>0) {mes_azm-=180;}
	//	else {mes_azm+=180;}
	//if (mes_azm<0){mes_azm+=360;};
		mes_zen=Tags.zen[0];
		
		error.x=(mes_azm-azm);
		error.y=(mes_zen-zen);
		v3_msg.publish(error);
		real.x=azm;
		real.y=zen;
		v3_msg2.publish(real);
		
		ROS_INFO("Quuppa AZM %f ZEN %f",mes_azm,mes_zen);
		ROS_INFO("ERROR AZM %f ZEN %f",error.x,error.y);
		ROS_INFO("\n");

}	
				//SPIN		
				ros::spinOnce();
				loop_rate.sleep();
}

return 0;
}
