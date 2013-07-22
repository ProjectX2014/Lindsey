
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>
//#include <tf/StampedTransform.h>


int had_message_1=0;
int had_message_2=0;

float tag_[] = {0.0,0.0,0.0};
float tag[] = {0.0,0.0,0.0};
float cur_tag[3];

float init[] ={0.0,0.0,0.0};
float mid[]={-2.0,0.0,0.0};
float mid1[]={-4.0,-0.0,0.0};
float mid2[] ={-4.0,-1.0,0.0};
float mid3[]={-4.0,1.0,0.0};
float mid4[]={-3.0,0.0,0.0};

void tracking_callback(const geometry_msgs::Vector3& msg)
{
	had_message_1 =1;
	tag_[0]=msg.x;
	tag_[1]=msg.y;
	tag_[2]=msg.z; //meters
}

int main(int argc, char** argv){

ros::init(argc, argv, "Point2Point");
ros::NodeHandle node;
ros::Rate loop_rate(100);

ros::Subscriber track_sub = node.subscribe("UAV_Pos", 1, tracking_callback);
ros::Publisher pub_v3 = node.advertise<geometry_msgs::Vector3>("Point2Point", 1);
ros::Publisher next_pub_v3 = node.advertise<geometry_msgs::Vector3>("NextPoint", 1);
geometry_msgs::Vector3 next_point;
geometry_msgs::Vector3 error;
tf::TransformBroadcaster br;
tf::TransformListener listener;
tf::Transform transform;
tf::StampedTransform err_transform;
transform.setRotation( tf::Quaternion(0,0,0) );
int point =0;

ROS_INFO("P2P: waiting for message");
while ( (had_message_1 ==0))
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
ROS_INFO("P2P: waiting starting");
while(ros::ok() && (had_message_1==1)){
				//merge messages
				tag[0]=tag_[0];
				tag[1]=tag_[1];
				tag[2]=tag_[2];
				//how far away

				if (point ==0)      {cur_tag[0]=init[0]; cur_tag[1]=init[1]; cur_tag[2]=init[2];}//init
				else if (point ==1) {cur_tag[0]=mid[0]; cur_tag[1]=mid[1]; cur_tag[2]=mid[2];} //mid
				else if (point ==2) {cur_tag[0]=mid1[0]; cur_tag[1]=mid1[1]; cur_tag[2]=mid1[2];}//mid1
				else if (point ==3) {cur_tag[0]=mid2[0]; cur_tag[1]=mid2[1]; cur_tag[2]=mid2[2];}//mid2
				else if (point ==4) {cur_tag[0]=mid3[0]; cur_tag[1]=mid3[1]; cur_tag[2]=mid3[2];}//mid3
				else if (point ==5) {cur_tag[0]=mid4[0]; cur_tag[1]=mid4[1]; cur_tag[2]=mid4[2];}//mid4
				else if (point ==6) {cur_tag[0]=mid[0]; cur_tag[1]=mid[1]; cur_tag[2]=mid[2];}//mid
				else if (point ==7) {cur_tag[0]=init[0]; cur_tag[1]=init[1]; cur_tag[2]=init[2];}//init
				else if (point > 7) {/*done*/}
				float away= sqrt((tag[0]-cur_tag[0])*(tag[0]-cur_tag[0]) +(tag[1]-cur_tag[1])*(tag[1]-cur_tag[1])); //l2 norm
				//Change desired point in terms of where the UAV is 
				
				if (away <0.25){ point++;}
				ROS_INFO("away : %f",away);
				next_point.x=cur_tag[0];
				next_point.y=cur_tag[1];
				next_point.z=cur_tag[2];
				transform.setOrigin( tf::Vector3(cur_tag[0],cur_tag[1],cur_tag[2]) );
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "Des_Pos"));

				next_pub_v3.publish(next_point);
try{
				listener.lookupTransform("Des_Pos", "UAV_LPF", ros::Time(0), err_transform); //could also be ekf_output_UAV
}
catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
     }
				error.x=err_transform.getOrigin().x();
				error.y=err_transform.getOrigin().y();
				error.z=err_transform.getOrigin().z();
				pub_v3.publish(error);
				//SPIN		
				ros::spinOnce();
				loop_rate.sleep();
}
return 0;
}
