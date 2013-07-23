#include <fstream>
#include <time.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <sstream>

#include <ros/ros.h>
#include <ros/time.h>
//#include <rosbag/bag.h>
//#include <rosbag/view.h>
//#include <boost/foreach.hpp>
#include <tf/transform_listener.h>
#include <std_msgs/Float32MultiArray.h>
//#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h> 
#include <geometry_msgs/Vector3.h> 
//#include <rosgraph_msgs/Log.h>
//#include <tf/tf.h>
//#include <sensor_msgs/Imu.h>

//makes csv from streaming data, use this program simultaniously with experiment or bag replaying

geometry_msgs::TransformStamped quad1_msg_keep;
geometry_msgs::TransformStamped quad2_msg_keep;
geometry_msgs::TransformStamped quad3_msg_keep;
std_msgs::Float32MultiArray state_keep;
/*
state_keep.data[0]=0;
state_keep.data[1]=0;
state_keep.data[2]=0;
state_keep.data[3]=0;
state_keep.data[4]=0;
state_keep.data[5]=0;
state_keep.data[6]=0;
state_keep.data[7]=0;
state_keep.data[8]=0;
*/
geometry_msgs::Vector3 p2p_keep;

geometry_msgs::Vector3 cmd_keep;

int m1 =0;
int m2 =0;
int m3 =0;
int m4 =0;
int m5 =0;
/*
void quad1_cb(const geometry_msgs::TransformStamped& pose_msg)
{
m1 =1;
  quad1_msg_keep=pose_msg;
}

void quad2_cb(const geometry_msgs::TransformStamped& pose_msg)
{
m2 =1;
  quad2_msg_keep=pose_msg;
}

void quad3_cb(const geometry_msgs::TransformStamped& pose_msg)
{
m3 =1;
  quad3_msg_keep=pose_msg;
}

*/
void p2p_cb(const geometry_msgs::Vector3& msg)
{
m4 =1;
  p2p_keep=msg;
}

void cmd_cb(const geometry_msgs::Vector3& msg)
{
m5 =1;
  cmd_keep=msg;
}

int main(int argc, char *argv[])
{
  ros::init (argc, argv, "csv_listener");
  ros::NodeHandle node;
  //ros::Subscriber quad1_sub = node.subscribe("/UAV_Smo", 1,  quad1_cb);
  //ros::Subscriber quad2_sub = node.subscribe("/UAV_Raw", 1, quad2_cb);
  //ros::Subscriber quad3_sub = node.subscribe("/UAV_LPF", 1, quad3_cb);
  ros::Subscriber p2p_sub = node.subscribe("NextPoint", 1, p2p_cb);
  ros::Subscriber cmd_sub = node.subscribe("joy_vel", 1, cmd_cb);
  ros::Rate loop_rate(30);
tf::TransformListener listener;

p2p_keep.x =0;
p2p_keep.y =0;
p2p_keep.z =0;
cmd_keep.x=0;
cmd_keep.y=0;
cmd_keep.z=0;
tf::StampedTransform transform1;
tf::StampedTransform transform2;
tf::StampedTransform transform3;
int count =0;
//ros::Time now;
state_keep.data[8];
/*
state_keep.data[0]=0;
state_keep.data[1]=0;
state_keep.data[2]=0;
state_keep.data[3]=0;
state_keep.data[4]=0;
state_keep.data[5]=0;
state_keep.data[6]=0;
state_keep.data[7]=0;
state_keep.data[8]=0;
*/
ROS_INFO("chech");
if ((argc = !2))
{
	
	std::cout << "Too many (or no) arguments........arg= filename_to_create \n";
	std::cout << "number of args:" << argc << "\n";
	std::cout << argv[1] << "\n";
	return -1;

}
	std::stringstream ss;
	std::string delim = ",";
	std::string argv_name;
	std::string argv_fullname;
	std::ofstream myfile;
	char str_now [80];

	ss << argv[1];
	ss >> argv_name;
	argv_fullname=argv_name;
	argv_name.erase(argv_name.end(),argv_name.end());
	
  
	ROS_INFO("Creating csv: %s.csv",argv[1]);
	
	

	myfile.open ((argv_name+ ".csv").c_str(), std::ios::out | std::ios::app);
	myfile  << str_now << "," << "Decomposition of " << argv_name << "\n";

myfile << "Sequence, Time,";

myfile<<  "Raw Position in x [m], Position in y [m], Position in z [m],";
myfile<<  "Smo Position in x [m], Position in y [m], Position in z [m],";
myfile<<  "LPF Position in x [m], Position in y [m], Position in z [m],";

myfile << "Desired Point Position x [m],Position y [m],Position z [m],";

myfile << "Cmd Vel x [m/s],Quad2 Vel y [m/s],Quad2 Vel z [m/s],";

myfile << std::endl; //end line

	if (0==(m4 && m5)){
	ROS_INFO("Waiting for message.");
	}
	while ((0==(m4 && m5)) ){
	//ROS_INFO("%d %d %d %d %d",m1,m2,m3,m4,m5);
	ros::spinOnce ();
	loop_rate.sleep ();
	}
	
	ROS_INFO("Creating csv");
	while (ros::ok() && ( 1==(m4 && m5) )  )
		{
		

	    try{
			listener.lookupTransform("/Des_Pos", "/UAV_LPF", ros::Time(0), transform3);
			listener.lookupTransform("/Des_Pos", "/UAV_Raw", ros::Time(0), transform1);
			listener.lookupTransform("/Des_Pos", "/UAV_Smo", ros::Time(0), transform2);
		    }
	    catch (tf::TransformException ex){
		      ROS_ERROR("%s",ex.what());
	    }
	/*
	//print quad1
		myfile << quad1_msg_keep.header.seq<< delim << quad1_msg_keep.header.stamp<< delim;
		myfile << quad1_msg_keep.transform.translation.x<< delim << quad1_msg_keep.transform.translation.y<< delim << quad1_msg_keep.transform.translation.z<< delim;
	//quad2
		myfile << quad2_msg_keep.transform.translation.x<< delim << quad2_msg_keep.transform.translation.y<< delim << quad2_msg_keep.transform.translation.z<< delim;
	//quad3
		myfile << quad3_msg_keep.transform.translation.x<< delim << quad3_msg_keep.transform.translation.y<< delim << quad3_msg_keep.transform.translation.z<< delim;
*/			

//print quad1
		myfile << count<< delim << ros::Time::now()<< delim;
	
		myfile << transform1.getOrigin().x()<< delim << transform1.getOrigin().y()<< delim << transform1.getOrigin().z()<< delim;
		myfile << transform2.getOrigin().x()<< delim << transform2.getOrigin().y()<< delim << transform2.getOrigin().z()<< delim;
		myfile << transform3.getOrigin().x()<< delim << transform3.getOrigin().y()<< delim << transform3.getOrigin().z()<< delim;
	



	//err
		myfile << p2p_keep.x << delim << p2p_keep.y << delim << p2p_keep.z << delim;
	//cmd
		myfile << cmd_keep.x << delim << cmd_keep.y << delim << cmd_keep.z << delim;
	//end the line
		myfile << std::endl; //end line

		count++;
		ros::spinOnce ();
		loop_rate.sleep ();
		}///while
	return 0;
	
}//main
