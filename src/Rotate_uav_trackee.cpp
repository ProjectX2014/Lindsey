
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/Vector3.h>

int had_message_1=0;
double rotation_roll =0.0;
double rotation_pitch =0.0;
double rotation_yaw =0.0;

void state_callback(const ardrone_autonomy::Navdata& msg_in) //will need this later to convert relative measurments
{
	had_message_1=1;
	const float deg2rad= 0.0174532925;
	//Take in states of robot that we are interested in from the navdata topic
	rotation_roll=msg_in.rotX*deg2rad;
	rotation_pitch=msg_in.rotY*deg2rad;
	rotation_yaw=msg_in.rotZ*deg2rad;
}

int main(int argc, char** argv){

	ros::init(argc, argv, "Rotate_tag");
	ros::NodeHandle node;
	ros::Rate loop_rate(200);
	tf::TransformListener listener;
	geometry_msgs::Vector3 relative_position;

	ros::Publisher tag_pub = node.advertise<geometry_msgs::Vector3> ("UAV_Trackee_RelPos", 1);

	while(ros::ok()){

			//look up transform
			tf::StampedTransform transform;
	    try{
		      listener.lookupTransform("/UAV", "/trackee", ros::Time(0), transform);
		    }
	    catch (tf::TransformException ex){
		      ROS_ERROR("%s",ex.what());
	    }
			relative_position.x=transform.getOrigin().x();
			relative_position.y=transform.getOrigin().y();
			relative_position.z=transform.getOrigin().z();
			tag_pub.publish(relative_position); //publish tag for easy use in Kalman filter

			ros::spinOnce();
			loop_rate.sleep();
	}
return 0;
}
