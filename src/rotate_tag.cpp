
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32MultiArray.h>
#include <ardrone_autonomy/Navdata.h>
#include <Eigen/Dense>
#include <geometry_msgs/Vector3.h>
//#include <tf/StampedTransform.h>

const int dimention_n =3;
typedef Eigen::Matrix<float, dimention_n, dimention_n> rotationMatrix;
typedef Eigen::Matrix<float,dimention_n,1> State_vector;

int had_message_1=0;
int had_message_2=0;
const float deg2rad= 0.0174532925;
geometry_msgs::Vector3 tag_in;
float x=0.0;
float y=0.0;
float z=0.0;
double rotation_roll =0.0;
double rotation_pitch =0.0;
double rotation_yaw =0.0;

void state_callback(const ardrone_autonomy::Navdata& msg_in)
{

	had_message_2=1;
	//Take in states of robot that we are interested in from the navdata topic
	rotation_roll=msg_in.rotX*deg2rad;
	rotation_pitch=msg_in.rotY*deg2rad;
	rotation_yaw=msg_in.rotZ*deg2rad;

//	time_stamp=msg_in.tm;	

}

void tag_callback(const geometry_msgs::Vector3& tag_in_){

tag_in=tag_in_; //place in local variable
had_message_1=1;
}


int main(int argc, char** argv){

ros::init(argc, argv, "Rotate_tag");
ros::NodeHandle node;
ros::Rate loop_rate(200);
tf::StampedTransform transRot;
tf::Transform transform;
tf::TransformBroadcaster br;
tf::TransformListener listener;
rotationMatrix R;
geometry_msgs::Vector3 rotated_tag;
geometry_msgs::Vector3 tag_local;

bool saidMsg = false;

ros::Subscriber nav_sub = node.subscribe("ardrone/navdata", 1, state_callback);
ros::Subscriber tag_sub = node.subscribe("Quuppa_relative_clean", 1, tag_callback);
ros::Publisher tag_pub = node.advertise<geometry_msgs::Vector3> ("Transformed_tag_v3", 1);

while ( (had_message_1 ==0) && (had_message_2==0))
	{
		if (!saidMsg){
		ROS_INFO("Rotate_tag waiting for message");
		saidMsg=true;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

while(ros::ok() && had_message_1 && had_message_2){

		//merge messages
		tag_local=tag_in;
		//
		transform.setOrigin( tf::Vector3(0, 0, 0) );
		transform.setRotation( tf::Quaternion(rotation_roll,rotation_pitch,rotation_yaw) );
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "ardrone_base_link"));

		//R= trans;
		transform.setOrigin( tf::Vector3(tag_local.x,tag_local.y,tag_local.z)  );
		ROS_INFO("TF; Displaying point: %f %f %f",x,y,z);
		transform.setRotation( tf::Quaternion(0, 0, 0) );
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "ardrone_base_link", "tracked_tag"));

		#if ROS_VERSION_MINIMUM(1,8,0)
  tf::Matrix3x3 trans;      
#else
  btMatrix3x3 trans;
#endif
		trans.setRPY(rotation_roll,rotation_pitch,0.0); //set roll and pitch values inside rotation matrix
		tf::Vector3 rotatedTrans=trans*tf::Vector3(x, y, z); //rotate the measured vector from local to global frame
		ROS_INFO("TF; Rotated point: %f %f %f",rotatedTrans[0],rotatedTrans[1],rotatedTrans[2]);
		transform.setOrigin( rotatedTrans );
		transform.setRotation( tf::Quaternion(0, 0, 0) ); //create a transform from such the global displacement
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "ardrone_base_link", "Transformed_tag_tf")); //send transform for visualization

		rotated_tag.x =rotatedTrans[0];
		rotated_tag.y=rotatedTrans[1];
		rotated_tag.z=rotatedTrans[2];
		tag_pub.publish(rotated_tag); //publish tag for easy use in Kalman filter

		ros::spinOnce();
		loop_rate.sleep();
}
return 0;
}
