/*
Parker Conroy
ARLab

Servoing of AR.drone from vector3


*/


#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

//float error[2];
//float dt_error[2];
//float integral_error[2];
//float control[2];
//double dt;
//geometry_msgs::Twist twist_msg;

unsigned int cam_x= 640;
unsigned int cam_y= 480;
int x =0;
int y =1;
double roi_time = 0.0;
float roi_pos[2];
float error_old[2];
	

void roi_callback(const sensor_msgs::RegionOfInterest& roi_in)
	{
	//Something about finding the center of the ROI is wrong, luckily it is close.

	roi_pos[x]=(float)roi_in.x_offset+((float)(roi_in.height/2))-((float)(cam_x/2));
	roi_pos[y]=(float)roi_in.y_offset+((float)(roi_in.width/2))-((float)(cam_y/2));
	}

geometry_msgs::Twist Pid_controller(float roi)
	{
	//Controller Constants
	float k_p=1;
	float k_d=0.2;
	float k_i= 0;
	//End constants

	double dt= roi_time-(double)ros::Time::now().toSec();

	float error[2];
	error[x]=roi[x]/((float)cam_x); //normalize between -1 and 1
	error[y]=roi[y]/((float)cam_y);

	printf("roi x:%d y:%d \n", roi_in.x_offset, roi_in.y_offset);
	printf("tag x:%d y:%d \n", roi_in.height, roi_in.width);
	printf("roi_center x:%f y:%f \n", roi_pos[x], roi_pos[y]);
	printf("cam x:%d y:%d \n", cam_x, cam_y);
	printf("error x:%f y:%f \n", error[x], error[y]);

	float dt_error[2];
	dt_error[x]=(-error[x]+error_old[x])/dt;
	dt_error[y]=(-error[y]+error_old[y])/dt;

	float integral_error[2];
	integral_error[x]=integral_error[x]+error[x]; //set integral
	integral_error[y]=integral_error[y]+error[y];

	//printf("error_dot x:%f y:%f \n", dt_error[x], dt_error[y]);
	//printf("int_error x:%f y:%f \n", integral_error[x], integral_error[y]);

	float control[2];
	control[x]= k_p*error[x]+k_d*dt_error[x]+k_i*integral_error[x]; //set controls w/ PID
	control[y]= k_p*error[y]+k_d*dt_error[y]+k_i*integral_error[y];

	//printf("twist x:%f y:%f \n", control[x], control[y]);

	geometry_msgs::Twist twist_msg;
	twist_msg.linear.x=control[x];
	twist_msg.linear.y=control[y];

	//set terms for next cycle
	error_old[x]=error[x]; //set old error
	error_old[y]=error[y];
	roi_time = (double)ros::Time::now().toSec(); //set time
	}


void cam_callback(const sensor_msgs::CameraInfo& cam_info)
	{
	if (cam_y != cam_info.height)
	{cam_y=cam_info.height;}

	if(cam_x != cam_info.width)
	{cam_x=cam_info.width;}

//store camera info from cam info

}



int main(int argc, char** argv)
{
    ros::init(argc, argv,"ARDrone_roi");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
	
/*
	error[x]=0.0;
	error[y]=0.0;
	dt_error[x]=0.0;
	dt_error[y]=0.0;
	integral_error[x]=0.0;
	integral_error[y]=0.0;
*/

	ros::Subscriber track_sub = n.subscribe("/UAV_Trackee_RelPos", 1, tracking_callback);
        ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1); /* Message queue length is just 1 */
		
 	while (ros::ok() && had message ) {
		
		ros::spinOnce();
		//check messages for values outside suitable range
		
		geometry_msgs::Twist control = Pid_controller(roi_pos);

		if(control.linear.x >1) {control.linear.x =1;}
		else if(control.linear.x <-1) {control.linear.x =-1;}
			
		if(control.linear.y >1) {control.linear.y =1;}
		else if(control.linear.y <-1) {control.linear.y =-1;}

		pub.publish(control);
		
		loop_rate.sleep();


			}//ros::ok

}//main
