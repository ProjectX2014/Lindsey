/*
Parker Conroy
Parcon Robotics

This code samples a JSON tag from a url
you may need to link the library location after install via
$ ln -s /usr/local/lib/libjansson.so.4 /usr/lib/libjansson.so.4
*/

//#define URL_FORMAT   "http://192.168.0.3/quuppaTag.txt"
//#define URL_FORMAT   "www.kspresearch.com/docs/quuppaTag"
#define URL_FORMAT   "http://192.168.0.124:8080/qpe/getHAIPLocation"
#define URL_SIZE     256
//#define BUFFER_SIZE  (10*1024)  /* 10 KB */
#define BUFFER_SIZE  (256*1024)  /* 256 KB */

#include <ros/ros.h>
#include <curl/curl.h>
#include <jansson.h>
#include <string.h>
#include <Lindsey/Tag.h>
#include <ctime>

//for timing
#include <geometry_msgs/Vector3.h>
#include <ros/time.h>
ros::Time Time_old;
geometry_msgs::Vector3 time_step;
int measure_time =1;
//end tming

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

double PosX_old =0.0;
double PosY_old =0.0;
double PosZ_old =0.0;

int loop_times =0;
int DEBUG =0;
//std_msgs::Float32 timestep = 0.0;
void merge_new_mgs(void){
	
	}

static int newline_offset(const char *text)
{
    const char *newline = strchr(text, '\n');
    if(!newline)
        return strlen(text);
    else
        return (int)(newline - text);
}

struct write_result
{
    char *data;
    int pos;
};

static size_t write_response(void *ptr, size_t size, size_t nmemb, void *stream)
{
    struct write_result *result = (struct write_result *)stream;

    if(result->pos + size * nmemb >= BUFFER_SIZE - 1)
    {
        fprintf(stderr, "error: too small buffer\n");
        return 0;
    }

    memcpy(result->data + result->pos, ptr, size * nmemb);
    result->pos += size * nmemb;

    return size * nmemb;
}

static char *request(const char *url)
{
    CURL *curl;
    CURLcode status;
    char *data;
    long code;

    curl = curl_easy_init();
   data = (char*) malloc(256*1024);

if(!curl || !data)
        return NULL;

    struct write_result write_result;
	write_result.data =data;
		write_result.pos =0;

    curl_easy_setopt(curl, CURLOPT_URL, url);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_response);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &write_result);

    status = curl_easy_perform(curl);
    if(status != 0)
    {
        fprintf(stderr, "error: unable to request data from %s:\n", url);
        fprintf(stderr, "%s\n", curl_easy_strerror(status));
        return NULL;
    }

    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &code);
    if(code != 200)
    {
        fprintf(stderr, "error: server responded with code %ld\n", code);
        return NULL;
    }

    curl_easy_cleanup(curl);
    curl_global_cleanup();

    /* zero-terminate the result */
    data[write_result.pos] = '\0';

    return data;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv,"JSON_TAG");
	ros::NodeHandle node;
	ros::Rate loop_rate(200);

	size_t i;
	char *text;
	char url[URL_SIZE];
	snprintf(url, URL_SIZE, URL_FORMAT);

	json_t *root;
   	json_error_t error;
	
	//int First_time = 1;
	//clock_t start, end;
	ros::Publisher tag_msg = node.advertise<Lindsey::Tag>("Quuppa_raw", 1); 
	ros::Publisher time_msg = node.advertise<geometry_msgs::Vector3>("Json_timestep", 1); 

	//ros::Subscriber nav_sub;	
	//joy_sub = node.subscribe("joy", 1, joy_callback);
	
   	//ROS_INFO("Json tag Node");
 	while (ros::ok()) {


		//ROS_INFO("Loop Check");
		Lindsey::Tag QTags; //custom tag message
		text = request(url);

		if(!text){ 	return 1;	}
		
		root = json_loads(text, 0, &error);
		free(text);
		if(!root)
			{
				fprintf(stderr, "error: on line %d: %s\n", error.line, error.text);
				return 1;
			}
		if(!json_is_array(root))
			{
				fprintf(stderr, "error: root is not an array\n");
				return 1;
			}

//Set flag in custom message to size of array
int number_tags=json_array_size(root);
//printf("Number of tags: %i \n",number_tags);
QTags.NumTags = number_tags;

QTags.Id[number_tags];
QTags.Name[number_tags];
//QTags.TimeStamp[number_tags];
QTags.PosX[number_tags];
//QTags.PosX[1]=0.0;
QTags.PosY[number_tags];
//QTags.PosY[1]=0.0;
QTags.PosZ[number_tags];
//QTags.PosY[1]=0.0;
QTags.SmoPosX[number_tags];
QTags.SmoPosY[number_tags];
QTags.SmoPosZ[number_tags];



//start = clock(); //for timing
		 for(i = 0; i < json_array_size(root); i++)
		{
		    json_t *data, *positionY, *positionZ, *smoothedPositionZ, *smoothedPositionY, *smoothedPositionX, *id,*positionX, *name, *positionTimestampEpoch;

//json_t *areaId, *positionAccuracy,  *areaName, *color, *positionX, *name, *positionTimestamp;
			
		    const char *message_text;

		    data = json_array_get(root, i); //take ith slice of the data, in our case first tag
		    if(!json_is_object(data)) //check if its valid
		    {
		        ROS_ERROR("error: quuppa data %i is not an object\n", i + 1);
		        return 1;
		    }
			

			//start walking through each of the pieces of data, varaible = get object (current tag, "name of data in json format")
			//one must convert from hex to either number or string
			
			positionX = json_object_get(data, "positionX");
			positionY = json_object_get(data, "positionY");
			positionZ = json_object_get(data, "positionZ");
			smoothedPositionX = json_object_get(data, "smoothedPositionX");
			smoothedPositionY = json_object_get(data, "smoothedPositionY");
			smoothedPositionZ = json_object_get(data, "smoothedPositionZ");
			id = json_object_get(data, "id");
			name = json_object_get(data, "name");
			positionTimestampEpoch= json_object_get(data,"positionTimestampEpoc");
			//std::cout <<positionTimestampEpoch <<'\n';
			/*
			if(!json_is_real(positionY))
			message_text = json_string_value(name);
			printf("Name number: %f \n",json_number_value(name));
			printf("Name: %s \n",message_text);
			*/
message_text = json_string_value(id);		
QTags.Id.push_back(message_text);
message_text = json_string_value(name);
QTags.Name.push_back(message_text);
/*
//if (json_is_object(positionTimestampEpoch)) { ROS_INFO("Time is obj");}

//std::count << json_string_value(positionTimestampEpoch) << '\n';
if (json_integer_value(positionTimestampEpoch)) {ROS_INFO("Time is int");}
if (json_real_value(positionTimestampEpoch)) {ROS_INFO("Time is real");}
if (json_is_number(positionTimestampEpoch)) {ROS_INFO("Time is num");}
if (json_is_object(positionTimestampEpoch)) {ROS_INFO("Time is obj");}
if (json_is_string(positionTimestampEpoch)) {ROS_INFO("Time is str");}
if (json_is_null(positionTimestampEpoch)) {ROS_INFO("Time is null");}
QTags.TimeStamp.push_back(json_number_value(positionTimestampEpoch));
//ROS_INFO("Time raw %s",message_text.c_str());
*/
QTags.PosX.push_back(json_number_value(positionX));
QTags.PosY.push_back(json_number_value(positionY));
QTags.PosZ.push_back(json_number_value(positionZ));

QTags.SmoPosX.push_back(json_number_value(smoothedPositionX));
QTags.SmoPosY.push_back(json_number_value(smoothedPositionY));
QTags.SmoPosZ.push_back(json_number_value(smoothedPositionZ));

}

	json_decref(root); //free up the memory taken up by the root
	loop_times++;
	

//Check that the message is new
		
		if (0==(QTags.PosX[1]==PosX_old && QTags.PosY[1]==PosY_old && QTags.PosZ[1]==PosZ_old))//if new data, post it
				{
				
				
				PosX_old=QTags.PosX[1];
				PosY_old=QTags.PosY[1];
				PosZ_old=QTags.PosZ[1];
				tag_msg.publish(QTags);
					

		if(measure_time)
				{
				//timing stuff
				ros::Duration step=ros::Time::now()-Time_old;
				time_step.x=step.toSec();
				time_msg.publish(time_step);
				Time_old= ros::Time::now();
				}
					}	

		//if (DEBUG) {printf("Checked for Tags %i Times \n",loop_times);}
		
		//ros::spinOnce();
		//ROS_INFO("Loop Check End");
		loop_rate.sleep();

		}//ros::ok

ROS_ERROR("ROS::ok failed- Json Node Closing");
    return 0;
}//main
