/*
Parker Conroy
Parcon Robotics

This code samples a JSON tag from a url
*/

#define URL_FORMAT   "www.kspresearch.com/docs/quuppaTag"
#define URL_SIZE     256
#define BUFFER_SIZE  (256*1024)  /* 256 KB */

#include <ros/ros.h>
#include <curl/curl.h>
#include <jansson.h>
#include <string.h>

/*
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Vector3.h>
#include <ardrone_autonomy/Navdata.h>
*/	

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
	ros::Rate loop_rate(100);

	size_t i;
	char *text;
	char url[URL_SIZE];
	snprintf(url, URL_SIZE, URL_FORMAT);

	json_t *root;
    json_error_t error;



	/*
	ros::Publisher pub_empty_reset;
	ros::Subscriber nav_sub;

	pub_twist = node.advertise<geometry_msgs::Twist>("cmd_vel", 1); 
	joy_sub = node.subscribe("joy", 1, joy_callback);
	*/
    ROS_INFO("Json tag Node");
 	while (ros::ok()) {

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
std::cout <<"size of array"<<std::endl;
std::cout <<json_array_size(root)<<std::endl;
		 for(i = 0; i < json_array_size(root); i++)
		{
		    json_t *data, *positionY, *positionZ, *smoothedPositionZ, *smoothedPositionY, *smoothedPositionX, *areaId, *positionAccuracy, *id, *areaName, *color, *positionX, *name, *positionTimestampEpoch, *positionTimestamp;
			
		    const char *message_text;

		    data = json_array_get(root, i);
		    if(!json_is_object(data))
		    {
		        fprintf(stderr, "error: quuppa data %d is not an object\n", i + 1);
		        return 1;
		    }

		    positionY = json_object_get(data, "positionY");
//		    if(!json_is_real(positionY))
			printf("PosY tag: %f \n",json_number_value(positionY));

			positionZ = json_object_get(data, "positionZ");
			printf("PosZ tag: %f \n",json_number_value(positionZ));

			areaId = json_object_get(data, "areaId");
            message = json_object_get(commit, "message");

		    if(!json_is_string(areaId))
		    {
		        fprintf(stderr, "error on %d: message is not a string\n", i + 1);
		        return 1;
		    }
	  

		    message_text = json_string_value(areaId);
		    printf("%.8s %.8s %.*s\n",
		           json_number_value(positionY),
					json_number_value(positionZ),
		           newline_offset(message_text),
		           message_text);
		}

	    json_decref(root);
	    return 0;

	
		}//ros::ok
ROS_ERROR("ROS::ok failed- Node Closing");
    return 0;
}//main
