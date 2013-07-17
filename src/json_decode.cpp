/*
Parker Conroy
Parcon Robotics

This code samples a JSON tag from a url
you may need to link the library location after install via
$ ln -s /usr/local/lib/libjansson.so.4 /usr/lib/libjansson.so.4
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

		    data = json_array_get(root, i); //take ith slice of the data, in our case first tag
		    if(!json_is_object(data)) //check if its valid
		    {
		        fprintf(stderr, "error: quuppa data %i is not an object\n", i + 1);
		        return 1;
		    }
			printf("Start of tag \n");

			//start walking through each of the pieces of data, varaible = get object (current tag, "name of data in json format")
			//one must convert from hex to either number or string

		    positionY = json_object_get(data, "positionY");
//		    if(!json_is_real(positionY))
			printf("PosY tag: %f \n",json_number_value(positionY));

			positionZ = json_object_get(data, "positionZ");
			printf("PosZ tag: %f \n",json_number_value(positionZ));


			smoothedPositionZ = json_object_get(data, "smoothedPositionZ");
			printf("SmoPosZ tag: %f \n",json_number_value(smoothedPositionZ));
			
			smoothedPositionY = json_object_get(data, "smoothedPositionY");
			printf("SmoPosy tag: %f \n",json_number_value(smoothedPositionY));
			
			smoothedPositionX = json_object_get(data, "smoothedPositionX");
			printf("SmoPosX tag: %f \n",json_number_value(smoothedPositionX));
			
			areaId = json_object_get(data, "areaId");
		    message_text = json_string_value(areaId);
			printf("areaId : %s \n",message_text);
	   		
			printf("End of tag \n");
			printf("------------\n");
}
	json_decref(root); //free up the memory taken up by the root
	printf("End of column");
	printf("===========");
	return 1;
		}//ros::ok
ROS_ERROR("ROS::ok failed- Node Closing");
    return 0;
}//main
