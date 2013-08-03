//UDPServer.c

/* 
 *  gcc -o server UDPServer.c
 *  ./server
 */
#include <ros/ros.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdlib.h> 
#include <string.h>
#include <Lindsey/AzTag.h>
#include <boost/algorithm/string.hpp>
#include <vector>

#define BUFLEN 512
#define PORT 22102

void err(char *str)
{
    perror(str);
    exit(1);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv,"QUUPPA_AZ_NODE");
	ros::NodeHandle node;
	ros::Rate loop_rate(200);
	ros::Publisher tag_msg = node.advertise<Lindsey::AzTag>("Quuppa_AZ", 1); 

	Lindsey::AzTag Tags;
	int number_tags =1;
	Tags.NumTags = number_tags;
	Tags.Id[number_tags];
	Tags.azm[number_tags];
	Tags.zen[number_tags];

typedef std::vector< std::string > split_vector_type;
split_vector_type fields;

    struct sockaddr_in my_addr, cli_addr;
    int sockfd, i; 
    socklen_t slen=sizeof(cli_addr);
    char buf[BUFLEN];
char * pEnd;
	std::string str;

    if ((sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
      err("socket");
    else 
      printf("Server : Socket() successful\n");

    bzero(&my_addr, sizeof(my_addr));
    my_addr.sin_family = AF_INET;
    my_addr.sin_port = htons(PORT);
    my_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    
    if (bind(sockfd, (struct sockaddr* ) &my_addr, sizeof(my_addr))==-1)
      err("bind");
    else
      printf("Server : bind() successful\n");

    while(ros::ok())
    {
        if (recvfrom(sockfd, buf, BUFLEN, 0, (struct sockaddr*)&cli_addr, &slen)==-1)
            err("recvfrom()");
        printf("Received packet from %s:%d\nData: %s\n\n",inet_ntoa(cli_addr.sin_addr), ntohs(cli_addr.sin_port), buf);
    		//char buf_name[BUFLEN];
		int size = BUFLEN;
		str.assign(buf);
		std::cout <<str<<std::endl;
		//split incomming string
		boost::split( fields, str, boost::is_any_of( "," ) );
		printf("split");
		Tags.Id[0]=fields[0];
		const char *cstr = fields[1].c_str();
		Tags.azm[0]=strtod(cstr,NULL);
		cstr = fields[1].c_str();
		Tags.zen[0]=strtod(cstr,NULL);
		tag_msg.publish(Tags);
loop_rate.sleep();
}

    close(sockfd);
    return 0;
}
