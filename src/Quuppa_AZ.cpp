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
#include <ros/time.h>

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
    int sockfd;

    socklen_t slen=sizeof(cli_addr);
    char buf[BUFLEN];
	

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

		std::string name;
		std::string azm;
		std::string zen;
		int not_named_yet = 1;
		int not_azm_yet =0;
		int not_zen_yet =0;
		int tags_read = 0;

		for (int n=0; n<BUFLEN+1; n++){
			
			
				 if (  buf[n]==NULL ){
					
					
					Tags.Id.push_back(name);
					
					Tags.azm.push_back(strtod(azm.c_str(),NULL));
					
					Tags.zen.push_back(strtod(zen.c_str(),NULL));
					break;
					std::cout <<"Break"<<std::endl;
				}
				
		
				if ( ( buf[n]==',')  && ( (not_named_yet==1) && (not_azm_yet==0) && (not_zen_yet==0) ) )
					{// std::cout <<"Found 1,"<<std::endl;
					not_named_yet = 0;
					not_azm_yet =1;
					not_zen_yet =0;
					continue;}
				else if ( ( buf[n]==',')  && ( (not_named_yet==0) && (not_azm_yet==1) && (not_zen_yet==0) ) )
					{ //std::cout <<"Found 2,"<<std::endl;
					not_named_yet = 0;
					not_azm_yet =0;
					not_zen_yet =1;
					continue;}
				else if ( ( buf[n]==',')  && ( (not_named_yet==0) && (not_azm_yet==0) && (not_zen_yet==1) ) )
					{ //std::cout <<"Found 3,"<<std::endl;
					not_named_yet = 1;
					not_azm_yet =0;
					not_zen_yet =0;
										
					Tags.Id.push_back(name);
					Tags.azm.push_back(strtod(azm.c_str(),NULL));
					Tags.zen.push_back(strtod(zen.c_str(),NULL));
					
					tags_read++;
					name.clear();
					azm.clear();
					zen.clear();
					continue;
					}

					if (not_named_yet)	{name.push_back(buf[n]);}
					else if (not_azm_yet)	{azm.push_back(buf[n]);}
					else if (not_zen_yet)	{zen.push_back(buf[n]);}

				
	
		}
		std::cout <<"Done loopin"<<std::endl;	
		memset(&buf[0], 0, sizeof(buf));
		Tags.header.stamp = ros::Time::now();
		tag_msg.publish(Tags);
		Tags.Id.clear();
		Tags.azm.clear();
		Tags.zen.clear();
		loop_rate.sleep();
}
close(sockfd);

    return 0;
}
