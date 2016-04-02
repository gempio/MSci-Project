/**
    C++ client example using sockets
*/
#include<iostream>    //cout
#include<stdio.h> //printf
#include<string.h>    //strlen
#include<string>  //string
#include<sys/socket.h>    //socket
#include<arpa/inet.h> //inet_addr
#include<netdb.h> //hostent
#include <stdlib.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <pthread.h>

using namespace std;
 
/**
    TCP Client class
*/
class tcp_client
{
private:
    int sock;
    std::string address;
    int port;
    struct sockaddr_in server;
     
public:
    tcp_client();
    bool conn(string, int);
    bool send_data(string data);
    string receive(int);
};
 
tcp_client::tcp_client()
{
    sock = -1;
    port = 0;
    address = "";
}
 
/**
    Connect to a host on a certain port number
*/
bool tcp_client::conn(string address , int port)
{
    //create socket if it is not already created
    if(sock == -1)
    {
        //Create socket
        sock = socket(AF_INET , SOCK_STREAM , 0);
        if (sock == -1)
        {
            perror("Could not create socket");
        }
         
        cout<<"Socket created\n";
    }
    else    {   /* OK , nothing */  }
     
    //setup address structure
    if(inet_addr(address.c_str()) == -1)
    {
        struct hostent *he;
        struct in_addr **addr_list;
         
        //resolve the hostname, its not an ip address
        if ( (he = gethostbyname( address.c_str() ) ) == NULL)
        {
            //gethostbyname failed
            herror("gethostbyname");
            cout<<"Failed to resolve hostname\n";
             
            return false;
        }
         
        //Cast the h_addr_list to in_addr , since h_addr_list also has the ip address in long format only
        addr_list = (struct in_addr **) he->h_addr_list;
 
        for(int i = 0; addr_list[i] != NULL; i++)
        {
            //strcpy(ip , inet_ntoa(*addr_list[i]) );
            server.sin_addr = *addr_list[i];
             
            cout<<address<<" resolved to "<<inet_ntoa(*addr_list[i])<<endl;
             
            break;
        }
    }
     
    //plain ip address
    else
    {
        server.sin_addr.s_addr = inet_addr( address.c_str() );
    }
     
    server.sin_family = AF_INET;
    server.sin_port = htons( port );
     
    //Connect to remote server
    if (connect(sock , (struct sockaddr *)&server , sizeof(server)) < 0)
    {
        perror("connect failed. Error");
        return 1;
    }
     
    cout<<"Connected\n";
    return true;
}
 
/**
    Send data to the connected host
*/
bool tcp_client::send_data(string data)
{
    //Send some data
    if( send(sock , data.c_str() , strlen( data.c_str() ) , 0) < 0)
    {
        perror("Send failed : ");
        return false;
    }
    cout<<data + "\nData sent.\n";
     
    return true;
}
 
/**
    Receive data from the connected host
*/
string tcp_client::receive(int size=512)
{
    char buffer[size];
    string reply;
     
    //Receive a reply from the server
    if( recv(sock , buffer , sizeof(buffer) , 0) < 0)
    {
        puts("recv failed");
    }
     
    reply = buffer;
    return reply;
}
void *run_controller(void*) {
    system("rosrun controller controller");
}

void *run_server(void*) {

    tcp_client c;
    string host;
    //system("roslaunch my_robot_name_2dnav launch_robot.launch");
    //connect to host
    c.conn("127.0.1.1" , 6009);
     
    //send some data
    c.send_data("%%setid SimR \r\n");

    while(true) {
         //receive and echo reply
        cout<<"----------------------------\n";
        string temp = c.receive(1024);
        if(temp.find("ping") != std::string::npos) {
            cout<<"PING\n";
            c.send_data("%%pong\n");
        } else if(temp.find("ack") != std::string::npos) {
            cout<<"ACKNOWLEDGED\n";
        }
        cout<<temp;
        cout<<"\n----------------------------\n";
     
    //done
    } 
   
    return 0;
}
 
int main(int argc , char *argv[])
{



    ros::init(argc, argv, "sender");
    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("sendRobots", 50);
    ros::Rate loop_rate(10);
    pthread_t temp;
    pthread_create(&temp, NULL, run_server, NULL);
    while (ros::ok())
    {
    /**
     * This is a message object. You stuff it with data, and then publish it.
         */
        std_msgs::String msg;
        string temp;

        std::cin.clear();
        std::cin.ignore(256,'\n');
        std::cin >> temp;
        
        msg.data = temp;
        chatter_pub.publish(msg);

        ROS_INFO("%s", msg.data.c_str());

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}