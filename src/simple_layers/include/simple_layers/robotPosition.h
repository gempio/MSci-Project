#include <iostream>    //cout
#include <stdio.h> //printf
#include <string.h>    //strlen
#include <string>  //string
#include <sys/socket.h>    //socket
#include <arpa/inet.h> //inet_addr
#include <netdb.h> //hostent
#include <stdlib.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <simple_layers/grid_layer.h>

class robotPosition {
    int width, height;
  public:
  	robotPosition();
    void set_values (int,int);
    void set_GridLayer(simple_layer_namespace::GridLayer& grid);
    int area() {return width*height;}
};