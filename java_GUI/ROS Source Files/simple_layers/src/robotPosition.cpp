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
#include <simple_layers/robotPosition.h>

void robotPosition::set_values (int x, int y) {
  width = x;
  height = y;
}

void robotPosition::set_GridLayer(simple_layer_namespace::GridLayer& grid) {
    // std::cout << &grid;
    grid.x_coord = 0.0;
}

robotPosition::robotPosition() {
    std::cout << "Created";
}
int main() {

}