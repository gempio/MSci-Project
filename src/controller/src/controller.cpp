#include <string>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <pthread.h>
#include <vector>
#include <sstream>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

int run_once;
double x;
double y;
int numberInput;
int numberInput2;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
pthread_t threads[2];
bool robotSuccess;
std::string tempo;
int robot[2];

void *send_goal(void*) {
  int temp = numberInput2;
  float x []= {-2.67080068588, -2.735394945468, -2.8692850494, 3.11004161835, 2.61151981354, 8.74360466003, 9.0023651123, 9.08391475677};
  float y []= {4.69156122208, 1.44183540344, -2.96959543228, 4.85724782944, -2.75757598877, 5.43985700607, 1.45707416534, -2.5175409317};
  
  //tell the action client that we want to spin a thread by default
  std::ostringstream oss;
  oss << "robot" << (numberInput2+1) << "/move_base";
  std::cout << oss.str();
  MoveBaseClient ac(oss.str(), true);
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  //we'll send a goal to the robot.
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  float x_cord = x[numberInput];
  float y_cord = y[numberInput];
  robot[numberInput2] = numberInput;
  goal.target_pose.pose.position.x = x_cord;
  goal.target_pose.pose.position.y = y_cord;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    std::stringstream sstm;
    sstm << "%%error SimR TabUI \"" << temp << ";" << robot[temp] << "\"";
    tempo = sstm.str();
    robotSuccess = true;
  } else
    ROS_INFO("The base 1 failed to move forward 1 meter for some reason");
}
//Debuggin method without using the server.
void runQuestions() {
  std::cout << "Please enter a room number from 0 to 7: ";
  std::cin >> numberInput;

  while(std::cin.fail() || numberInput<-1 || numberInput>8) {
        std::cout << "Incorrect value. Please enter a room number from 0 to 7: " << std::endl;
        std::cin.clear();
        std::cin.ignore(256,'\n');
        std::cin >> numberInput;
  }
  if(numberInput == -1) {
    exit(0);
  }
  std::cout << "Please which robot should go?: ";
  std::cin >> numberInput2;
  
  while(std::cin.fail() || numberInput2<-1 || numberInput2>8) {
        std::cout << "Incorrect value. Please enter a robot number from 0 to 1: " << std::endl;
        std::cin.clear();
        std::cin.ignore(256,'\n');
        std::cin >> numberInput2;
  }
  if(numberInput2 == -1) {
    exit(0);
  }
  return;

}
//Initialize the therad to ensure we can get 
void runThread() {
  int rc;
  int i;
  rc = pthread_create(&threads[numberInput2], NULL, 
                          send_goal, NULL);  
  
  if (rc){
     std::cout << "Error:unable to create thread," << rc << std::endl;
     exit(-1);
  }
  return;
}

//Splitter methods for C++.
std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}


std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

//Command processor.
void processCommand(const std_msgs::String::ConstPtr& msg)
{
  using namespace boost::algorithm;

  ROS_INFO("I heard: %s", msg->data.c_str());
  std::string temp2 = msg->data.c_str();
  std::vector<std::string> tokens;

  split(tokens, temp2, is_any_of(" ")); // here it is

  std::cout << (tokens.at(3) + "\n");
  std::cout << (tokens.at(4) + "\n");

  numberInput2 = atoi(tokens.at(3).c_str());
  numberInput = atoi(tokens.at(4).c_str());
  runThread();
  //std::cout << ("Done Converting data" + "\n");
}

int main(int argc, char **argv)
{

    
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  
  ros::init(argc, argv, "controller");
  
  
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  tempo = "";
  robot[0] = 0;
  robot[1] = 0;
  robotSuccess = false;
  ros::Subscriber sub = n.subscribe("sendRobots", 50, processCommand);
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("success", 50);
  ros::Rate loop_rate(10);
  while (ros::ok())
    {
    /**
     * This is a message object. You stuff it with data, and then publish it.
         */
        if(robotSuccess) {
            std_msgs::String msg;
            
            msg.data = tempo;
            chatter_pub.publish(msg);

            ROS_INFO("%s", msg.data.c_str());
            robotSuccess = !robotSuccess;
            
        }
        ros::spinOnce();

        loop_rate.sleep();
    }

  ros::spin();
  
  


  return 0;
}

