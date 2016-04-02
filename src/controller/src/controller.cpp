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


int run_once;
double x;
double y;
int numberInput;
int numberInput2;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
pthread_t threads[2];

void *send_goal_1(void*) {

  float x []= {-2.67080068588, -2.735394945468, -2.8692850494, 3.11004161835, 2.61151981354, 8.74360466003, 9.0023651123, 9.08391475677};
  float y []= {4.69156122208, 1.44183540344, -2.96959543228, 4.85724782944, -2.75757598877, 5.43985700607, 1.45707416534, -2.5175409317};

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  float x_cord = x[numberInput];
  float y_cord = y[numberInput];
  goal.target_pose.pose.position.x = x_cord;
  goal.target_pose.pose.position.y = y_cord;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Robot 1 Success");
  else
    ROS_INFO("The base 1 failed to move forward 1 meter for some reason");
}

void *send_goal_2(void*) {
  float x []= {-2.67080068588, -2.735394945468, -2.8692850494, 3.11004161835, 2.61151981354, 8.74360466003, 9.0023651123, 9.08391475677};
  float y []= {4.69156122208, 1.44183540344, -2.96959543228, 4.85724782944, -2.75757598877, 5.43985700607, 1.45707416534, -2.5175409317};

  MoveBaseClient ac("/robot2/move_base", true);

  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "/robot2/map";
  goal.target_pose.header.stamp = ros::Time::now();
  float x_cord = x[numberInput];
  float y_cord = y[numberInput];
  goal.target_pose.pose.position.x = x_cord;
  goal.target_pose.pose.position.y = y_cord;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);


  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Robot 2 Success");
  else
    ROS_INFO("The base 2 failed to move forward 1 meter for some reason");
}

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

void runThread() {
  int rc;
  int i;
  if(numberInput2 == 0) {
    rc = pthread_create(&threads[numberInput2], NULL, 
                          send_goal_1, NULL);  
  } else { 
    rc = pthread_create(&threads[numberInput2], NULL, 
                          send_goal_2, NULL);  
  }
  
   
  if (rc){
     std::cout << "Error:unable to create thread," << rc << std::endl;
     exit(-1);
  }
  return;
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
  
  ros::init(argc, argv, "listener");
  
  
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  while (true){
      runQuestions();
      runThread();
  }
  
  
  

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}

