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


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
int run_once;
double x;
double y;
int numberInput;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void poseAdjustment(const geometry_msgs::Twist & velocity) {
  ROS_INFO("Recieved a /cmd_vel message!");
  ROS_INFO("Linear components: [%.2f, %.2f. %.2f,%.2f, %.2f. %.2f]", velocity.linear.x,velocity.linear.y,velocity.linear.z,velocity.angular.x,velocity.angular.y,velocity.angular.z);
}

void send_goal() {
  float x []= {-2.67080068588, -2.735394945468, -2.8692850494, 3.11004161835, 2.61151981354, 8.74360466003, 9.0023651123, 9.08391475677};
  float y []= {4.69156122208, 1.44183540344, -2.96959543228, 4.85724782944, -2.75757598877, 5.43985700607, 1.45707416534, -2.5175409317};

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("robot2/move_base", true);
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "robot2/map";
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
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");
}

void get_room() {
  ROS_INFO("Info run once");
  std::cout << "Please enter a room number from 0 to 7: ";
  std::cin >> numberInput;
  while(std::cin.fail() || numberInput<0 || numberInput>8) {
        std::cout << "Incorrect value. Please enter a room number from 0 to 7: " << std::endl;
        std::cin.clear();
        std::cin.ignore(256,'\n');
        std::cin >> numberInput;
  }
  send_goal();
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
  
  get_room();
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

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
  ros::Subscriber sub = n.subscribe("cmd_vel", 50, poseAdjustment);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}

