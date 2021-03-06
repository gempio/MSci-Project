#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GridCells.h>
#include "std_msgs/String.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <costmap_2d/costmap_2d_ros.h>

// LayeredCostmap* layered_costmap_;
double x;
double y;
double th;
double linear_x;
double linear_y;
double linear_z;
double angular_x;
double angular_y;
double angular_z;

bool publish_transform;
ros::Publisher odom_pub;
char **args;

// void updateCostCostMap() {
//   costmap_2d::getCostmap();

// }
void poseCallBack(const geometry_msgs::PoseWithCovarianceStamped & pose )
  {
    //ROS_INFO("I Heard some shit: [%.2f, %.2f. %.2f]",pose.pose.pose.position.x,pose.pose.pose.position.y,pose.pose.pose.position.z);
    //ROS_INFO("I heard: [%s]", msg->data.c_str());
    x = pose.pose.pose.position.x;
    y = pose.pose.pose.position.y;
    //th = pose.pose.pose.position.z;
    publish_transform = true;
  }


void poseAdjustment(const geometry_msgs::Twist & velocity) {
  //ROS_INFO("Recieved a /cmd_vel message!");
  //ROS_INFO("Linear components: [%.2f, %.2f. %.2f,%.2f, %.2f. %.2f]", velocity.linear.x,velocity.linear.y,velocity.linear.z,velocity.angular.x,velocity.angular.y,velocity.angular.z);
  //x = velocity.linear.x;
  //y = velocity.linear.y;
  //th = 1.0;

  linear_x = velocity.linear.x;
  linear_y = velocity.linear.y;
  angular_z = velocity.angular.z;
  publish_transform = true;


}

std::string getName(std::string temp, char **args,bool addLastNumber) {
    std::string y("/robot");
    y += args[1];
    y += temp;
    if(addLastNumber)y += +args[1];
    return y;
}

int main(int argc, char** argv) {

  ros::init(argc, argv, getName("state_publisher", argv, true));
  ros::NodeHandle n;
  odom_pub = n.advertise<nav_msgs::Odometry>(getName("/odom",argv,true), 50);
  args = argv;
  int32_t publish_rate_ = 50;
  tf::TransformBroadcaster tf_br_;
  tf::StampedTransform tf_map_to_odom_;

  // set up parent and child frames
  tf_map_to_odom_.frame_id_ = std::string("/map");
  tf_map_to_odom_.child_frame_id_ = std::string(getName("/odom",argv,true));

  tf::StampedTransform tf_footprint_to_base_;

  // set up parent and child frames
  tf_footprint_to_base_.frame_id_ = std::string(getName("/base_footprint",argv,true));
  tf_footprint_to_base_.child_frame_id_ = std::string(getName("/base_link",argv,true));

  tf::StampedTransform tf_laser_to_frame_;

  // set up parent and child frames
  tf_laser_to_frame_.frame_id_ = std::string(getName("/base_laser", argv, true));
  tf_laser_to_frame_.child_frame_id_ = std::string(getName("/laser_frame",argv,true));

  //publishing the first position
  publish_transform = true;

  // initial position
  x = 0.0; 
  y = 0.0;
  th = 0;

  // velocity
  linear_x = 0.0;
  linear_y = 0.0;
  angular_z = 0.0;

  ros::Time current_time;
  ros::Time last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  tf::TransformBroadcaster broadcaster;
  ros::Rate loop_rate(50);

  const double degree = M_PI/180;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 50);
  // message declarations
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.frame_id = getName("/odom", argv, true);
  odom_trans.child_frame_id = getName("/base_footprint", argv, true);

  ros::Subscriber sub = n.subscribe(getName("/initialpose",argv,false), 50, poseCallBack);
  ros::Subscriber sub2 = n.subscribe(getName("/cmd_vel",argv,false), 50, poseAdjustment);
  int count = 0;
  while (ros::ok()) {

    // time stamp
    tf_map_to_odom_.stamp_ = ros::Time::now();

    // specify actual transformation vectors from odometry
    // NOTE: zeros have to be substituted with actual variable data
    tf_map_to_odom_.setOrigin(tf::Vector3(0.0f, 0.0f, 0.0f));
    tf_map_to_odom_.setRotation(tf::Quaternion(0.0f, 0.0f, 0.0f));

    // broadcast transform
    tf_br_.sendTransform(tf_map_to_odom_);

    // time stamp
    tf_footprint_to_base_.stamp_ = ros::Time::now();

    // specify actual transformation vectors from odometry
    // NOTE: zeros have to be substituted with actual variable data
    tf_footprint_to_base_.setOrigin(tf::Vector3(0.0f, 0.0f, 0.0f));
    tf_footprint_to_base_.setRotation(tf::Quaternion(0.0f, 0.0f, 0.0f));

    // broadcast transform
    tf_br_.sendTransform(tf_footprint_to_base_);

    // time stamp
    tf_laser_to_frame_.stamp_ = ros::Time::now();

    // specify actual transformation vectors from odometry
    // NOTE: zeros have to be substituted with actual variable data
    tf_laser_to_frame_.setOrigin(tf::Vector3(0.0f, 0.0f, 0.0f));
    tf_laser_to_frame_.setRotation(tf::Quaternion(0.0f, 0.0f, 0.0f));

    // broadcast transform
    tf_br_.sendTransform(tf_laser_to_frame_);

    current_time = ros::Time::now(); 

    double dt = (current_time - last_time).toSec();
    double delta_x = (linear_x * cos(th) - linear_y * sin(th)) * dt;
    double delta_y = (linear_x * sin(th) + linear_y * cos(th)) * dt;
    double delta_th = angular_z * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    geometry_msgs::Quaternion odom_quat;  
    odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,th);

    // update transform
    odom_trans.header.stamp = current_time; 
    odom_trans.transform.translation.x = x; 
    odom_trans.transform.translation.y = y; 
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);

    //filling the odometry
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = getName("/odom", argv, true);
    odom.child_frame_id = getName("/base_footprint",argv,true);

    // position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = th;
    odom.pose.pose.orientation = odom_quat;

    //velocity
    odom.twist.twist.linear.x = linear_x;
    odom.twist.twist.linear.y = linear_y;
    odom.twist.twist.linear.z = linear_z;
    odom.twist.twist.angular.x = angular_x;
    odom.twist.twist.angular.y = angular_y;
    odom.twist.twist.angular.z = angular_z;

    last_time = current_time;

    // publishing the odometry and the new tf
    if(publish_transform) {
      odom_pub.publish(odom);
      
    }
    
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
    chatter_pub.publish(msg);

    ++count;
    //publish_transform = false;
    broadcaster.sendTransform(odom_trans);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}