#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

std::string getName(std::string temp, char **args,bool addLastNumber) {
    std::string y("/robot");
    y += args[1];
    y += temp;
    if(addLastNumber)y += +args[1];
    return y;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, getName("/pose_publisher",argv,true));
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  
  double publish_frequency;
  std::string map_frame, base_frame;
  ros::Publisher pose_publisher;
  
  private_nh.param<double>("publish_frequency", publish_frequency, 50);
  private_nh.param<std::string>(getName("/map_frame",argv,false), map_frame, getName("/map",argv,false));
  private_nh.param<std::string>(getName("/base_frame",argv,true), base_frame, getName("/base_link",argv,true));
  
  pose_publisher = nh.advertise<geometry_msgs::PoseStamped>(getName("/pose",argv,false), 50);
  
  tf::TransformListener listener;
  std::string tf_prefix = tf::getPrefixParam(private_nh);
  
  ros::Rate rate(publish_frequency);
  while(nh.ok()) {
    tf::StampedTransform transform;
    bool tf_ok = true;
    try {
      listener.lookupTransform(map_frame, base_frame, ros::Time(0), transform);
    } catch(tf::TransformException ex) {
      //ROS_ERROR("-------> %s", ex.what());
      tf_ok = false;
    }
    
    if(tf_ok) {
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header.stamp = ros::Time::now();
      pose_stamped.header.frame_id = tf_prefix+"/"+map_frame;
      
      pose_stamped.pose.position.x = transform.getOrigin().getX();
      pose_stamped.pose.position.y = transform.getOrigin().getY();
      pose_stamped.pose.position.z = transform.getOrigin().getZ();
      
      pose_stamped.pose.orientation.x = transform.getRotation().getX();
      pose_stamped.pose.orientation.y = transform.getRotation().getY();
      pose_stamped.pose.orientation.z = transform.getRotation().getZ();
      pose_stamped.pose.orientation.w = transform.getRotation().getW();
      
      pose_publisher.publish(pose_stamped);
    }
    
    rate.sleep();
  }
  
  return 0;
}