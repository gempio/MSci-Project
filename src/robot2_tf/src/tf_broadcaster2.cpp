#include <ros/ros.h>
#include <tf/transform_broadcaster.h>


std::string getName(std::string temp, char **args,bool addLastNumber) {
    std::string y("/robot");
    y += args[1];
    y += temp;
    if(addLastNumber)y += +args[1];
    return y;
}

int main(int argc, char** argv){
  ros::init(argc, argv, getName("/robot_tf_publisher",argv,true));
  ros::NodeHandle n;

  ros::Rate r(0.1);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
        ros::Time::now(),getName("/base_link",argv,true), getName("/base_laser",argv,true)));
    r.sleep();



  }
}