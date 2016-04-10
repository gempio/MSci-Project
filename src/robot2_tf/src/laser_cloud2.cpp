#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>


std::string getName(std::string temp, char **args,bool addLastNumber) {
    std::string y("/robot");
    y += args[1];
    y += temp;
    if(addLastNumber)y += +args[1];
    return y;
}

int main(int argc, char** argv){
  ros::init(argc, argv, getName("/point_cloud_publisher",argv,true));

  ros::NodeHandle n;
  ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud>(getName("/cloud",argv,true), 50);

  unsigned int num_points = 100;

  int count = 0;
  ros::Rate r(0.2);
  while(n.ok()){
    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = getName("/base_link",argv,true);

    cloud.points.resize(num_points);

    //we'll also add an intensity channel to the cloud
    cloud.channels.resize(1);
    cloud.channels[0].name = "intensities";
    cloud.channels[0].values.resize(num_points);

    //generate some fake data for our point cloud
    for(unsigned int i = 0; i < num_points; ++i){
      cloud.points[i].x = 1 + count;
      cloud.points[i].y = 2 + count;
      cloud.points[i].z = 3 + count;
      cloud.channels[0].values[i] = 100 + count;
    }

    cloud_pub.publish(cloud);
    ++count;
    r.sleep();
  }
}
