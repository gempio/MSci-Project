#ifndef SOCIAL_COSTMAP_PLUGIN_H_
#define SOCIAL_COSTMAP_PLUGIN_H_
#include <ros/ros.h>
#include <costmap_2d/plugin_base.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>
#include <people_velocity_tracker/PersonPositionAndVelocity.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>
#include <stdlib.h>
#include <tf/tf.h>
#include <math.h>
#include <dynamic_reconfigure/server.h>
#include <social_costmap_plugin/SocialCostmapConfig.h>

double gaussian(double x, double y, double x0, double y0, double A, double varx, double vary, double skew){
    double dx = x-x0, dy = y-y0;
    double h = sqrt(dx*dx+dy*dy);
    double angle = atan2(dy,dx);
    double mx = cos(angle-skew) * h;
    double my = sin(angle-skew) * h;
    double f1 = pow(mx, 2.0)/(2.0 * varx), 
           f2 = pow(my, 2.0)/(2.0 * vary);
    return A * exp(-(f1 + f2));
}

double get_radius(double cutoff, double A, double var){
    return sqrt(-2*var * log(cutoff/A) );
}

namespace social_costmap_plugin
{
  class SocialCostmapPlugin : public costmap_2d::CostmapPlugin
  {
    public:
      SocialCostmapPlugin() { costmap_ = NULL; }

      void initialize(costmap_2d::Costmap2D* costmap, costmap_2d::Costmap2DROS* costmap_ros, ros::NodeHandle* nh)
      {
        costmap_ = costmap;
        costmap_ros_ = costmap_ros;
        nh_ = new ros::NodeHandle(*nh, "social_plugin");
        server = new dynamic_reconfigure::Server<SocialCostmapConfig>(*nh_);
        
        f = boost::bind(&SocialCostmapPlugin::configure, this, _1, _2);
        server->setCallback(f);

        people_sub_ = nh_->subscribe("/people", 1, &SocialCostmapPlugin::peopleCallback, this);
      }

      void peopleCallback(const people_velocity_tracker::PersonPositionAndVelocity& person) {
        boost::recursive_mutex::scoped_lock lock(lock_);
        if(people_list_.size()>0 && (*people_list_.begin()).header.stamp!=person.header.stamp)
          people_list_.clear();

        people_list_.push_front(person);
      }

        void applyChanges(){
                applyChanges(0,0,costmap_->getSizeInCellsX(),costmap_->getSizeInCellsY());
        }

        void applyChanges(double wx, double wy, double w_size_x, double w_size_y){
            int minx, miny, maxx, maxy;
            costmap_->worldToMapNoBounds(wx - w_size_x / 2, wy - w_size_y / 2, minx, miny);
            costmap_->worldToMapNoBounds(wx + w_size_x / 2, wy + w_size_y / 2, maxx, maxy);
            applyChanges(minx, miny, maxx+1, maxy+1);
        }

    void applyChanges(int minx, int miny, int maxx, int maxy){
        boost::recursive_mutex::scoped_lock lock(lock_);

        if( people_list_.size() == 0 )
          return;
        if( cutoff_ >= amplitude_)
            return;

        // clear old people
        ros::Duration time_diff = ros::Time::now() - (*people_list_.begin()).header.stamp;
        if(time_diff > people_keep_time_){
          people_list_.clear();
          return;
        }

        // application
        std::list<people_velocity_tracker::PersonPositionAndVelocity>::iterator p_it;
        std::string global_frame = costmap_ros_->getGlobalFrameID();

        for(p_it = people_list_.begin(); p_it != people_list_.end(); ++p_it){
            people_velocity_tracker::PersonPositionAndVelocity person = *p_it;
            geometry_msgs::PointStamped pt, opt;
            geometry_msgs::Point tpt, tv;

            try{
              pt.point.x = person.position.x;
              pt.point.y = person.position.y;
              pt.point.z = person.position.z;
              pt.header.frame_id = person.header.frame_id;
              tf_.transformPoint(global_frame, pt, opt);
              tpt = opt.point;
              pt.point.x += person.velocity.x;
              pt.point.y += person.velocity.y;
              pt.point.z += person.velocity.z;
              tf_.transformPoint(global_frame, pt, opt);
              tv = opt.point;
              tv.x -= tpt.x;
              tv.y -= tpt.y;
              tv.z -= tpt.z;
            }
            catch(tf::LookupException& ex) {
              ROS_ERROR("No Transform available Error: %s\n", ex.what());
              continue;
            }
            catch(tf::ConnectivityException& ex) {
              ROS_ERROR("Connectivity Error: %s\n", ex.what());
              continue;
            }
            catch(tf::ExtrapolationException& ex) {
              ROS_ERROR("Extrapolation Error: %s\n", ex.what());
              continue;
            }

            double res = costmap_ros_->getResolution();
            double angle = atan2(tv.y, tv.x);
            double mag = sqrt(pow(person.velocity.x,2) + pow(person.velocity.y, 2));
            double factor = 1.0 + mag * factor_;
            double base = get_radius(cutoff_, amplitude_, covar_);
            double point = get_radius(cutoff_, amplitude_, covar_ * factor );

            unsigned int width = std::max(1, int( (base + point) / res )),
                        height = std::max(1, int( (base + point) / res ));

            double ox, oy;
            if(sin(angle)>0)
                oy = tpt.y - base;
            else
                oy = tpt.y + (point-base) * sin(angle) - base;

            if(cos(angle)>=0)
                ox = tpt.x - base;
            else
                ox = tpt.x + (point-base) * cos(angle) - base;


            int dx, dy;
            costmap_->worldToMapNoBounds(ox, oy, dx, dy);

            int start_x = 0, start_y=0, end_x=width, end_y = height;
            if(dx < 0)
                start_x = -dx;
            else if(dx + width > costmap_->getSizeInCellsX())
                end_x = std::max(0, (int)costmap_->getSizeInCellsX() - dx);

            if((int)(start_x+dx) < minx)
                start_x = minx - dx;
            if((int)(end_x+dx) > maxx)
                end_x = maxx - dx;

            if(dy < 0)
                start_y = -dy;
            else if(dy + height > costmap_->getSizeInCellsY())
                end_y = std::max(0, (int) costmap_->getSizeInCellsY() - dy);

            if((int)(start_y+dy) < miny)
                start_y = miny - dy;
            if((int)(end_y+dy) > maxy)
                end_y = maxy - dy;

            double bx = ox + res / 2,
                   by = oy + res / 2;
            for(int i=start_x;i<end_x;i++){
                for(int j=start_y;j<end_y;j++){
                  unsigned char old_cost = costmap_->getCost(i+dx, j+dy);
                  if(old_cost == costmap_2d::NO_INFORMATION)
                    continue;

                  double x = bx+i*res, y = by+j*res;
                  double ma = atan2(y-tpt.y,x-tpt.x);
                  double diff = angles::shortest_angular_distance(angle, ma);
                  double a;
                  if(fabs(diff)<M_PI/2)
                      a = gaussian(x,y,tpt.x,tpt.y,amplitude_,covar_*factor,covar_,angle);
                  else
                      a = gaussian(x,y,tpt.x,tpt.y,amplitude_,covar_,       covar_,0);

                  if(a < cutoff_)
                    continue;
                  unsigned char cvalue = (unsigned char) a;
                  costmap_->setCost(i+dx, j+dy, std::max(cvalue, old_cost));

              }
            } 
            double minx, miny, maxx, maxy;
            costmap_->mapToWorld(start_x+dx, start_y+dy, minx, miny);
            costmap_->mapToWorld(end_x+dx, end_y+dy, maxx, maxy);
            costmap_->setModifiedBorders(minx, miny, maxx, maxy);
        }
    }

    void configure(SocialCostmapConfig &config, uint32_t level) {
        cutoff_ = config.cutoff;
        amplitude_ = config.amplitude;
        covar_ = config.covariance;
        factor_ = config.factor;
        people_keep_time_ = ros::Duration(config.keep_time);
    }

    private:
      ros::Subscriber people_sub_;
      std::list<people_velocity_tracker::PersonPositionAndVelocity> people_list_;
      double cutoff_, amplitude_, covar_, factor_;
      ros::Duration people_keep_time_;
      mutable boost::recursive_mutex lock_;
      tf::TransformListener tf_;
      dynamic_reconfigure::Server<SocialCostmapConfig>* server;
      dynamic_reconfigure::Server<SocialCostmapConfig>::CallbackType f;
  };
};
#endif

