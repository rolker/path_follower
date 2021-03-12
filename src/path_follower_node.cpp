#include <vector>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/TwistStamped.h"
#include "geographic_visualization_msgs/GeoVizItem.h"
#include "path_follower/path_followerAction.h"
#include "actionlib/server/simple_action_server.h"
#include "project11/utils.h"
#include "project11/tf2_utils.h"

namespace p11 = project11;

class PathFollower
{
public:
  struct AzimuthDistance
  {
    p11::AngleRadians azimuth;
    double distance;
  };
  
    PathFollower(std::string name):
      m_action_server(ros::NodeHandle(), name, false),m_send_display_flag(true)
    {
        m_goal_speed = 0.0;
        m_crab_angle = 0.0;
        m_current_segment_index = 0;
        m_total_distance = 0.0;
        m_cumulative_distance = 0.0;
    
        ros::NodeHandle nh;
        
        m_enabled = true;
        m_enable_sub = nh.subscribe<std_msgs::Bool>("enable", 10, [&](const std_msgs::BoolConstPtr& msg){this->m_enabled = msg->data; this->sendDisplay();});
        
        m_cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("cmd_vel",1);

        m_display_pub = nh.advertise<geographic_visualization_msgs::GeoVizItem>("project11/display",5);
        m_vis_display.id = "path_follower";

        ros::NodeHandle nh_private("~");
        
        nh_private.param<std::string>("map_frame", m_map_frame, "map");
        nh_private.param<std::string>("base_frame", m_base_frame, "base_link");

        
        m_setpoint_pub = nh_private.advertise<std_msgs::Float64>("crab_angle_pid/setpoint",1, true);
        m_state_pub = nh_private.advertise<std_msgs::Float64>("crab_angle_pid/state",1);
        m_control_effort_sub = nh_private.subscribe("crab_angle_pid/control_effort", 10, &PathFollower::controlEfforCallback, this);
        m_pid_enable_pub = nh_private.advertise<std_msgs::Bool>("crab_angle_pid/pid_enable", 1);
        
        std_msgs::Float64 setpoint;
        setpoint.data = 0.0;
        m_setpoint_pub.publish(setpoint);

        m_action_server.registerGoalCallback(boost::bind(&PathFollower::goalCallback, this));
        m_action_server.registerPreemptCallback(boost::bind(&PathFollower::preemptCallback, this));
        m_action_server.start();
        
        m_timer = nh.createTimer(ros::Duration(0.1), std::bind(&PathFollower::timerCallback, this, std::placeholders::_1));
    }
    
    ~PathFollower()
    {
    }
    
    void goalCallback()
    {
        auto goal = m_action_server.acceptNewGoal();

        m_goal_speed = goal->speed;
        m_current_segment_index = 0;
        m_total_distance = 0.0;
        m_cumulative_distance = 0.0;

        m_goal_path.clear();
        
        geometry_msgs::TransformStamped earth_to_map = m_transforms().lookupTransform(m_map_frame, "earth", ros::Time(0));

        geographic_visualization_msgs::GeoVizPointList gvpl;
        
        for(auto pose: goal->path.poses)
        {
          p11::LatLongDegrees p;
          p11::fromMsg(pose.pose.position, p);
          p11::ECEF ecef_point = p;
          geometry_msgs::Point ecef_point_msg;
          p11::toMsg(ecef_point,ecef_point_msg);
          geometry_msgs::PoseStamped map_point;
          tf2::doTransform(ecef_point_msg, map_point.pose.position, earth_to_map);
          map_point.header.frame_id = m_map_frame;
          map_point.header.stamp = pose.header.stamp;
          m_goal_path.push_back(map_point);
          gvpl.points.push_back(pose.pose.position);
        }
        m_vis_display.lines.clear();
        m_vis_display.lines.push_back(gvpl);
        
        m_segment_azimuth_distances.clear();
        for(int i = 0; i < m_goal_path.size()-1; i++)
        {
          AzimuthDistance ad;
          double dx = m_goal_path[i+1].pose.position.x - m_goal_path[i].pose.position.x;
          double dy = m_goal_path[i+1].pose.position.y - m_goal_path[i].pose.position.y;
          ad.azimuth = atan2(dy,dx);
          ad.distance = sqrt(dx*dx+dy*dy);
          m_total_distance += ad.distance;
          m_segment_azimuth_distances.push_back(ad);
        }
        m_send_display_flag = true;
    }
    
    void sendDisplay()
    {
      if(!m_vis_display.lines.empty())
      {
        auto& plist = m_vis_display.lines.front();
        plist.size = 5.0;
        if (m_enabled)
        {
            plist.color.r = 1.0;
            plist.color.g = 0.0;
            plist.color.b = 0.0;
            plist.color.a = 1.0;
        }
        else
        {
            plist.color.r = 0.5;
            plist.color.g = 0.0;
            plist.color.b = 0.0;
            plist.color.a = 0.5;
        }
      }
      m_display_pub.publish(m_vis_display);
      m_send_display_flag = false;
    }

    void preemptCallback()
    {
        m_action_server.setPreempted();
        m_vis_display.lines.clear();
        m_send_display_flag = true;
    }
    
    void controlEfforCallback(const std_msgs::Float64::ConstPtr& inmsg)
    {
        m_crab_angle = p11::AngleDegrees(inmsg->data);
    }

    void timerCallback(const ros::TimerEvent event)
    {
      if(m_send_display_flag)
        sendDisplay();
        if(m_action_server.isActive() && m_enabled && !m_goal_path.empty())
        {
          geometry_msgs::TransformStamped base_to_map;
          try
          {
            base_to_map = m_transforms().lookupTransform(m_map_frame, m_base_frame, ros::Time(0));
          }
          catch (tf2::TransformException &ex)
          {
            ROS_WARN_STREAM("timerCallback: " << ex.what());
          }

          double vehicle_distance;
            
          p11::AngleRadians error_azimuth;
          double sin_error_azimuth;
          double cos_error_azimuth;
          double progress;

          bool found_current_segment = false;
            
          while(!found_current_segment)
          {
            double dx = m_goal_path[m_current_segment_index].pose.position.x - base_to_map.transform.translation.x;
            double dy = m_goal_path[m_current_segment_index].pose.position.y - base_to_map.transform.translation.y;
            vehicle_distance = sqrt(dx*dx+dy*dy);

            // angle from p to vehicle
            p11::AngleRadians azimuth = atan2(-dy, -dx);
            
            error_azimuth = azimuth - m_segment_azimuth_distances[m_current_segment_index].azimuth;

            sin_error_azimuth = sin(error_azimuth);
            cos_error_azimuth = cos(error_azimuth);

            progress = vehicle_distance*cos_error_azimuth;
            if (progress >= m_segment_azimuth_distances[m_current_segment_index].distance)
            {
              m_cumulative_distance += m_segment_azimuth_distances[m_current_segment_index].distance;
              m_current_segment_index += 1;
              // have we reached the last segment?
              if(m_current_segment_index >= m_goal_path.size()-1)
              {
                path_follower::path_followerResult result;
                result.ending_pose.position = m_transforms.map_to_wgs84(geometry_msgs::Point(), m_base_frame);
                m_action_server.setSucceeded(result);
                sendDisplay();
                return;
              }
            }
            else
            {
              found_current_segment = true;
            }
          }
            
          double cross_track = vehicle_distance*sin_error_azimuth;
            
          path_follower::path_followerFeedback feedback;
          feedback.percent_complete = (m_cumulative_distance+progress)/m_total_distance;
          feedback.crosstrack_error = cross_track;
          m_action_server.publishFeedback(feedback);
            
          std_msgs::Bool pid_enable;
          pid_enable.data = true;
          m_pid_enable_pub.publish(pid_enable);
            
          std_msgs::Float64 setpoint;
          setpoint.data = 0.0;
          m_setpoint_pub.publish(setpoint);
            
          std_msgs::Float64 state;
          state.data = cross_track;
          m_state_pub.publish(state);
            
            
          ros::Time now = ros::Time::now();
          geometry_msgs::TwistStamped ts;
          ts.header.frame_id = m_base_frame;
          ts.header.stamp = event.current_real;
          
          p11::AngleRadians heading = tf2::getYaw(base_to_map.transform.rotation);
          p11::AngleRadians target_heading = m_segment_azimuth_distances[m_current_segment_index].azimuth + m_crab_angle;

          ts.twist.angular.z = p11::AngleRadiansZeroCentered(target_heading-heading).value();
          ts.twist.linear.x = m_goal_speed;

          m_cmd_vel_pub.publish(ts);
        }
        else
        {
            std_msgs::Bool pid_enable;
            pid_enable.data = false;
            m_pid_enable_pub.publish(pid_enable);
        }
    }
    
private:
    actionlib::SimpleActionServer<path_follower::path_followerAction> m_action_server;

    std::vector<geometry_msgs::PoseStamped> m_goal_path;
    std::vector<AzimuthDistance> m_segment_azimuth_distances;
    int m_current_segment_index;

    double m_goal_speed;

    p11::AngleRadians m_crab_angle;
    
    double m_total_distance; // total distance of complete path in meters.
    double m_cumulative_distance; // distance of completed segments in meters.

    bool m_enabled;
    ros::Subscriber m_enable_sub;
    
    // output
    ros::Publisher m_cmd_vel_pub;

    // pid topics
    ros::Publisher m_state_pub;
    ros::Publisher m_setpoint_pub;
    ros::Subscriber m_control_effort_sub;
    ros::Publisher m_pid_enable_pub;

    // display
    ros::Publisher m_display_pub;
    geographic_visualization_msgs::GeoVizItem m_vis_display;


    // tf frames
    std::string m_map_frame;
    std::string m_base_frame;
    
    p11::Transformations m_transforms;
    
    ros::Timer m_timer;
    bool m_send_display_flag;
};





int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_follower");

    PathFollower pf("path_follower_action");
    
    ros::spin();
    
    return 0;
}

    
