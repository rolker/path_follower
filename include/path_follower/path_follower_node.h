/** 
 * @file path_follower_node.h
 * 
 * Discription goes here.
 * 
 * Local ROS Parameters:
 *  - map_frame: 
 *  - base_frame: 
 *  - dynamics_mode: str: one of "unicycle" or "holonomic".  
 *      Default is "unicycle"
 * 
 * TODO: Create header file and organize private/public attributes.
 * 
 * Subscribes:
 *  - 
 *  -
 * 
 * Publishes:
 * 
 *
 */
#ifndef PATH_FOLLOWER_NODE_H_
#define PATH_FOLLOWER_NODE_H_

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
  
  PathFollower(std::string name);
  ~PathFollower();
  void goalCallback();
  void preemptCallback();
  void sendDisplay();
  void controlEfforCallback(const std_msgs::Float64::ConstPtr& inmsg);
  void timerCallback(const ros::TimerEvent event);
    
private:
  actionlib::SimpleActionServer<path_follower::path_followerAction>
    m_action_server;

  std::vector<geometry_msgs::PoseStamped> m_goal_path;

  std::vector<AzimuthDistance> m_segment_azimuth_distances;

  int m_current_segment_index;

  double m_goal_speed;

  p11::AngleRadians m_crab_angle;
    
  double m_total_distance; // total distance of complete path in meters.
  double m_cumulative_distance; // distance of completed segments in meters.

  bool m_enabled;
  ros::Subscriber m_enable_sub;
  bool m_send_display_flag;
    
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
  
  // Dynamics mode
  enum DynamicsMode { unicycle, holonomic };
  DynamicsMode m_dynamics_mode;
};

#endif
