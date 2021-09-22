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
#include <cmath>

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
  
  PathFollower(std::string name);
  ~PathFollower();
  void goalCallback();
  void preemptCallback();
  void controlEfforCallback(const std_msgs::Float64::ConstPtr& inmsg);
  
    
private:
  // Dynamics mode
  enum DynamicsMode { unicycle, holonomic };
  DynamicsMode m_dynamics_mode;

  /**
   * @brief Converts string (from ROS parameter) to mode enumeration.
   */
  PathFollower::DynamicsMode str2dynamicsmode(std::string str);

  void sendDisplay();

  void timerCallback(const ros::TimerEvent event);

  /** @brief Holds the path bearing [rad, ENU] and distance [m] values. **/
  struct AzimuthDistance
  {
    p11::AngleRadians azimuth;
    double distance;
  };
  actionlib::SimpleActionServer<path_follower::path_followerAction>
    m_action_server;

  std::vector<geometry_msgs::PoseStamped> m_goal_path;

  std::vector<AzimuthDistance> m_segment_azimuth_distances;


  float m_kp_yaw;
  float m_kp_surge;
  float m_kp_sway;
  bool m_turn_in_place;
  float m_turn_in_place_threshold;
  
  int m_current_segment_index;

  double m_goal_speed;

  p11::AngleRadians m_crab_angle;
    
  double m_total_distance; // total distance of complete path in meters.
  double m_cumulative_distance; // distance of completed segments in meters.

  bool m_enabled;
  ros::Subscriber m_enable_sub;
  bool m_send_display_flag;
    
  // Pub
  ros::Publisher m_cmd_vel_pub;
  
  // PID pub/subs
  ros::Publisher m_state_pub;
  ros::Publisher m_setpoint_pub;
  ros::Subscriber m_control_effort_sub;
  ros::Publisher m_pid_enable_pub;
  
  // display
  ros::Publisher m_display_pub;
  geographic_visualization_msgs::GeoVizItem m_vis_display;
  ros::Time m_last_display_send_time;
  
  // tf frames
  std::string m_map_frame;
  std::string m_base_frame;
  
  p11::Transformations m_transforms;
  
  ros::Timer m_timer;
  

};

#endif
