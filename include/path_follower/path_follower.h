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
#include "project11/utils.h"
#include "project11/tf2_utils.h"

namespace p11 = project11;

class PathFollower
{
public:
  
  PathFollower();
  ~PathFollower();

protected:
  void initialize(ros::NodeHandle& nh, ros::NodeHandle& nh_private, const tf2_ros::Buffer *tf);
  bool generateCommands(geometry_msgs::Twist &cmd_vel);
  void setGoal(const std::vector< geometry_msgs::PoseStamped > & 	plan, double speed=0.0);
  double crossTrackError() const;
  double progress() const;
  bool goalReached() const;
  double distanceRemaining() const;
  void sendDisplay(bool dim=false);

  std::string m_base_frame;
  geographic_visualization_msgs::GeoVizItem vis_display_;

  const tf2_ros::Buffer *m_tf_buffer = nullptr;
private:
  void controlEfforCallback(const std_msgs::Float64::ConstPtr& inmsg);

  // Dynamics mode
  enum DynamicsMode { unicycle, holonomic };
  DynamicsMode m_dynamics_mode;

  /**
   * @brief Converts string (from ROS parameter) to mode enumeration.
   */
  PathFollower::DynamicsMode str2dynamicsmode(std::string str);


  /** @brief Holds the path bearing [rad, ENU] and distance [m] values. **/
  struct AzimuthDistance
  {
    p11::AngleRadians azimuth;
    double distance;
  };

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
  double m_current_segment_progress; // distance of completed progress for current segment in meters.
  double m_cross_track_error; // cross-track distance in meters. 

  
  // PID pub/subs
  ros::Publisher m_state_pub;
  ros::Publisher m_setpoint_pub;
  ros::Subscriber m_control_effort_sub;
  ros::Publisher m_pid_enable_pub;
  
  // display
  ros::Publisher display_pub_;

};

#endif
