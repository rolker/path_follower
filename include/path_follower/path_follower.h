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
#ifndef PATH_FOLLOWER_H
#define PATH_FOLLOWER_H

#include <vector>
#include <cmath>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/TwistStamped.h"
#include "geographic_visualization_msgs/GeoVizItem.h"
#include "project11/utils.h"
#include "project11/tf2_utils.h"
#include "project11/pid.h"
#include <project11_navigation/interfaces/task_to_twist_workflow.h>

namespace p11 = project11;

namespace path_follower
{

class PathFollowerPlugin: public project11_navigation::TaskToTwistWorkflow
{
public:
  void configure(std::string name, project11_navigation::Context::Ptr context) override;
  void setGoal(const project11_navigation::Task::Ptr& input) override;
  bool running() override;
  bool getResult(geometry_msgs::TwistStamped& output) override;

private:
  void updateTask();
  void clear();
  double crossTrackError() const;
  double progress() const;
  bool goalReached() const;
  double distanceRemaining() const;
  void updateDisplay();
  void sendDisplay(bool dim=false);

  geographic_visualization_msgs::GeoVizItem vis_display_;

  const tf2_ros::Buffer *m_tf_buffer = nullptr;
  project11_navigation::Context::Ptr context_;
  project11_navigation::Task::Ptr current_task_;
  ros::Time task_update_time_;
  std::string map_frame_;

  // Dynamics mode
  enum DynamicsMode { unicycle, holonomic };
  DynamicsMode m_dynamics_mode = DynamicsMode::unicycle;

  /**
   * @brief Converts string (from ROS parameter) to mode enumeration.
   */
  DynamicsMode str2dynamicsmode(std::string str);


  /** @brief Holds the path bearing [rad, ENU] and distance [m] values. **/
  struct AzimuthDistance
  {
    p11::AngleRadians azimuth;
    double distance;
  };

  std::vector<geometry_msgs::PoseStamped> m_goal_path;

  std::vector<AzimuthDistance> m_segment_azimuth_distances;


  float m_kp_yaw = 1.0;
  float m_kp_surge = 0.1;
  float m_kp_sway = 0.1;
  bool m_turn_in_place = true;
  float m_turn_in_place_threshold = 20.0;
  
  int m_current_segment_index = 0;

  double m_goal_speed = 0.0;

  p11::AngleRadians m_crab_angle = 0.0; // Steering angle correction to correct for external forces
    
  double m_total_distance = 0.0; // total distance of complete path in meters.
  double m_cumulative_distance = 0.0; // distance of completed segments in meters.
  double m_current_segment_progress = 0.0; // distance of completed progress for current segment in meters.
  double m_cross_track_error = 0.0; // cross-track distance in meters. 

  p11::PID m_pid;
  
  // display
  ros::Publisher display_pub_;

};

} // namespace path_follower

#endif
