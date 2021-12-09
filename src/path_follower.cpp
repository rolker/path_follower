/** 
 * @file path_follower_node.cpp
 * 
 * Discription goes here.
 * 
 * Local ROS Parameters:
 *  - map_frame: 
 *  - base_frame: 
 *  - dynamics_mode: str: one of "unicycle" or "holonomic".  
 *      Default is "unicycle"
 *  - update_rate: float: update rate in Hz.
 *      Default is 10.0
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

#include "path_follower/path_follower.h"

namespace p11 = project11;

PathFollower::PathFollower():
  m_goal_speed(0.0),
  m_crab_angle(0.0),
  m_current_segment_index(0),
  m_total_distance(0.0),
  m_cumulative_distance(0.0),
  m_current_segment_progress(0.0),
  m_kp_surge(0.1),
  m_kp_sway(0.1),
  m_kp_yaw(1.0),
  m_turn_in_place(true),
  m_turn_in_place_threshold(20.0),
  m_dynamics_mode(unicycle)
{
}

PathFollower::~PathFollower()
{
}


void PathFollower::initialize(ros::NodeHandle& nh, ros::NodeHandle& nh_private, const tf2_ros::Buffer *tf)
{
  m_tf_buffer = tf;
  // Initiate node and get parameters.
  nh_private.param<std::string>("base_frame", this->m_base_frame, "base_link");

  
  std::string dyn_mode_str;
  nh_private.param<std::string>("dynamics_mode", dyn_mode_str ,
				      "unicycle");
  this->m_dynamics_mode = this->str2dynamicsmode(dyn_mode_str);
  // Gains for holonomic control
  nh_private.param<float>("kp_surge", this->m_kp_surge , 0.1);
  nh_private.param<float>("kp_sway", this->m_kp_sway , 0.1);
  nh_private.param<float>("kp_yaw", this->m_kp_yaw , 1.0);
  nh_private.param<bool>("turn_in_place", this->m_turn_in_place , true);
  nh_private.param<float>("turn_in_place_threshold",
			  this->m_turn_in_place_threshold , 20.0);


  
  // Crab Angle PID pub/subs - only for unicycle mode
  if (this->m_dynamics_mode == PathFollower::DynamicsMode::unicycle)
  {
    this->m_setpoint_pub = nh_private.advertise<std_msgs::Float64>(
      "crab_angle_pid/setpoint",1, true);
    this->m_state_pub = nh_private.advertise<std_msgs::Float64>(
      "crab_angle_pid/state",1);
    this->m_control_effort_sub = nh_private.subscribe(
      "crab_angle_pid/control_effort", 10,
      &PathFollower::controlEfforCallback, this);
    this->m_pid_enable_pub = nh_private.advertise<std_msgs::Bool>(
      "crab_angle_pid/pid_enable", 1);
    std_msgs::Float64 setpoint;
    setpoint.data = 0.0;
    this->m_setpoint_pub.publish(setpoint);
  }

  display_pub_ = nh.advertise<geographic_visualization_msgs::GeoVizItem>
    ("project11/display",5);
  vis_display_.id = "path_follower";

}
    

PathFollower::DynamicsMode PathFollower::str2dynamicsmode(std::string str)
{
  if (str == "unicycle")
    return PathFollower::DynamicsMode::unicycle;
  else if (str == "holonomic")
    return PathFollower::DynamicsMode::holonomic;
  else
    ROS_FATAL_STREAM("path_follower_node: dynamics_mode <" << str <<
		     "> is not recognized.  Shutting down");
  ros::shutdown();
  // This is just to avoid a compiler warning - should never get here.
  return PathFollower::DynamicsMode::unicycle;
}

void PathFollower::setGoal(const std::vector< geometry_msgs::PoseStamped > & 	plan, double speed)
{
  m_goal_path = plan;
  m_goal_speed = speed;
  this->m_current_segment_index = 0;
  this->m_total_distance = 0.0;
  this->m_cumulative_distance = 0.0;
  this->m_current_segment_progress = 0.0;
  this->m_segment_azimuth_distances.clear();
  for(int i = 0; i+i < this->m_goal_path.size(); i++)
  {
    AzimuthDistance ad;
    double dx = this->m_goal_path[i+1].pose.position.x -
      this->m_goal_path[i].pose.position.x;
    double dy = this->m_goal_path[i+1].pose.position.y -
      this->m_goal_path[i].pose.position.y;
    ad.azimuth = atan2(dy,dx);
    ad.distance = sqrt(dx*dx+dy*dy);
    this->m_total_distance += ad.distance;
    this->m_segment_azimuth_distances.push_back(ad);
  }
}


void PathFollower::controlEfforCallback(const std_msgs::Float64::ConstPtr& inmsg)
{
  this->m_crab_angle = p11::AngleDegrees(inmsg->data);
}


bool PathFollower::generateCommands(geometry_msgs::Twist &cmd_vel)
{
  if(!m_goal_path.empty())
  {
    geometry_msgs::TransformStamped base_to_map;
    try
    {
      base_to_map = this->m_tf_buffer->lookupTransform(
        m_goal_path[0].header.frame_id, this->m_base_frame, ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN_STREAM("PathFollower::generateCommands: " << ex.what());
    }
  
    double vehicle_distance;
    
    p11::AngleRadians error_azimuth;
    double sin_error_azimuth;
    double cos_error_azimuth;
    double progress;
    
    bool found_current_segment = false;
    // For readability, define current segment azimuth and distance vars.
    p11::AngleRadians curr_seg_azi(0.0);
    double curr_seg_dist = 0.0;

    ros::Time now = ros::Time::now();

    while(!found_current_segment)
    {
      // For hover, dx/dy is the distance:
      // From: start point of the path (where the vehicle started)
      // To: vehicle positon
      double dx =
        this->m_goal_path[this->m_current_segment_index].pose.position.x -
        base_to_map.transform.translation.x;
      double dy =
        this->m_goal_path[this->m_current_segment_index].pose.position.y -
        base_to_map.transform.translation.y;
      vehicle_distance = sqrt(dx*dx+dy*dy);
      ROS_DEBUG_NAMED("path_follower_node",
          "path.x: %.1f, veh.x: %.1f, path.y: %.1f, veh.y: %.1f, "
          "dx: %.1f m , dy: %.1f, vehicle_distance: %.1f m",
          this->m_goal_path[this->m_current_segment_index].
          pose.position.x,
          base_to_map.transform.translation.x,
          this->m_goal_path[this->m_current_segment_index].
          pose.position.y,
          base_to_map.transform.translation.y,
          dx, dy, vehicle_distance);
      // Angle from path to vehicle
      // For hover, this is the angle (ENU) from the start point of the path
      // to the vehicle.
      p11::AngleRadians azimuth = atan2(-dy, -dx);

      // For readability, define current segment azimuth and distance vars.
      curr_seg_azi =
        this->m_segment_azimuth_distances[this->m_current_segment_index].
        azimuth;
      curr_seg_dist =
        this->m_segment_azimuth_distances[this->m_current_segment_index].
        distance;

      error_azimuth = azimuth - curr_seg_azi;
      
      sin_error_azimuth = sin(error_azimuth);
      cos_error_azimuth = cos(error_azimuth);

      // Distance traveled along the line.
      progress = vehicle_distance*cos_error_azimuth;
      ROS_DEBUG_NAMED("path_follower_node",
          "azimuth: %.1f deg, segment_azimuth: %.1f deg, "
          "error_azimuth: %.1f deg, "
          "segment_distance: %.1f m, progress %.2f ",
          (double)azimuth*180.0/M_PI,
          (double)curr_seg_azi*180.0/M_PI,
          (double)error_azimuth*180.0/M_PI,
          curr_seg_dist, progress);

      // Have we completed this segment?
      bool segment_complete;
      if (m_goal_speed == 0.0)
        segment_complete = m_goal_path[m_current_segment_index+1].header.stamp < now; // time based
      else
        segment_complete = progress >= curr_seg_dist; // distance based

      if(segment_complete)
      {
        m_cumulative_distance += curr_seg_dist;
        m_current_segment_index += 1;
        if(this->m_current_segment_index >= this->m_goal_path.size()-1)
        {
          m_current_segment_progress = 0;
          return false;
        }
      }
      else
      {
        m_current_segment_progress = progress;
        found_current_segment = true;
      }
    }

    // Distance away from line, m
    m_cross_track_error = vehicle_distance*sin_error_azimuth;


    // Cross track PID for unicycle mode.
    if (this->m_dynamics_mode == PathFollower::DynamicsMode::unicycle)
    {
      std_msgs::Bool pid_enable;
      pid_enable.data = true;
      this->m_pid_enable_pub.publish(pid_enable);
      
      std_msgs::Float64 setpoint;
      setpoint.data = 0.0;
      this->m_setpoint_pub.publish(setpoint);
      
      std_msgs::Float64 state;
      state.data = m_cross_track_error;
      this->m_state_pub.publish(state);
    }
    
    
    p11::AngleRadians heading = tf2::getYaw(base_to_map.transform.rotation);

    // Choose which algorithm to use.
    if (this->m_dynamics_mode == PathFollower::DynamicsMode::unicycle)
    {
      p11::AngleRadians target_heading = curr_seg_azi +	this->m_crab_angle;
      cmd_vel.angular.z =
        p11::AngleRadiansZeroCentered(target_heading-heading).value();
      double target_speed = m_goal_speed;
      if(m_goal_speed == 0.0)
      {
        // make sure we are not ahead of schedule
        if(progress < curr_seg_dist)
        {
          ros::Duration dt = m_goal_path[m_current_segment_index+1].header.stamp - now;
          if(dt < ros::Duration(0.0)) // we are late
            dt = ros::Duration(1.0); // try to get there in one second
          target_speed = (curr_seg_dist - progress)/dt.toSec();
        }
      }

      double cos_crab = std::max(cos(m_crab_angle), 0.5);
      //ROS_INFO_STREAM_THROTTLE(1.0, "target speed along track: " << target_speed << " accounting for crab: " << target_speed/cos_crab << " cos crab: " << cos_crab);


      cmd_vel.linear.x = target_speed/cos_crab;
      return true;
    }
    else if (this->m_dynamics_mode == PathFollower::DynamicsMode::holonomic)
    {
      // Heading: Proportional heading feedback.
      // Heading along the line
      p11::AngleRadians target_heading = curr_seg_azi;
      // Heading error, rad, ENU
      p11::AngleRadiansZeroCentered hdg_error(target_heading-heading);
      cmd_vel.angular.z = this->m_kp_yaw * hdg_error.value();
      // Surge: target speed and then slow down when we are close.
      // Find distance to end of path
      double dx_goal =
       this->m_goal_path[this->m_current_segment_index+1].pose.position.x -
        base_to_map.transform.translation.x;
      double dy_goal =
        this->m_goal_path[this->m_current_segment_index+1].pose.position.y -
        base_to_map.transform.translation.y;
      double dist_goal = sqrt(dx_goal * dx_goal + dy_goal * dy_goal);
      cmd_vel.linear.x = std::min(this->m_kp_surge * dist_goal,
            this->m_goal_speed);
      // Sway: Proporational to cross track error
      cmd_vel.linear.y = 1.0 * std::copysign(this->m_kp_sway * m_cross_track_error,
                  error_azimuth.value());

      // If turn in place, then reduce surge if we have large yaw error
      if (std::abs(hdg_error.value())*180.0/M_PI >
        this->m_turn_in_place_threshold)
      {
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;  
      }
      ROS_DEBUG("hdg_error: %.1f deg, yaw_rate: %.1f rad/s, "
        "dist_goal: %.1f m, surge: %.1f m/s, "
        "xtrack_err: %.1f m, sway: %.1f m/s, ",
        hdg_error.value(), cmd_vel.angular.z,
        dist_goal, cmd_vel.linear.x,
        m_cross_track_error, cmd_vel.linear.y);
      return true;
    }
    else
    {
      ROS_FATAL_NAMED("path_follower_node",
          "Unrecognized DynamicsMode!");
    }
  }
  // If action server is inactive or the control is disabled or no more points
  else  
  {
    // Cross track PID
    if (this->m_dynamics_mode == PathFollower::DynamicsMode::unicycle)
    {
      std_msgs::Bool pid_enable;
      pid_enable.data = false;
      this->m_pid_enable_pub.publish(pid_enable);
    }
  }
  return false;
}

double PathFollower::progress() const
{
  if(m_total_distance == 0)
    return 0.0;
  return (this->m_cumulative_distance+m_current_segment_progress)/this->m_total_distance;
}

double PathFollower::crossTrackError() const
{
  return m_cross_track_error;
}    

bool PathFollower::goalReached() const
{
  return !m_goal_path.empty() && m_current_segment_index >= m_goal_path.size()-1;
}

double PathFollower::distanceRemaining() const
{
  if(m_total_distance>0.0)
    return m_total_distance - m_cumulative_distance - m_current_segment_progress;
  return 0.0;
}


void PathFollower::sendDisplay(bool dim)
{
  if(!this->vis_display_.lines.empty())
  {
    double intensity = 1.0;
    if(dim)
      intensity = 0.5;

    geographic_visualization_msgs::GeoVizPointList past_segments;
    past_segments.size = 5.0;
    past_segments.color.r = 0.5*intensity;
    past_segments.color.g = 0.5*intensity;
    past_segments.color.b = 0.5*intensity;
    past_segments.color.a = 1.0*intensity;
    geographic_visualization_msgs::GeoVizPointList current_segments;
    current_segments.size = 7.0;
    current_segments.color.r = 0.0*intensity;
    current_segments.color.g = 1.0*intensity;
    current_segments.color.b = 0.0*intensity;
    current_segments.color.a = 1.0*intensity;
    geographic_visualization_msgs::GeoVizPointList future_segments;
    future_segments.size = 5.0;
    future_segments.color.r = 0.0*intensity;
    future_segments.color.g = 0.0*intensity;
    future_segments.color.b = 1.0*intensity;
    future_segments.color.a = 1.0*intensity;


    auto& plist = this->vis_display_.lines.front();
    for(int i = 0; i < plist.points.size() && i <= m_current_segment_index; i++)
      past_segments.points.push_back(plist.points[i]);
    for(int i = m_current_segment_index; i < plist.points.size() && i <= m_current_segment_index+1; i++)
      current_segments.points.push_back(plist.points[i]);
    for(int i = m_current_segment_index+1; i < plist.points.size(); i++)
      future_segments.points.push_back(plist.points[i]);

    geographic_visualization_msgs::GeoVizItem display;
    display.id = vis_display_.id;
    display.lines.push_back(past_segments);
    display.lines.push_back(current_segments);
    display.lines.push_back(future_segments);
    display_pub_.publish(display);
  }
  else
    this->display_pub_.publish(this->vis_display_);
}


