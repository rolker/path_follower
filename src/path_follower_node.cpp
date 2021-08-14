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

#include "path_follower/path_follower_node.h"

namespace p11 = project11;

PathFollower::PathFollower(std::string name):
  m_action_server(ros::NodeHandle(), name, false),
  m_send_display_flag(true),
  m_goal_speed(0.0),
  m_crab_angle(0.0),
  m_current_segment_index(0),
  m_total_distance(0.0),
  m_cumulative_distance(0.0),
  m_enabled(true),
  m_kp_surge(0.1),
  m_kp_sway(0.1),
  m_kp_yaw(1.0),
  m_turn_in_place(true),
  m_turn_in_place_threshold(20.0),
  m_dynamics_mode(unicycle)
{

  // Initiate node and get parameters.
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  
  nh_private.param<std::string>("map_frame", this->m_map_frame, "map");
  nh_private.param<std::string>("base_frame", this->m_base_frame, "base_link");
  std::string dyn_mode_str;
  nh_private.param<std::string>("dynamics_mode", dyn_mode_str ,
				      "unicycle");
  this->m_dynamics_mode = this->str2dynamicsmode(dyn_mode_str);
  float update_rate;
  nh_private.param<float>("update_rate", update_rate , 10.0);
  // Gains for holonomic control
  nh_private.param<float>("kp_surge", this->m_kp_surge , 0.1);
  nh_private.param<float>("kp_sway", this->m_kp_sway , 0.1);
  nh_private.param<float>("kp_yaw", this->m_kp_yaw , 1.0);
  nh_private.param<bool>("turn_in_place", this->m_turn_in_place , true);
  nh_private.param<float>("turn_in_place_threshold",
			  this->m_turn_in_place_threshold , 20.0);

  // Subscribers
  this->m_enable_sub = nh.subscribe<std_msgs::Bool>(
    "enable", 10,
    [&](const std_msgs::BoolConstPtr& msg){this->m_enabled =
	msg->data; this->sendDisplay();});

  // Publishers
  this->m_cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("cmd_vel",1);
  this->m_display_pub = nh.advertise<geographic_visualization_msgs::GeoVizItem>
    ("project11/display",5);
  this->m_vis_display.id = "path_follower";
  
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
  
  // Action server callbacks
  this->m_action_server.registerGoalCallback(
    boost::bind(&PathFollower::goalCallback, this));
  this->m_action_server.registerPreemptCallback(
    boost::bind(
      &PathFollower::preemptCallback, this));
  this->m_action_server.start();

  // Updater
  this->m_timer = nh.createTimer(ros::Duration(1.0/update_rate),
				 std::bind(&PathFollower::timerCallback,
					   this, std::placeholders::_1));
}
    
PathFollower::~PathFollower()
{
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

void PathFollower::goalCallback()
{
  auto goal = this->m_action_server.acceptNewGoal();
  
  this->m_goal_speed = goal->speed;
  this->m_current_segment_index = 0;
  this->m_total_distance = 0.0;
  this->m_cumulative_distance = 0.0;
  
  this->m_goal_path.clear();
  
  geometry_msgs::TransformStamped earth_to_map =
    this->m_transforms().lookupTransform(this->m_map_frame,
					 "earth",
					 ros::Time(0));
  
  geographic_visualization_msgs::GeoVizPointList gvpl;
  
  for(auto pose: goal->path.poses)
  {
    // See header files in project11 repo.
    //  See: https://github.com/GFOE/project11/blob/dd6b840c342c62b69d0922bfb646c93fba807c55/include/project11/utils.h#L21
    // Which reassigns to this type: gz4d::GeoPointLatLongDegrees
    // See https://github.com/GFOE/project11/blob/dd6b840c342c62b69d0922bfb646c93fba807c55/include/project11/gz4d_geo.h#L1527
    p11::LatLongDegrees p;
    // See: https://github.com/GFOE/project11/blob/dd6b840c342c62b69d0922bfb646c93fba807c55/include/project11/utils.h#L40
    p11::fromMsg(pose.pose.position, p);
    // typedef gz4d::GeoPointECEF ECEF;
    // Not clear how assigning ecef_point from latlon
    p11::ECEF ecef_point = p;
    // Stuff into a message
    geometry_msgs::Point ecef_point_msg;
    p11::toMsg(ecef_point,ecef_point_msg);
    geometry_msgs::PoseStamped map_point;
    tf2::doTransform(ecef_point_msg, map_point.pose.position, earth_to_map);
    map_point.header.frame_id = this->m_map_frame;
    map_point.header.stamp = pose.header.stamp;
    this->m_goal_path.push_back(map_point);
    gvpl.points.push_back(pose.pose.position);
  }
  this->m_vis_display.lines.clear();
  this->m_vis_display.lines.push_back(gvpl);
  
  this->m_segment_azimuth_distances.clear();
  for(int i = 0; i < this->m_goal_path.size()-1; i++)
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
  this->m_send_display_flag = true;
}

void PathFollower::sendDisplay()
{
  if(!this->m_vis_display.lines.empty())
  {
    auto& plist = this->m_vis_display.lines.front();
    plist.size = 5.0;
    if (this->m_enabled)
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
  this->m_display_pub.publish(this->m_vis_display);
  this->m_send_display_flag = false;
}

void PathFollower::preemptCallback()
{
  this->m_action_server.setPreempted();
  this->m_vis_display.lines.clear();
  this->m_send_display_flag = true;
}

void PathFollower::controlEfforCallback(const std_msgs::Float64::ConstPtr& inmsg)
{
  this->m_crab_angle = p11::AngleDegrees(inmsg->data);
}

void PathFollower::timerCallback(const ros::TimerEvent event)
{
  if(this->m_send_display_flag)
    this->sendDisplay();
  if(this->m_action_server.isActive() &&
     this->m_enabled &&
     !this->m_goal_path.empty())
  {
    geometry_msgs::TransformStamped base_to_map;
    try
    {
      base_to_map = this->m_transforms().lookupTransform(
	this->m_map_frame, this->m_base_frame, ros::Time(0));
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
    // For readability, define current segment azimuth and distance vars.
    p11::AngleRadians curr_seg_azi(0.0);
    double curr_seg_dist = 0.0;
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
      if (progress >= curr_seg_dist)
      {
	this->m_cumulative_distance += curr_seg_dist;
	this->m_current_segment_index += 1;
	// have we reached the last segment?
	if(this->m_current_segment_index >= this->m_goal_path.size()-1)
	{
	  path_follower::path_followerResult result;
	  result.ending_pose.position =
	    this->m_transforms.map_to_wgs84(geometry_msgs::Point(),
					    this->m_base_frame);
	  this->m_action_server.setSucceeded(result);
	  sendDisplay();
	  return;
	}
      }
      else
      {
	found_current_segment = true;
      }
    }

    // Distance away from line, m
    double cross_track = vehicle_distance*sin_error_azimuth;

    // Publish action feedback
    path_follower::path_followerFeedback feedback;
    feedback.percent_complete =
      (this->m_cumulative_distance+progress)/this->m_total_distance;
    feedback.crosstrack_error = cross_track;
    this->m_action_server.publishFeedback(feedback);


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
      state.data = cross_track;
      this->m_state_pub.publish(state);
    }
    
    ros::Time now = ros::Time::now();
    geometry_msgs::TwistStamped ts;
    ts.header.frame_id = this->m_base_frame;
    ts.header.stamp = event.current_real;
    
    p11::AngleRadians heading = tf2::getYaw(base_to_map.transform.rotation);

    // Choose which algorithm to use.
    if (this->m_dynamics_mode == PathFollower::DynamicsMode::unicycle)
    {
      p11::AngleRadians target_heading = curr_seg_azi +	this->m_crab_angle;
      ts.twist.angular.z =
	p11::AngleRadiansZeroCentered(target_heading-heading).value();
      ts.twist.linear.x = this->m_goal_speed;
    }
    else if (this->m_dynamics_mode == PathFollower::DynamicsMode::holonomic)
    {
      // Heading: Proportional heading feedback.
      // Heading along the line
      p11::AngleRadians target_heading = curr_seg_azi;
      // Heading error, rad, ENU
      p11::AngleRadiansZeroCentered hdg_error(target_heading-heading);
      ts.twist.angular.z = this->m_kp_yaw * hdg_error.value();
      // Surge: target speed and then slow down when we are close.
      // Find distance to end of path
      double dx_goal =
	this->m_goal_path[this->m_current_segment_index+1].pose.position.x -
	base_to_map.transform.translation.x;
      double dy_goal =
	this->m_goal_path[this->m_current_segment_index+1].pose.position.y -
	base_to_map.transform.translation.y;
      double dist_goal = sqrt(dx_goal * dx_goal + dy_goal * dy_goal);
      ts.twist.linear.x = std::min(this->m_kp_surge * dist_goal,
				   this->m_goal_speed);
      // Sway: Proporational to cross track error
      ts.twist.linear.y = 1.0 * std::copysign(this->m_kp_sway * cross_track,
					       error_azimuth.value());

      // If turn in place, then reduce surge if we have large yaw error
      if (std::abs(hdg_error.value())*180.0/M_PI >
	  this->m_turn_in_place_threshold)
      {
	ts.twist.linear.x = 0.0;
	ts.twist.linear.y = 0.0;  
      }
      ROS_DEBUG("hdg_error: %.1f deg, yaw_rate: %.1f rad/s, "
		"dist_goal: %.1f m, surge: %.1f m/s, "
		"xtrack_err: %.1f m, sway: %.1f m/s, ",
		hdg_error.value(), ts.twist.angular.z,
		dist_goal, ts.twist.linear.x,
		cross_track, ts.twist.linear.y);
    }
    else
    {
      ROS_FATAL_NAMED("path_follower_node",
		      "Unrecognized DynamicsMode!");
    }
    this->m_cmd_vel_pub.publish(ts);
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
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_follower");

    PathFollower pf("path_follower_action");
    
    ros::spin();
    
    return 0;
}

    
