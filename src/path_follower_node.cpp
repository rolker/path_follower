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
  m_dynamics_mode(unicycle)
{

  ros::NodeHandle nh;
  
  this->m_enabled = true;
  this->m_enable_sub = nh.subscribe<std_msgs::Bool>("enable", 10, [&](const std_msgs::BoolConstPtr& msg){this->m_enabled = msg->data; this->sendDisplay();});
  
  this->m_cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("cmd_vel",1);
  
  this->m_display_pub = nh.advertise<geographic_visualization_msgs::GeoVizItem>("project11/display",5);
  this->m_vis_display.id = "path_follower";
  
  ros::NodeHandle nh_private("~");
  
  nh_private.param<std::string>("map_frame", this->m_map_frame, "map");
  nh_private.param<std::string>("base_frame", this->m_base_frame, "base_link");
  nh_private.param<std::string>("dynamics_mode", this->m_base_frame, "unicycle");
  
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
  
  this->m_action_server.registerGoalCallback(
    boost::bind(&PathFollower::goalCallback, this));
  this->m_action_server.registerPreemptCallback(
    boost::bind(
      &PathFollower::preemptCallback, this));
  this->m_action_server.start();
  
  this->m_timer = nh.createTimer(ros::Duration(0.1),
			   std::bind(&PathFollower::timerCallback,
				     this, std::placeholders::_1));
}
    
PathFollower::~PathFollower()
{
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
    this->m_transforms().lookupTransform(this->m_map_frame, "earth", ros::Time(0));
  
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
    sendDisplay();
  if(this->m_action_server.isActive() && this->m_enabled && !this->m_goal_path.empty())
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
    
    while(!found_current_segment)
    {
      double dx = this->m_goal_path[this->m_current_segment_index].pose.position.x -
	base_to_map.transform.translation.x;
      double dy = this->m_goal_path[this->m_current_segment_index].pose.position.y -
	base_to_map.transform.translation.y;
      vehicle_distance = sqrt(dx*dx+dy*dy);
      
      // angle from p to vehicle
      p11::AngleRadians azimuth = atan2(-dy, -dx);
      
      error_azimuth = azimuth - this->m_segment_azimuth_distances[this->m_current_segment_index].azimuth;
      
      sin_error_azimuth = sin(error_azimuth);
      cos_error_azimuth = cos(error_azimuth);
      
      progress = vehicle_distance*cos_error_azimuth;
      if (progress >=
	  this->m_segment_azimuth_distances[this->m_current_segment_index].distance)
      {
	this->m_cumulative_distance +=
	  this->m_segment_azimuth_distances[this->m_current_segment_index].distance;
	this->m_current_segment_index += 1;
	// have we reached the last segment?
	if(this->m_current_segment_index >= this->m_goal_path.size()-1)
	{
	  path_follower::path_followerResult result;
	  result.ending_pose.position =
	    this->m_transforms.map_to_wgs84(geometry_msgs::Point(), this->m_base_frame);
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
    
    double cross_track = vehicle_distance*sin_error_azimuth;
    
    path_follower::path_followerFeedback feedback;
    feedback.percent_complete =
      (this->m_cumulative_distance+progress)/this->m_total_distance;
    feedback.crosstrack_error = cross_track;
    this->m_action_server.publishFeedback(feedback);
    
    std_msgs::Bool pid_enable;
    pid_enable.data = true;
    this->m_pid_enable_pub.publish(pid_enable);
    
    std_msgs::Float64 setpoint;
    setpoint.data = 0.0;
    this->m_setpoint_pub.publish(setpoint);
    
    std_msgs::Float64 state;
    state.data = cross_track;
    this->m_state_pub.publish(state);
    
    ros::Time now = ros::Time::now();
    geometry_msgs::TwistStamped ts;
    ts.header.frame_id = this->m_base_frame;
    ts.header.stamp = event.current_real;
    
    p11::AngleRadians heading = tf2::getYaw(base_to_map.transform.rotation);
    p11::AngleRadians target_heading =
      this->m_segment_azimuth_distances[this->m_current_segment_index].azimuth +
      this->m_crab_angle;
    ts.twist.angular.z =
      p11::AngleRadiansZeroCentered(target_heading-heading).value();
    ts.twist.linear.x = this->m_goal_speed;
    
    this->m_cmd_vel_pub.publish(ts);
  }
  else
  {
    std_msgs::Bool pid_enable;
    pid_enable.data = false;
    this->m_pid_enable_pub.publish(pid_enable);
  }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_follower");

    PathFollower pf("path_follower_action");
    
    ros::spin();
    
    return 0;
}

    
