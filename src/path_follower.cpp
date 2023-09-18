/** 
 * @file path_follower.cpp
 * 
 * Discription goes here.
 * 
 * Local ROS Parameters:
 *  - map_frame: 
 *  - base_frame: 
 *  - dynamics_mode: str: one of "unicycle" or "holonomic".  
 *      Default is "unicycle"
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
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(path_follower::PathFollowerPlugin, project11_navigation::TaskToTwistWorkflow)

namespace p11 = project11;

namespace path_follower
{

void PathFollowerPlugin::configure(std::string name, project11_navigation::Context::Ptr context)
{
  context_ = context;
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~/" + name);
  m_tf_buffer = &context_->tfBuffer();
  // Initiate node and get parameters.

  std::string dyn_mode_str;
  private_nh.param<std::string>("dynamics_mode", dyn_mode_str , "unicycle");
  m_dynamics_mode = this->str2dynamicsmode(dyn_mode_str);
  // Gains for holonomic control
  private_nh.param<float>("kp_surge", m_kp_surge , 0.1);
  private_nh.param<float>("kp_sway", m_kp_sway , 0.1);
  private_nh.param<float>("kp_yaw", m_kp_yaw , 1.0);
  private_nh.param<bool>("turn_in_place", m_turn_in_place , true);
  private_nh.param<float>("turn_in_place_threshold", m_turn_in_place_threshold , 20.0);

  // Crab Angle PID pub/subs - only for unicycle mode
  if (m_dynamics_mode == DynamicsMode::unicycle)
  {
    ros::NodeHandle pid_nh(private_nh.getNamespace()+"/pid");
    m_pid.configure(pid_nh);
  }

  display_pub_ = nh.advertise<geographic_visualization_msgs::GeoVizItem>
    ("project11/display",5);
  vis_display_.id = "path_follower";
  vis_display_.lines.clear();
  sendDisplay();
}
    

PathFollowerPlugin::DynamicsMode PathFollowerPlugin::str2dynamicsmode(std::string str)
{
  if (str == "unicycle")
    return DynamicsMode::unicycle;
  else if (str == "holonomic")
    return DynamicsMode::holonomic;
  else
    ROS_FATAL_STREAM("path_follower_node: dynamics_mode <" << str <<
		     "> is not recognized.  Shutting down");
  ros::shutdown();
  // This is just to avoid a compiler warning - should never get here.
  return DynamicsMode::unicycle;
}

void PathFollowerPlugin::setGoal(const project11_navigation::Task::Ptr& input)
{
  if(current_task_ != input)
    task_update_time_ = ros::Time();
  current_task_ = input;
  updateTask();
}

void PathFollowerPlugin::updateTask()
{
  if(current_task_)
  {
    if(current_task_->lastUpdateTime() != task_update_time_)
    {
      clear();
      if(!current_task_->done())
      {
        m_goal_speed = context_->getRobotCapabilities().default_velocity.linear.x;
        auto speed_item = current_task_->dataItem("speed");
        try
        {
          m_goal_speed = speed_item.as<double>();
        }
        catch(const std::exception& e)
        {
        }
        
        if(speed_item.IsDefined())
        // auto data = current_task_->data();
        // if(data["speed"])
        //   m_goal_speed = data["speed"].as<double>();
        ROS_INFO_STREAM("speed: " << m_goal_speed);
        m_goal_path = current_task_->message().poses;
        for(int i = 0; i+1 < m_goal_path.size(); i++)
        {
          AzimuthDistance ad;
          double dx = m_goal_path[i+1].pose.position.x -
            m_goal_path[i].pose.position.x;
          double dy = m_goal_path[i+1].pose.position.y -
            m_goal_path[i].pose.position.y;
          ad.azimuth = atan2(dy,dx);
          ad.distance = sqrt(dx*dx+dy*dy);
          m_total_distance += ad.distance;
          m_segment_azimuth_distances.push_back(ad);
        }
      }
      updateDisplay();
      task_update_time_ = current_task_->lastUpdateTime();
    }
  }
  else
    clear();
}

void PathFollowerPlugin::clear()
{
  m_goal_path.clear();
  m_current_segment_index = 0;
  m_total_distance = 0.0;
  m_cumulative_distance = 0.0;
  m_current_segment_progress = 0.0;
  m_segment_azimuth_distances.clear();
  updateDisplay();
}

bool PathFollowerPlugin::running()
{
  updateTask();
  if(current_task_ && !current_task_->done())
    if(goalReached())
      current_task_->setDone();
    else
      return true;
  return false;
}

bool PathFollowerPlugin::getResult(geometry_msgs::TwistStamped& output)
{
  auto base_frame = output.header.frame_id;

  if(!m_goal_path.empty())
  {
    geometry_msgs::TransformStamped base_to_map;
    try
    {
      base_to_map = this->m_tf_buffer->lookupTransform(
        m_goal_path[0].header.frame_id, base_frame, ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN_STREAM("Error getting path to base_frame transform: " << ex.what());
    }
  
    double vehicle_distance;
    
    p11::AngleRadians error_azimuth;
    double sin_error_azimuth;
    double cos_error_azimuth;
    double progress;
    
    bool found_current_segment = false;
    // For readability, define current segment azimuth and distance vars.
    p11::AngleRadians current_segment_azimuth(0.0);
    double current_segment_distance = 0.0;

    ros::Time now = ros::Time::now();

    while(!found_current_segment)
    {
      // For hover, dx/dy is the distance:
      // From: start point of the path (where the vehicle started)
      // To: vehicle positon
      double dx = m_goal_path[m_current_segment_index].pose.position.x -
        base_to_map.transform.translation.x;
      double dy = m_goal_path[m_current_segment_index].pose.position.y -
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
      current_segment_azimuth = m_segment_azimuth_distances[m_current_segment_index].
        azimuth;
      current_segment_distance = m_segment_azimuth_distances[m_current_segment_index].
        distance;

      error_azimuth = azimuth - current_segment_azimuth;

      
      sin_error_azimuth = sin(error_azimuth);
      cos_error_azimuth = cos(error_azimuth);

      // Distance traveled along the line.
      progress = vehicle_distance*cos_error_azimuth;
      ROS_DEBUG_NAMED("path_follower_node",
          "azimuth: %.1f deg, segment_azimuth: %.1f deg, "
          "error_azimuth: %.1f deg, "
          "segment_distance: %.1f m, progress %.2f ",
          (double)azimuth*180.0/M_PI,
          (double)current_segment_azimuth*180.0/M_PI,
          (double)error_azimuth*180.0/M_PI,
          current_segment_distance, progress);

      // Have we completed this segment?
      bool segment_complete;
      segment_complete = progress >= current_segment_distance; // distance based

      if(segment_complete)
      {
        m_cumulative_distance += current_segment_distance;
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
    if (this->m_dynamics_mode == DynamicsMode::unicycle)
    {
      m_crab_angle = p11::AngleDegrees(m_pid.update(m_cross_track_error, now));
    }
    
    p11::AngleRadians heading = tf2::getYaw(base_to_map.transform.rotation);

    // Choose which algorithm to use.
    if (m_dynamics_mode == DynamicsMode::unicycle)
    {
      p11::AngleRadians target_heading = current_segment_azimuth +	m_crab_angle;
      output.twist.angular.z = p11::AngleRadiansZeroCentered(target_heading-heading).value();
      double target_speed = m_goal_speed;
      auto segment_start_time = m_goal_path[m_current_segment_index].header.stamp;
      auto segment_end_time = m_goal_path[m_current_segment_index+1].header.stamp;
      if(segment_start_time.isValid() && !segment_start_time.is_zero() && segment_end_time.isValid() && !segment_end_time.is_zero() && segment_end_time > segment_start_time)
      {
        auto dt = segment_end_time - segment_start_time;
        target_speed = current_segment_distance/dt.toSec();
        //ROS_INFO_STREAM_THROTTLE(0.5, "default speed: " << m_goal_speed << " target speed: " << target_speed);
      }

      double cos_crab = std::max(cos(m_crab_angle), 0.5);
      //ROS_INFO_STREAM_THROTTLE(0.5, "target speed along track: " << target_speed << " accounting for crab: " << target_speed/cos_crab << " cos crab: " << cos_crab);

      output.twist.linear.x = target_speed/cos_crab;
      sendDisplay();
      return true;
    }
    else if (m_dynamics_mode == DynamicsMode::holonomic)
    {
      // Heading: Proportional heading feedback.
      // Heading along the line
      p11::AngleRadians target_heading = current_segment_azimuth;
      // Heading error, rad, ENU
      p11::AngleRadiansZeroCentered hdg_error(target_heading-heading);
      output.twist.angular.z = this->m_kp_yaw * hdg_error.value();
      // Surge: target speed and then slow down when we are close.
      // Find distance to end of path
      double dx_goal =
       this->m_goal_path[this->m_current_segment_index+1].pose.position.x -
        base_to_map.transform.translation.x;
      double dy_goal =
        this->m_goal_path[this->m_current_segment_index+1].pose.position.y -
        base_to_map.transform.translation.y;
      double dist_goal = sqrt(dx_goal * dx_goal + dy_goal * dy_goal);
      output.twist.linear.x = std::min(this->m_kp_surge * dist_goal,
            this->m_goal_speed);
      // Sway: Proportional to cross track error
      output.twist.linear.y = 1.0 * std::copysign(this->m_kp_sway * m_cross_track_error,
                  error_azimuth.value());

      // If turn in place, then reduce surge if we have large yaw error
      if (std::abs(hdg_error.value())*180.0/M_PI >
        this->m_turn_in_place_threshold)
      {
        output.twist.linear.x = 0.0;
        output.twist.linear.y = 0.0;  
      }
      ROS_DEBUG("hdg_error: %.1f deg, yaw_rate: %.1f rad/s, "
        "dist_goal: %.1f m, surge: %.1f m/s, "
        "xtrack_err: %.1f m, sway: %.1f m/s, ",
        hdg_error.value(), output.twist.angular.z,
        dist_goal, output.twist.linear.x,
        m_cross_track_error, output.twist.linear.y);
      sendDisplay();
      return true;
    }
    else
    {
      ROS_FATAL_NAMED("path_follower_node",
          "Unrecognized DynamicsMode!");
    }
  }
  sendDisplay();
  return false;
}

double PathFollowerPlugin::progress() const
{
  if(m_total_distance == 0)
    return 0.0;
  return (this->m_cumulative_distance+m_current_segment_progress)/this->m_total_distance;
}

double PathFollowerPlugin::crossTrackError() const
{
  return m_cross_track_error;
}    

bool PathFollowerPlugin::goalReached() const
{
  return !m_goal_path.empty() && m_current_segment_index >= m_goal_path.size()-1;
}

double PathFollowerPlugin::distanceRemaining() const
{
  if(m_total_distance>0.0)
    return m_total_distance - m_cumulative_distance - m_current_segment_progress;
  return 0.0;
}

void PathFollowerPlugin::updateDisplay()
{
  vis_display_.lines.clear();
  if(!m_goal_path.empty() and m_tf_buffer)
  {
    try
    {
      geometry_msgs::TransformStamped map_to_earth = m_tf_buffer->lookupTransform("earth", m_goal_path.front().header.frame_id, ros::Time(0));

      geographic_visualization_msgs::GeoVizPointList gvpl;

      for(const auto& map_point: m_goal_path)
      {
        geometry_msgs::Point ecef_point_msg;
        tf2::doTransform(map_point.pose.position, ecef_point_msg, map_to_earth);
        p11::ECEF ecef_point;
        p11::fromMsg(ecef_point_msg, ecef_point);
        p11::LatLongDegrees ll_point = ecef_point;
        geographic_msgs::GeoPoint gp;
        p11::toMsg(ll_point, gp);
        gvpl.points.push_back(gp);
      }
      vis_display_.lines.push_back(gvpl);
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN_STREAM("Unable to find transform to generate display: " << ex.what());
    }

  }
  sendDisplay();
}

void PathFollowerPlugin::sendDisplay(bool dim)
{
  if(!vis_display_.lines.empty())
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


    auto& plist = vis_display_.lines.front();
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
    display_pub_.publish(vis_display_);
}

} // namespace path_follower
