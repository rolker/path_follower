#include <path_follower/path_follower_plugin.h>
#include <geographic_visualization_msgs/GeoVizItem.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(path_follower_planner::PathFollowerPlugin, nav_core::BaseLocalPlanner)

namespace path_follower_planner
{

bool PathFollowerPlugin::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
{
  auto in_cmd_vel = cmd_vel;
  bool ret = generateCommands(cmd_vel);
  //ROS_INFO_STREAM_THROTTLE(1.0,"incoming twist:\n" << in_cmd_vel << "\nresult: " << ret << "\noutgoing twist:\n" << cmd_vel);
  if(ret)
  {
    std_msgs::Float64 cross_track_error;
    cross_track_error.data = crossTrackError();
    cross_track_error_pub_.publish(cross_track_error);

    std_msgs::Float64 distance_remaining;
    distance_remaining.data = distanceRemaining();
    distance_remaining_pub_.publish(distance_remaining);
  }
  sendDisplay();
  return ret;
}

void PathFollowerPlugin::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
{
  ROS_INFO_STREAM("Initializing path_follower with name " << name);
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~/" + name);
  PathFollower::initialize(nh, private_nh, tf);
  cross_track_error_pub_ = private_nh.advertise<std_msgs::Float64>("cross_track_error",10);
  distance_remaining_pub_ = private_nh.advertise<std_msgs::Float64>("distance_remaining",10);
} 

bool PathFollowerPlugin::isGoalReached()
{
  return goalReached();
}

bool PathFollowerPlugin::setPlan(const std::vector< geometry_msgs::PoseStamped > &plan)
{
  vis_display_.lines.clear();
  setGoal(plan);
  if(!plan.empty() and m_tf_buffer)
  {
    try
    {
      geometry_msgs::TransformStamped map_to_earth = m_tf_buffer->lookupTransform("earth", plan.front().header.frame_id, ros::Time(0));

      geographic_visualization_msgs::GeoVizPointList gvpl;

      for(const auto& map_point: plan)
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
      sendDisplay();
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN_STREAM("Unable to find transform to generate display: " << ex.what());
    }

  }
  return true;
}

}