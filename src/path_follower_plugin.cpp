#include <path_follower/path_follower_plugin.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(path_follower_planner::PathFollowerPlugin, nav_core::BaseLocalPlanner)

namespace path_follower_planner
{

bool PathFollowerPlugin::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
{
  bool ret = generateCommands(cmd_vel);
  if(ret)
  {
    std_msgs::Float64 cross_track_error;
    cross_track_error.data = crossTrackError();
    cross_track_error_pub_.publish(cross_track_error);

    std_msgs::Float64 distance_remaining;
    distance_remaining.data = distanceRemaining();
    distance_remaining_pub_.publish(distance_remaining);
  }
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
  setGoal(plan);
  return true;
}

}