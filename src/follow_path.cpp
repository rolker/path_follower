#include <path_follower/follow_path.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "project11/utils.h"
#include "project11/pid.h"
#include <tf2_ros/buffer.h>
#include <nav_msgs/Odometry.h>

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<path_follower::FollowPathCommand>("FollowPathCommand");
}

namespace path_follower
{

FollowPathCommand::FollowPathCommand(const std::string& name, const BT::NodeConfig& config):
  BT::StatefulActionNode(name, config)
{

}

BT::PortsList FollowPathCommand::providedPorts()
{
  return {
    BT::InputPort<std::shared_ptr<std::vector<geometry_msgs::PoseStamped> > >("navigation_path"),
    BT::InputPort<int>("current_navigation_segment"),
    BT::InputPort<nav_msgs::Odometry>("odometry"),
    BT::InputPort<std::shared_ptr<tf2_ros::Buffer> >("tf_buffer"),
    BT::InputPort<double>("target_speed"),
    BT::OutputPort<geometry_msgs::TwistStamped>("command_velocity"),
    BT::BidirectionalPort<std::shared_ptr<project11::PID> >("pid"),
  };
}

BT::NodeStatus FollowPathCommand::onStart()
{
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FollowPathCommand::onRunning()
{
  auto navigation_path_bb = getInput<std::shared_ptr<std::vector< geometry_msgs::PoseStamped> > >("navigation_path");
  if(!navigation_path_bb)
  {
    throw BT::RuntimeError("missing required input [navigation_path]: ", navigation_path_bb.error() );
  }
  auto navigation_path = navigation_path_bb.value();

  int segment_count = std::max<int>(0,navigation_path->size()-1);

  auto current_segment = getInput<int>("current_navigation_segment");
  if(!current_segment)
  {
    throw BT::RuntimeError("missing required input [current_navigation_segment]: ", current_segment.error() );
  }

  if(current_segment.value() < 0 || current_segment.value() > segment_count)
    return BT::NodeStatus::FAILURE;

  // We're done if we are at the segment past the last one
  if(current_segment.value() == segment_count)
    return BT::NodeStatus::SUCCESS;

  auto odom = getInput<nav_msgs::Odometry>("odometry");
  if(!odom)
  {
    throw BT::RuntimeError("missing required input [odometry]: ", odom.error() );
  }

  auto tf_buffer = getInput<std::shared_ptr<tf2_ros::Buffer> >("tf_buffer");
  if(!tf_buffer && tf_buffer.value())
  {
    throw BT::RuntimeError("missing required input [tf_buffer]: ", tf_buffer.error() );
  }

  std::shared_ptr<project11::PID> pid;
  auto pid_bb = getInput<std::shared_ptr<project11::PID> >("pid");
  if(pid_bb)
    pid = pid_bb.value();

  // create pid if it's not on the blackboard yet
  if(!pid)
  {
    pid = std::make_shared<project11::PID>();
    ros::NodeHandle nh("~/path_follower/pid");
    pid->configure(nh);
    setOutput("pid", pid);
  }

  double target_speed = 0.0;
  auto target_speed_bb = getInput<double>("target_speed");
  if(target_speed_bb)
    target_speed = target_speed_bb.value();

  geometry_msgs::TransformStamped base_to_map;
  try
  {
    base_to_map = tf_buffer.value()->lookupTransform(navigation_path->front().header.frame_id , odom.value().child_frame_id, ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_STREAM("FollowPathCommand node named " << name() << " Error getting path to base_frame transform: " << ex.what());
    return BT::NodeStatus::FAILURE;
  }

  auto p1 = (*navigation_path)[current_segment.value()];
  auto p2 = (*navigation_path)[current_segment.value()+1];

  auto segment_dx = p2.pose.position.x - p1.pose.position.x;
  auto segment_dy = p2.pose.position.y - p1.pose.position.y;

  project11::AngleRadians segment_azimuth = atan2(segment_dy, segment_dx);
  auto segment_distance = sqrt(segment_dx*segment_dx+segment_dy*segment_dy);

  // vehicle distance and azimuth relative to the segment's start point
  double dx = p1.pose.position.x - base_to_map.transform.translation.x;
  double dy = p1.pose.position.y - base_to_map.transform.translation.y;
  auto vehicle_distance = sqrt(dx*dx+dy*dy);

  project11::AngleRadians vehicle_azimuth = atan2(-dy, -dx);

  auto error_azimuth = vehicle_azimuth - segment_azimuth;
     
  auto sin_error_azimuth = sin(error_azimuth);
  auto cos_error_azimuth = cos(error_azimuth);

  // Distance traveled along the line.
  auto progress = vehicle_distance*cos_error_azimuth;

  auto cross_track_error = vehicle_distance*sin_error_azimuth;
  auto crab_angle = project11::AngleDegrees(pid->update(cross_track_error, odom.value().header.stamp));
  project11::AngleRadians heading = tf2::getYaw(base_to_map.transform.rotation);

  project11::AngleRadians target_heading = segment_azimuth +	crab_angle;

  geometry_msgs::TwistStamped cmd_vel;
  cmd_vel.header.stamp = odom.value().header.stamp;
  cmd_vel.header.frame_id = odom.value().child_frame_id;
  
  cmd_vel.twist.angular.z = project11::AngleRadiansZeroCentered(target_heading-heading).value();

  auto segment_start_time = p1.header.stamp;
  auto segment_end_time = p2.header.stamp;
  if(segment_start_time.isValid() && !segment_start_time.is_zero() && segment_end_time.isValid() && !segment_end_time.is_zero() && segment_end_time > segment_start_time)
  {
    auto dt = segment_end_time - segment_start_time;
    target_speed = segment_distance/dt.toSec();
  }

  double cos_crab = std::max(cos(crab_angle), 0.5);
  cmd_vel.twist.linear.x = target_speed/cos_crab;
  setOutput("command_velocity", cmd_vel);
  return BT::NodeStatus::RUNNING;
}


void FollowPathCommand::onHalted()
{
  
}


} // namespace path_follower
