#ifndef PATH_FOLLOWER_PLUGIN_H
#define PATH_FOLLOWER_PLUGIN_H

#include "path_follower.h"
#include <nav_core/base_local_planner.h>

namespace path_follower_planner
{

class PathFollowerPlugin: public nav_core::BaseLocalPlanner, public PathFollower
{
public:
  bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel) override;
  void initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros) override;
  bool isGoalReached() override;
  bool setPlan(const std::vector< geometry_msgs::PoseStamped > &plan) override;

private:
  // Pub
  ros::Publisher cross_track_error_pub_;
  ros::Publisher distance_remaining_pub_;

};

}

#endif
