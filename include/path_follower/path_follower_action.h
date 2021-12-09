#ifndef PATH_FOLLOWER_ACTION_H
#define PATH_FOLLOWER_ACTION_H

#include "path_follower.h"
#include "path_follower/path_followerAction.h"
#include "actionlib/server/simple_action_server.h"

class PathFollowerAction: public PathFollower
{
public:
  PathFollowerAction(std::string name);
  ~PathFollowerAction();

  void goalCallback();
  void preemptCallback();

private:
  void timerCallback(const ros::TimerEvent event);

  actionlib::SimpleActionServer<path_follower::path_followerAction> action_server_;

  // Pub
  ros::Publisher cmd_vel_pub_;

  // Only send control commands when enabled
  bool enabled_;
  ros::Subscriber enable_sub_;

  p11::Transformations transforms_;
  ros::Timer timer_;

  // tf frames
  std::string map_frame_;
};

#endif
