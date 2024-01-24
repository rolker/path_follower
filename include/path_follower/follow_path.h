#ifndef PATH_FOLLOWER_FOLLOW_PATH_H
#define PATH_FOLLOWER_FOLLOW_PATH_H

#include <behaviortree_cpp/bt_factory.h>

namespace path_follower
{

class FollowPathCommand: public BT::StatefulActionNode
{
public:
  FollowPathCommand(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

} // namespace path_follower

#endif
