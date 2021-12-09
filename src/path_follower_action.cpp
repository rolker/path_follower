#include "path_follower/path_follower_action.h"


PathFollowerAction::PathFollowerAction(std::string name):
  action_server_(ros::NodeHandle(), name, false),
  enabled_(true)
{
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  initialize(nh, nh_private, &transforms_());

  nh_private.param<std::string>("map_frame", this->map_frame_, "map");
  float update_rate;
  nh_private.param<float>("update_rate", update_rate , 10.0);

  // Subscribers
  enable_sub_ = nh.subscribe<std_msgs::Bool>(
    "enable", 10,
    [&](const std_msgs::BoolConstPtr& msg){this->enabled_ =
	msg->data; this->sendDisplay();});

  // Publishers
  cmd_vel_pub_ = nh.advertise<geometry_msgs::TwistStamped>("cmd_vel",1);


  // Action server callbacks
  action_server_.registerGoalCallback(
    boost::bind(&PathFollowerAction::goalCallback, this));
  action_server_.registerPreemptCallback(
    boost::bind(
      &PathFollowerAction::preemptCallback, this));
  action_server_.start();

  // Updater
  timer_ = nh.createTimer(ros::Duration(1.0/update_rate),
				 std::bind(&PathFollowerAction::timerCallback,
					   this, std::placeholders::_1));

}

PathFollowerAction::~PathFollowerAction()
{

}

void PathFollowerAction::goalCallback()
{
  auto goal = action_server_.acceptNewGoal();
  
  std::vector< geometry_msgs::PoseStamped > goal_path;

  geometry_msgs::TransformStamped earth_to_map =
    transforms_().lookupTransform(map_frame_,
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
    map_point.header.frame_id = map_frame_;
    map_point.header.stamp = pose.header.stamp;
    goal_path.push_back(map_point);
    gvpl.points.push_back(pose.pose.position);
  }
  vis_display_.lines.clear();
  vis_display_.lines.push_back(gvpl);
  
  sendDisplay();
  setGoal(goal_path, goal->speed);
}


void PathFollowerAction::preemptCallback()
{
  action_server_.setPreempted();
  vis_display_.lines.clear();
  sendDisplay();
}

void PathFollowerAction::timerCallback(const ros::TimerEvent event)
{
  if(action_server_.isActive())
    if(enabled_)
    {
      geometry_msgs::TwistStamped ts;
      ts.header.frame_id = m_base_frame;
      ts.header.stamp = event.current_real;
      if(generateCommands(ts.twist))
        cmd_vel_pub_.publish(ts);
      else
      {
        path_follower::path_followerResult result;
        result.ending_pose.position =
        transforms_.map_to_wgs84(geometry_msgs::Point(), this->m_base_frame);
        action_server_.setSucceeded(result);
      }

      // Publish action feedback
      path_follower::path_followerFeedback feedback;
      feedback.percent_complete = progress();
      feedback.crosstrack_error = crossTrackError();
      action_server_.publishFeedback(feedback);
      sendDisplay();
    }
    else
      sendDisplay(false);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_follower");

    PathFollowerAction pf("path_follower_action");
    
    ros::spin();
    
    return 0;
}
