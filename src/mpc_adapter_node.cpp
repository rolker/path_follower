#include <ros/ros.h>
#include <path_planner_common/UpdateReferenceTrajectory.h>
#include <actionlib/client/simple_action_client.h>
#include <path_follower/path_followerAction.h>
#include "project11/tf2_utils.h"
#include <nav_msgs/Odometry.h>
#include "project11/utils.h"

namespace p11 = project11;

extern "C" {
    #include "dubins.h"
}

struct State
{
    double x;
    double y;
    double yaw;
};

// callback for dubins library
int buildPath(double q[3], double t, void* user_data)
{
    std::vector<State>* states = reinterpret_cast<std::vector<State>*>(user_data);

    State s;
    s.x = q[0];
    s.y = q[1];
    s.yaw = q[2];
    
    states->push_back(s);
    
    return 0;
}


class MPCAdapterNode
{
public:
    MPCAdapterNode():m_path_follower_client("path_follower_action")
    {
        m_update_reference_trajectory_service = m_node_handle.advertiseService("mpc/update_reference_trajectory", &MPCAdapterNode::updateReferenceTrajectory, this);
        
        m_odom_sub = m_node_handle.subscribe("odom", 10, &MPCAdapterNode::odometryCallback, this);

        ros::NodeHandle nh_private("~");
        nh_private.param<std::string>("map_frame", m_map_frame, "map");

        
        ROS_INFO("Waiting for path_follower action server to start.");
        m_path_follower_client.waitForServer();
        ROS_INFO("Action server started.");
        
        
    }

    bool updateReferenceTrajectory(path_planner_common::UpdateReferenceTrajectory::Request &req, path_planner_common::UpdateReferenceTrajectory::Response &res)
    {
        ROS_DEBUG_STREAM("Got " << req.plan.paths.size() << " dubins paths");
        
        if(!req.plan.paths.empty())
        {
            while (!m_transformations().canTransform(m_map_frame, "earth", ros::Time(0), ros::Duration(0.5)))
            {
                ROS_WARN("Waiting for origin...");
            }

            path_follower::path_followerGoal goal;
            goal.speed = req.plan.paths[0].speed;
            
            std::vector<State> states;
            
            for(auto path: req.plan.paths)
            {
                DubinsPath dpath;
                dpath.qi[0] = path.initial_x;
                dpath.qi[1] = path.initial_y;
                dpath.qi[2] = path.initial_yaw;
                dpath.param[0] = path.length0;
                dpath.param[1] = path.length1;
                dpath.param[2] = path.length2;
                dpath.rho = path.rho;
                dpath.type = DubinsPathType(path.type);
                ROS_DEBUG_STREAM("path lengths: " << path.length0 << ", " << path.length1 << ", " << path.length2);
                
                int ret = dubins_path_sample_many(&dpath, goal.speed, buildPath, &states);
            }
            
            if(states.size() > 1 && m_odometry)
            {
                ROS_DEBUG_STREAM("path first point: " << states[0].x << ", " << states[0].y << " yaw: " << states[0].yaw  << " (degs) " << states[0].yaw*180/M_PI);
                ROS_DEBUG_STREAM("path second point: " << states[1].x << ", " << states[1].y << " yaw: " << states[1].yaw  << " (degs) " << states[1].yaw*180/M_PI);
                
                ROS_DEBUG_STREAM("now:\n" << *m_odometry);
                double next_x = m_odometry->pose.pose.position.x + m_odometry->twist.twist.linear.x;
                double next_y = m_odometry->pose.pose.position.y + m_odometry->twist.twist.linear.y;
                ROS_DEBUG_STREAM("in a sec: " << next_x << ", " << next_y);
                res.state.x = next_x;
                res.state.y = next_y;
                res.state.heading = M_PI_2-tf2::getYaw(m_odometry->pose.pose.orientation);

                if (sqrt(pow(next_x-states[1].x,2)+pow(next_y-states[1].y,2)) < 25)
                {
                    res.state.x = states[1].x;
                    res.state.y = states[1].y;
                    res.state.heading = M_PI_2-states[1].yaw;
                }
                
                res.state.speed = goal.speed;
                res.state.time = req.plan.paths[0].start_time+1.0;
                
                
                for(auto s:states)
                {
                    geometry_msgs::Point point;
                    point.x = s.x;
                    point.y = s.y;
                    
                    auto position = m_transformations.map_to_wgs84(point, m_map_frame);

                    geographic_msgs::GeoPoseStamped gps;
                    gps.pose.position.latitude = position.latitude;
                    gps.pose.position.longitude = position.longitude;
                    goal.path.poses.push_back(gps);
                }
    
                m_path_follower_client.sendGoal(goal);
                
                return true;
            }
        }
        return false;
    }

    void odometryCallback(const nav_msgs::Odometry::ConstPtr &inmsg)
    {
      m_odometry = inmsg;
    }

private:
    ros::NodeHandle m_node_handle;
    ros::ServiceServer m_update_reference_trajectory_service;
    actionlib::SimpleActionClient<path_follower::path_followerAction> m_path_follower_client;
    project11::Transformations m_transformations;
    
    ros::Subscriber m_odom_sub;
    nav_msgs::Odometry::ConstPtr m_odometry;
    std::string m_map_frame;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc_adapter");
    MPCAdapterNode node;
    ros::spin();

    return 0;
}
