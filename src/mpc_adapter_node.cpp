#include <ros/ros.h>
#include <path_planner_common/UpdateReferenceTrajectory.h>
#include <actionlib/client/simple_action_client.h>
#include <path_follower/path_followerAction.h>
#include "project11_transformations/local_services.h"
#include <geometry_msgs/PoseStamped.h>
#include "marine_msgs/CourseMadeGoodStamped.h"
#include <geometry_msgs/TwistStamped.h>

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
    MPCAdapterNode():m_path_follower_client("path_follower_action"),m_transformations(m_node_handle)
    {
        m_update_reference_trajectory_service = m_node_handle.advertiseService("/mpc/update_reference_trajectory", &MPCAdapterNode::updateReferenceTrajectory, this);
        
        m_position_sub = m_node_handle.subscribe("/position_map", 10, &MPCAdapterNode::positionCallback, this);
        m_cmg_sub = m_node_handle.subscribe("/cmg", 10, &MPCAdapterNode::cmgCallback, this);
        m_sog_sub = m_node_handle.subscribe("/sog", 10, &MPCAdapterNode::sogCallback, this);

        
        ROS_INFO("Waiting for path_follower action server to start.");
        m_path_follower_client.waitForServer();
        ROS_INFO("Action server started.");
        
        
    }

    bool updateReferenceTrajectory(path_planner_common::UpdateReferenceTrajectory::Request &req, path_planner_common::UpdateReferenceTrajectory::Response &res)
    {
        ROS_INFO_STREAM("Got " << req.plan.paths.size() << " dubins paths");
        
        if(!req.plan.paths.empty())
        {
            while (!m_transformations.haveOrigin())
            {
                ROS_INFO("Waiting for origin...");
                ros::Duration(0.5).sleep();
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
                ROS_INFO_STREAM("path lengths: " << path.length0 << ", " << path.length1 << ", " << path.length2);
                
                int ret = dubins_path_sample_many(&dpath, goal.speed, buildPath, &states);
            }
            
            if(states.size() > 1)
            {
                ROS_INFO_STREAM("path first point: " << states[0].x << ", " << states[0].y << " yaw: " << states[0].yaw  << " (degs) " << states[0].yaw*180/M_PI);
                ROS_INFO_STREAM("path second point: " << states[1].x << ", " << states[1].y << " yaw: " << states[1].yaw  << " (degs) " << states[1].yaw*180/M_PI);
                
                ROS_INFO_STREAM("now: " << m_position_x << ", " << m_position_y << " cmg: " << m_cmg << " (degs) " << m_cmg*180/M_PI);
                double next_x = m_position_x + m_sog*sin(m_cmg);
                double next_y = m_position_y + m_sog*cos(m_cmg);
                ROS_INFO_STREAM("in a sec: " << next_x << ", " << next_y);
                res.state.x = next_x;
                res.state.y = next_y;
                res.state.heading = m_cmg;

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
                    
                    auto position = m_transformations.map_to_wgs84(point);

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

    void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &inmsg)
    {
        m_position_x = inmsg->pose.position.x;
        m_position_y = inmsg->pose.position.y;
    }

    void cmgCallback(const marine_msgs::CourseMadeGoodStamped::ConstPtr& inmsg)
    {
        m_cmg = inmsg->course * M_PI / 180.0;
    }

    void sogCallback(const geometry_msgs::TwistStamped::ConstPtr& inmsg)
    {
        m_sog = inmsg->twist.linear.x;
    }

private:
    ros::NodeHandle m_node_handle;
    ros::ServiceServer m_update_reference_trajectory_service;
    actionlib::SimpleActionClient<path_follower::path_followerAction> m_path_follower_client;
    project11::Transformations m_transformations;
    
    ros::Subscriber m_position_sub;
    ros::Subscriber m_cmg_sub;
    ros::Subscriber m_sog_sub;

    double m_sog, m_cmg, m_position_x, m_position_y;
    
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc_adapter");
    MPCAdapterNode node;
    ros::spin();

    return 0;
}
