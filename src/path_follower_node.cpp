#include "ros/ros.h"
#include "geographic_msgs/GeoPointStamped.h"
#include "geographic_msgs/GeoPath.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "marine_msgs/NavEulerStamped.h"
#include <vector>
#include "project11/gz4d_geo.h"
#include "path_follower/path_followerAction.h"
#include "actionlib/server/simple_action_server.h"

struct LatLong
{
    double latitude;
    double longitude;
};

// http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28GoalCallbackMethod%29
// http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Callback%20Based%20Simple%20Action%20Client

class PathFollower
{
public:
    PathFollower(std::string name):
      m_action_server(m_node_handle, name, false)
    {
        m_current_speed = 3.0;
        m_crab_angle = 0.0;
        m_current_segment_index = 0;
        m_total_distance = 0.0;
        m_cumulative_distance = 0.0;
    
        m_desired_heading_pub = m_node_handle.advertise<marine_msgs::NavEulerStamped>("/project11/desired_heading",1);
        m_desired_speed_pub = m_node_handle.advertise<geometry_msgs::TwistStamped>("/project11/desired_speed",1);
        m_setpoint_pub = m_node_handle.advertise<std_msgs::Float64>("/project11/crab_angle/setpoint",1);
        m_state_pub = m_node_handle.advertise<std_msgs::Float64>("/project11/crab_angle/state",1);
        
        m_position_sub = m_node_handle.subscribe("/position", 10, &PathFollower::positionCallback, this);
        m_heading_sub = m_node_handle.subscribe("/heading", 10, &PathFollower::headingCallback, this);
        m_control_effort_pub = m_node_handle.subscribe("/project11/crab_angle/control_effort", 10, &PathFollower::controlEfforCallback, this);
        
        m_action_server.registerGoalCallback(boost::bind(&PathFollower::goalCallback, this));
        m_action_server.registerPreemptCallback(boost::bind(&PathFollower::preemptCallback, this));
        m_action_server.start();
    }
    
    ~PathFollower()
    {
    }
    
    void goalCallback()
    {
        auto goal = m_action_server.acceptNewGoal();

        m_current_speed = goal->speed;
        m_current_segment_index = 0;
        m_total_distance = 0.0;
        m_cumulative_distance = 0.0;

        m_current_path.clear();
        for(auto pose: goal->path.poses)
        {
            LatLong ll;
            ll.latitude = pose.pose.position.latitude;
            ll.longitude = pose.pose.position.longitude;
            m_current_path.push_back(ll);
        }
        
        m_segment_azimuth_distances.clear();
        for(int i = 0; i < m_current_path.size()-1; i++)
        {
            gz4d::geo::Point<double,gz4d::geo::WGS84::LatLon> p1, p2;
            p1[0] = m_current_path[i].latitude;
            p1[1] = m_current_path[i].longitude;
            p2[0] = m_current_path[i+1].latitude;
            p2[1] = m_current_path[i+1].longitude;
            m_segment_azimuth_distances.push_back(gz4d::geo::WGS84::Ellipsoid::inverse(p1,p2));
            m_total_distance += m_segment_azimuth_distances.back().second;
        }
    }
    
    void preemptCallback()
    {
        m_action_server.setPreempted();
    }
    
    void controlEfforCallback(const std_msgs::Float64::ConstPtr& inmsg)
    {
        m_crab_angle = inmsg->data;
    }

    void sendDesired()
    {
        if(m_action_server.isActive() && !m_current_path.empty())
        {
            gz4d::geo::Point<double,gz4d::geo::WGS84::LatLon> p1, p2,vehicle_position;
            
            std::pair<double,double> vehicle_azimuth_distance;
            double error_azimuth;
            double sin_error_azimuth;
            double cos_error_azimuth;
            double progress;

            bool found_current_segment = false;
            
            while(!found_current_segment)
            {
                p1[0] = m_current_path[m_current_segment_index].latitude;
                p1[1] = m_current_path[m_current_segment_index].longitude;
                //p2[0] = m_current_path[m_current_segment_index+1].latitude;
                //p2[1] = m_current_path[m_current_segment_index+1].longitude;
                //std::cerr << "p1: " << p1[0] << "," << p1[1] << " p2: " << p2[0] << "," << p2[1] << std::endl;
                
                vehicle_position[0] = m_current_position.latitude;
                vehicle_position[1] = m_current_position.longitude;
                    
                //path_azimuth_distance = gz4d::geo::WGS84::Ellipsoid::inverse(p1,p2);
                vehicle_azimuth_distance = gz4d::geo::WGS84::Ellipsoid::inverse(p1,vehicle_position);

                //std::cerr << "path azimuth: " << path_azimuth_distance.first << " distance: " << path_azimuth_distance.second << std::endl;
            
                error_azimuth = vehicle_azimuth_distance.first - m_segment_azimuth_distances[m_current_segment_index].first;
                sin_error_azimuth = sin(error_azimuth*M_PI/180.0);
                cos_error_azimuth = cos(error_azimuth*M_PI/180.0);

                progress = vehicle_azimuth_distance.second*cos_error_azimuth;
                if (progress >= m_segment_azimuth_distances[m_current_segment_index].second)
                {
                    m_cumulative_distance += m_segment_azimuth_distances[m_current_segment_index].second;
                    m_current_segment_index += 1;
                    // have we reached the last segment?
                    if(m_current_segment_index >= m_current_path.size()-1)
                    {
                        path_follower::path_followerResult result;
                        result.ending_pose.position.latitude = m_current_position.latitude;
                        result.ending_pose.position.longitude = m_current_position.longitude;
                        m_action_server.setSucceeded(result);
                        return;
                    }
                }
                else
                {
                    found_current_segment = true;
                }
            }
            
            double cross_track = vehicle_azimuth_distance.second*sin_error_azimuth;
            //std::cerr << "cross track: " << cross_track << std::endl;
            
            path_follower::path_followerFeedback feedback;
            feedback.percent_complete = (m_cumulative_distance+progress)/m_total_distance;
            feedback.crosstrack_error = cross_track;
            m_action_server.publishFeedback(feedback);
            
            std_msgs::Float64 setpoint;
            setpoint.data = 0.0;
            m_setpoint_pub.publish(setpoint);
            
            std_msgs::Float64 state;
            state.data = cross_track;
            m_state_pub.publish(state);
            
            
            ros::Time now = ros::Time::now();
            
            marine_msgs::NavEulerStamped desired_heading;
            desired_heading.header.stamp = now;
            //desired_heading.orientation.heading = path_azimuth_distance.first - std::max(-90.0,std::min(90.0,cross_track));
            desired_heading.orientation.heading = m_segment_azimuth_distances[m_current_segment_index].first + m_crab_angle;
            m_desired_heading_pub.publish(desired_heading);
            
            geometry_msgs::TwistStamped desired_speed;
            desired_speed.header.stamp = now;
            desired_speed.twist.linear.x = m_current_speed;
            m_desired_speed_pub.publish(desired_speed);
            
        }
    }

    void positionCallback(const geographic_msgs::GeoPointStamped::ConstPtr& inmsg)
    {
        m_current_position.latitude = inmsg->position.latitude;
        m_current_position.longitude = inmsg->position.longitude;
        sendDesired();
    }

    void headingCallback(const marine_msgs::NavEulerStamped::ConstPtr& inmsg)
    {
        m_current_heading = inmsg->orientation.heading;
    }
    
    
private:
    ros::NodeHandle m_node_handle;
    actionlib::SimpleActionServer<path_follower::path_followerAction> m_action_server;

    std::vector<LatLong> m_current_path;
    std::vector<std::pair<double, double> > m_segment_azimuth_distances;
    int m_current_segment_index;
    LatLong m_current_position;

    double m_current_speed;
    double m_current_heading;

    double m_crab_angle;
    
    double m_total_distance; // total distance of complete path in meters.
    double m_cumulative_distance; // distance of completed segments in meters.

    ros::Publisher m_desired_speed_pub;
    ros::Publisher m_desired_heading_pub;
    ros::Publisher m_state_pub;
    ros::Publisher m_setpoint_pub;
    
    ros::Subscriber m_position_sub;
    ros::Subscriber m_heading_sub;
    ros::Subscriber m_control_effort_pub;
};





int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_follower");

    PathFollower pf("path_follower_action");
    
    ros::spin();
    
    return 0;
}

    
