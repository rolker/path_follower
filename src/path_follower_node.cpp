#include "ros/ros.h"
#include "marine_msgs/Heartbeat.h"
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

ros::Publisher desired_speed_pub;
ros::Publisher desired_heading_pub;
ros::Publisher state_pub;
ros::Publisher setpoint_pub;


struct LatLong
{
    double latitude;
    double longitude;
};

std::vector<LatLong> current_path;
LatLong current_position;

double current_speed;
double current_heading;

double crab_angle;

void currentSpeedCallback(const std_msgs::Float32::ConstPtr& inmsg)
{
    current_speed = inmsg->data;
}

void currentPathCallback(const geographic_msgs::GeoPath::ConstPtr& inmsg)
{
    current_path.clear();
    for(auto pose: inmsg->poses)
    {
        LatLong ll;
        ll.latitude = pose.pose.position.latitude;
        ll.longitude = pose.pose.position.longitude;
        current_path.push_back(ll);
    }
}

void controlEfforCallback(const std_msgs::Float64::ConstPtr& inmsg)
{
    crab_angle = inmsg->data;
}

void sendDesired()
{
    if(!current_path.empty())
    {
        gz4d::geo::Point<double,gz4d::geo::WGS84::LatLon> p1, p2,vehicle_position;
        p1[0] = current_path[0].latitude;
        p1[1] = current_path[0].longitude;
        p2[0] = current_path[1].latitude;
        p2[1] = current_path[1].longitude;
        std::cerr << "p1: " << p1[0] << "," << p1[1] << " p2: " << p2[0] << "," << p2[1] << std::endl;
        
        vehicle_position[0] = current_position.latitude;
        vehicle_position[1] = current_position.longitude;
                
        auto path_azimuth_distance = gz4d::geo::WGS84::Ellipsoid::inverse(p1,p2);
        auto vehicle_azimuth_distance = gz4d::geo::WGS84::Ellipsoid::inverse(p1,vehicle_position);

        std::cerr << "path azimuth: " << path_azimuth_distance.first << " distance: " << path_azimuth_distance.second << std::endl;
        
        double error_azimuth = vehicle_azimuth_distance.first - path_azimuth_distance.first;
        double sin_error_azimuth = sin(error_azimuth*M_PI/180.0);
        double cos_error_azimuth = cos(error_azimuth*M_PI/180.0);
        
        double cross_track = vehicle_azimuth_distance.second*sin_error_azimuth;
        std::cerr << "cross track: " << cross_track << std::endl;
        
        std_msgs::Float64 setpoint;
        setpoint.data = 0.0;
        setpoint_pub.publish(setpoint);
        
        std_msgs::Float64 state;
        state.data = cross_track;
        state_pub.publish(state);
        
        double progress = vehicle_azimuth_distance.second*cos_error_azimuth;
        std::cerr << "progress: " << progress << std::endl;
        
        ros::Time now = ros::Time::now();
        
        marine_msgs::NavEulerStamped desired_heading;
        desired_heading.header.stamp = now;
        //desired_heading.orientation.heading = path_azimuth_distance.first - std::max(-90.0,std::min(90.0,cross_track));
        desired_heading.orientation.heading = path_azimuth_distance.first + crab_angle;
        desired_heading_pub.publish(desired_heading);
        
        geometry_msgs::TwistStamped desired_speed;
        desired_speed.header.stamp = now;
        desired_speed.twist.linear.x = current_speed;
        desired_speed_pub.publish(desired_speed);
        
    }
}

void positionCallback(const geographic_msgs::GeoPointStamped::ConstPtr& inmsg)
{
    current_position.latitude = inmsg->position.latitude;
    current_position.longitude = inmsg->position.longitude;
    sendDesired();
}

void headingCallback(const marine_msgs::NavEulerStamped::ConstPtr& inmsg)
{
    current_heading = inmsg->orientation.heading;
}

int main(int argc, char **argv)
{
    current_speed = 3.0;
    crab_angle = 0.0;
    
    ros::init(argc, argv, "path_follower");
    ros::NodeHandle n;

    desired_heading_pub = n.advertise<marine_msgs::NavEulerStamped>("/moos/desired_heading",1);
    desired_speed_pub = n.advertise<geometry_msgs::TwistStamped>("/moos/desired_speed",1);
    setpoint_pub = n.advertise<std_msgs::Float64>("/project11/crab_angle/setpoint",1);
    state_pub = n.advertise<std_msgs::Float64>("/project11/crab_angle/state",1);
    
    ros::Subscriber position_sub = n.subscribe("/position",10,positionCallback);
    ros::Subscriber heading_sub = n.subscribe("/heading",10,headingCallback);
    ros::Subscriber current_path_sub = n.subscribe("/project11/mission_manager/current_path",10,currentPathCallback);
    ros::Subscriber current_speed_sub = n.subscribe("/project11/mission_manager/current_speed", 10, currentSpeedCallback);
    ros::Subscriber control_effort_pub = n.subscribe("/project11/crab_angle/control_effort", 10, controlEfforCallback);
    
    ros::spin();
    
    return 0;
}

    
