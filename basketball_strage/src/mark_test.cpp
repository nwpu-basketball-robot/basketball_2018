#include <basketball_msgs/mark_post_position.h>
#include <basketball_msgs/basketball_position.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>

int main(int argc , char **argv)
{
    ros::init(argc ,argv , "mark_test") ;
    ros::NodeHandle node ;
    ros::Publisher ball_pub = node.advertise<basketball_msgs::basketball_position>("/mark_ball",10) ;
    basketball_msgs::basketball_position marker_info ;
    geometry_msgs::Point p ;
    p.x = 2.0 ;
    p.y = 1.0 ;
    p.z = 0.0 ;
    ros::Rate r(30) ;
    marker_info.basketball_position = p ;
    while(ros::ok())
    {
        ball_pub.publish(marker_info) ;
        r.sleep() ;
    }
}
