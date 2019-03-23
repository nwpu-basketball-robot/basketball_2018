#include <iostream>
#include <ros/ros.h>
#include "basketball_msgs/move_to_point.h"
#include <opencv2/opencv.hpp>
#include <unistd.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "remote_controler") ;
    ros::NodeHandle nh ;
    basketball_msgs::move_to_point move_srv ;
    double x_dis{0.0}, y_dis{0.0}, yaw_ang{0.0} ;

    ros::ServiceClient ControlClient = nh.serviceClient<basketball_msgs::move_to_point>("/robot_move/world_locate_srv") ;
    char c ;
    cv::imshow("control_window",0) ;
    c = cv::waitKey(20) ;
    while( c != 'c' )
    {
        switch(c)
        {
        case 'w' :
            y_dis += 0.04 ;
            break ;
        case 'a' :
            x_dis -= 0.04 ;
            break ;
        case 's' :
            y_dis -= 0.04 ;
            break ;
        case 'd' :
            x_dis += 0.04 ;
            break ;
        case 'q' :
            yaw_ang += 0.03 ;
            break ;
        case 'e' :
            yaw_ang -= 0.03 ;
            break ;
        default:
            break ;
        }
        if(yaw_ang < 0) yaw_ang += 360.0 ;
        move_srv.request.x_goal = x_dis ;
        move_srv.request.y_goal = y_dis ;
        move_srv.request.z_goal = yaw_ang ;
        ControlClient.call(move_srv.request, move_srv.response) ;
        c = cv::waitKey(20) ;
    }
    

    
    return 0;
}

