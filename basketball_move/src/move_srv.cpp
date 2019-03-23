/*
 * @brief robot move service with simple collvoid ability
 * @version 1.2
 * @author XiaMo
 * @First Debug Date 2018-05-16
 */
#include "robot_move_pkg/move_srv.h"
#include <unistd.h>
#define PI 3.14159265358

MovetoPoint::MovetoPoint(ros::NodeHandle &node)
:mtp_nh(node),
 AchiveHomePoint(false),
 moveable(true),
 ReturnHome(false)
{
    move_to_point       = mtp_nh.advertiseService("/robot_move/world_locate_srv", &MovetoPoint::MTPServiceCallBack, this) ;
    robot_rotate        = mtp_nh.advertiseService("/robot_move/rotate_cmd_srv", &MovetoPoint::ROTServiceCallBack, this) ;
    focus_target        = mtp_nh.advertiseService("/robot_move/focus_target_srv", &MovetoPoint::TGTServiceCallBack, this) ;
    world_locate_puber  = mtp_nh.advertise<geometry_msgs::Twist>("/robot_move/world_locate", 100) ;
    robot_rotate_puber  = mtp_nh.advertise<geometry_msgs::Twist>("/robot_move/rotate_cmd", 100) ;
    focus_target_puber  = mtp_nh.advertise<geometry_msgs::Twist>("/robot_move/focus_target", 100) ;
    odom_subscriber     = mtp_nh.subscribe("odom", 100, &MovetoPoint::OdomSubCallBack, this) ;
    StartTime          = ros::Time::now() ;
    obs_subscriber      = mtp_nh.subscribe("/RecvData/6", 10,  &MovetoPoint::ObsSubCallBack, this) ;
}

MovetoPoint::~MovetoPoint()
{
    mtp_nh.shutdown();
}

void MovetoPoint::OdomSubCallBack(const nav_msgs::Odometry::ConstPtr &ptr)
{
    ROS_WARN("Move Server Recv Odom Messages!!!\n") ;
    this->odom.pose.pose.position.x = ptr->pose.pose.position.x ;
    this->odom.pose.pose.position.y = ptr->pose.pose.position.y ;
    this->odom.pose.pose.position.z = ptr->pose.pose.position.z ;
    //ROS_WARN("now x, y, z = %lf, %lf, %lf ", odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z) ;
}

/*
 * @brief Recv the obstacle messages from slaver computer and store
 */
void MovetoPoint::ObsSubCallBack(const basketball_msgs::robot_state::ConstPtr &ptr)
{
    obstacle.detectTime = ros::Time::now() ;
    obstacle.x          = ptr->data.at(0) ;
    obstacle.y          = ptr->data.at(1) ;
}

/*
 * @brief move to a point in world coordinate
 */
bool MovetoPoint::MTPServiceCallBack(basketball_msgs::move_to_point::Request &req,
                                     basketball_msgs::move_to_point::Response &rep)
{
    geometry_msgs::Twist dest_point ;
    dest_point.linear.x     = req.x_goal ;
    dest_point.linear.y     = req.y_goal ;
    dest_point.angular.z    = req.z_goal ;
    odom_listener.lookupTransform("odom", "base_link", ros::Time(), odom_tf) ;
    ROS_WARN("Moving and ok = %d, now x, y, z = %lf, %lf, %lf ", (int)rep.ok, dest_point.linear.x, dest_point.linear.y, dest_point.angular.z) ;
    //ROS_WARN("goal x, y, z = %lf, %lf, %lf ", req.x_goal, req.y_goal, req.z_goal) ;
    ros::Time CurrentTime ;
    CurrentTime = ros::Time::now() ;
    if(CurrentTime.toSec() - StartTime.toSec() > 1200.0)
    {
        moveable = false ;
        ReturnHome = true ;
    }
    if(CurrentTime.toSec() - obstacle.detectTime.toSec() < 0.1)
    {
        moveable = false ;
        ReturnHome = false ;
    }
    else
    {
        moveable = true ;
    }
    if(moveable){
        if( std::abs(odom.pose.pose.position.x - req.x_goal) > 0.04 || std::abs(odom.pose.pose.position.y - req.y_goal) > 0.04 || (std::abs(odom.pose.pose.position.z - req.z_goal) > 0.03 && std::abs(std::abs(odom.pose.pose.position.z - req.z_goal) - 2*PI) > 0.03))
        {
            //ROS_WARN("Moving and ok = %d, now x, y, z = %lf, %lf, %lf ", (int)rep.ok, odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z) ;
            //ROS_WARN("goal x, y, z = %lf, %lf, %lf ", req.x_goal, req.y_goal, req.z_goal) ;
            rep.ok = false ;
    /*        if(std::fabs(dest_point.linear.x) > 10.0)
                dest_point.linear.x = 10.0 ;
            if(std::fabs(dest_point.linear.y) > 4.2)
                dest_point.linear.y = 4.2 ;
            if(dest_point.linear.y < 1.5)
                dest_point.linear.y = 1.5 ;*/
            world_locate_puber.publish(dest_point) ;
        }
        else rep.ok = true ;
    }
    else{
        // If timeout
        if(ReturnHome){
            dest_point.linear.x     = 0 ;
            dest_point.linear.y     = 1.8 ;
            dest_point.angular.z    = 0 ;
            world_locate_puber.publish(dest_point) ;
            if( (std::abs(odom.pose.pose.position.x - 0) > 0.04 || std::abs(odom.pose.pose.position.y - 1.8) > 0.04 || (std::abs(odom.pose.pose.position.z - 0) > 0.02 && std::abs(std::abs(odom.pose.pose.position.z - 0) - 2*PI) > 0.02)) && !AchiveHomePoint)
            {
                dest_point.linear.x     = 0 ;
                dest_point.linear.y     = 1.8 ;
                dest_point.angular.z    = 0 ;
                world_locate_puber.publish(dest_point) ;
            }
            else{
                AchiveHomePoint = true ;
                dest_point.linear.x     = 0 ;
                dest_point.linear.y     = 0 ;
                dest_point.angular.z    = 0 ;
                world_locate_puber.publish(dest_point) ;
            }
        }
        // If obstacle detected
        else{
            double obstacle_world_x     = odom.pose.pose.position.x + obstacle.x*std::cos(odom.pose.pose.position.z) - obstacle.y*std::sin(odom.pose.pose.position.z) ;
            double obstacle_world_y     = odom.pose.pose.position.y + obstacle.x*std::sin(odom.pose.pose.position.z) + obstacle.y*std::cos(odom.pose.pose.position.z) ;
            //if()
        }
    }
    
    return true ;
}

bool MovetoPoint::ROTServiceCallBack(basketball_msgs::robot_rotate::Request &req,
                                     basketball_msgs::robot_rotate::Response &rep)
{
    geometry_msgs::Twist rotate_cmd ;
    rotate_cmd.linear.x     = req.long_axis ;
    rotate_cmd.linear.y     = req.short_axis ;
    rotate_cmd.linear.z     = req.revo_speed ;
    rotate_cmd.angular.x    = req.center_x ;
    rotate_cmd.angular.y    = req.center_y ;
    rotate_cmd.angular.z    = req.rota_speed ;
    robot_rotate_puber.publish(rotate_cmd) ;
    ROS_ERROR("Rotate cmd called\n") ;
    rep.ok                  = true ;
    return true ;
}

bool MovetoPoint::TGTServiceCallBack(basketball_msgs::focus_target::Request &req,
                                     basketball_msgs::focus_target::Response &rep)
{
    geometry_msgs::Twist    target_location ;
    target_location.linear.x    = req.relative_x ;
    target_location.linear.y    = req.relative_y ;
    focus_target_puber.publish(target_location) ;
    rep.ok                      = true ;
    return true ;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MovetoPointService") ;
    ros::NodeHandle node ;
    MovetoPoint MoveSrv(node) ;
    ros::spin() ;
}
