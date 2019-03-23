//author : rescuer liao
//date : 2015-7-13
//this package get the request from some client and control the shoot component of robot
#include <ros/ros.h>
#include <basketball_msgs/basketball_shoot_srv.h>
#include <basketball_msgs/robot_message.h>

class RobotShootSrv
{
  public:
    RobotShootSrv(ros::NodeHandle &node) ;
    ~RobotShootSrv() ;
protected:
private:
    void waitForShootOver()
    {
	ros::Rate r(after_shoot_sleep_time_) ; 
        r.sleep() ;
    }
    bool robotShootSrvCallBack(basketball_msgs::basketball_shoot_srv::Request &req ,
                               basketball_msgs::basketball_shoot_srv::Response &rep)  ;

    bool pubShootCmd(const uint8_t func) ;
private:
    ros::Publisher robot_shoot_pub_ ;
    ros::ServiceServer robot_shoot_srv_ ;
    ros::NodeHandle nh_ ;
    int after_shoot_sleep_time_ ;

    uint8_t shoot_cmd_id_ ; 
};


RobotShootSrv::RobotShootSrv(ros::NodeHandle &node)
    :nh_(node),
    shoot_cmd_id_(0x02)
{
    nh_.param("after_shoot_sleep_time",after_shoot_sleep_time_ , 1) ; 
    robot_shoot_pub_ =  nh_.advertise<basketball_msgs::robot_message>("robot_cmd",1000);
    robot_shoot_srv_ = nh_.advertiseService("cmd_shoot",&RobotShootSrv::robotShootSrvCallBack,this) ;
}

RobotShootSrv::~RobotShootSrv()
{
    nh_.shutdown() ;
}

bool RobotShootSrv::robotShootSrvCallBack(basketball_msgs::basketball_shoot_srv::Request &req ,
                                       basketball_msgs::basketball_shoot_srv::Response &rep)
{
    rep.is_successed = pubShootCmd(0x04) ;
    return true ;
}

bool RobotShootSrv::pubShootCmd(const uint8_t func)
{
    basketball_msgs::robot_message robot_cmd_msg ; 
    robot_cmd_msg.data.resize(6 , 0) ; 
    uint8_t *data_ptr = robot_cmd_msg.data.data() ; 
    int data_len = 1 ; 
    data_ptr[0] = data_ptr[1] = 0xff ; 
    data_ptr[2] = shoot_cmd_id_ ; 
    data_ptr[3] = (u_int8_t)(data_len>>8) ;
    data_ptr[4] = (u_int8_t)(data_len & 0xff) ;
    data_ptr[5] = func ;
    robot_shoot_pub_.publish(robot_cmd_msg) ;
    waitForShootOver() ;
    return true ;
}

int main(int argc ,char **argv)
{
    ros::init(argc ,argv , "shoot_service") ;
    ros::NodeHandle node ;
    RobotShootSrv robot_shoot_srv(node) ;
    ros::spin() ;
}
