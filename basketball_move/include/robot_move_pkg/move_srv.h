#include "ros/ros.h"
#include "basketball_msgs/move_to_point.h"
#include "basketball_msgs/robot_rotate.h"
#include "basketball_msgs/focus_target.h"
#include "basketball_msgs/robot_state.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "nav_msgs/Odometry.h"
#include <mutex>

struct ObsMsg{
    ros::Time   detectTime;
    double       x ;
    double       y ;
};

class MovetoPoint
{
public:
    MovetoPoint(ros::NodeHandle &node) ;
    ~MovetoPoint() ;
protected:
private:
    ros::ServiceServer move_to_point ;
    ros::ServiceServer robot_rotate ; 
    ros::ServiceServer focus_target ;
    ros::Publisher world_locate_puber ;
    ros::Publisher robot_rotate_puber ;
    ros::Publisher focus_target_puber ;
    ros::Subscriber odom_subscriber ;
    ros::Subscriber obs_subscriber ;
    ros::NodeHandle mtp_nh ;
    tf::TransformListener odom_listener ;
    tf::StampedTransform odom_tf ;
    nav_msgs::Odometry odom ;
    ObsMsg obstacle ;
    std::mutex odom_mutex ;
    std::mutex chassis_mutex ;
    ros::Time StartTime ;
    bool AchiveHomePoint ;
    bool moveable ;
    bool ReturnHome ;
    bool MTPServiceCallBack(basketball_msgs::move_to_point::Request&, basketball_msgs::move_to_point::Response&) ;
    bool ROTServiceCallBack(basketball_msgs::robot_rotate::Request&, basketball_msgs::robot_rotate::Response&) ;
    bool TGTServiceCallBack(basketball_msgs::focus_target::Request&, basketball_msgs::focus_target::Response&) ;
    void ObsSubCallBack(const basketball_msgs::robot_state::ConstPtr&) ;
    void OdomSubCallBack(const nav_msgs::Odometry::ConstPtr&) ;
};
