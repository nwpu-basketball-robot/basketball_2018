#include <basketball_base_serial/SerialNode.h>

using namespace exp_serial ;

int main(int argc , char **argv)
{
    ros::init(argc , argv , "base_serial") ;
    ros::NodeHandle node ;
    ExpSerial serial(node) ;
    ros::spin() ;
    return 0 ;
}
