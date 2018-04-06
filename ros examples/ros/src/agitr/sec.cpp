// This is a ROS version of the standard "hello,world" program .

// This header defines the standard ROS classes .
#include <ros/ros.h>

int main(int argc ,char **argv)
{
	//Initialize the ros system
    ros::init(argc,argv,"hello_ros");
    
	//Enstablish this program as a node
    ros::NodeHandle nh;

	//Send some output as a long message
    ROS_INFO_STREAM("second");
}
