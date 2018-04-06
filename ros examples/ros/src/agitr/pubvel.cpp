#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cstdlib>

int main(int argc, char* argv[])
{
    //initialize the ros system and become a node 
    ros::init(argc, argv, "publish_velocity");
    ros::NodeHandle nh;

    //create a publisher object
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>
			("turtle1/cmd_vel", 1000);

    //seed the random number generator
    srand(time(0));

    //loop at 2hz until the node is shut down
    ros::Rate rate(2);
    while(ros::ok())
    {
	geometry_msgs::Twist msg;
	msg.linear.x = double(rand())/double(RAND_MAX);
	msg.angular.z = 2*double(rand())/double(RAND_MAX) - 1;

	//publish the message
	pub.publish(msg);

	//send a message to ros::out
	ROS_INFO_STREAM("Sending random velocity command: "
			<< "linear= " << msg.linear.x
			<< "angualr= " << msg.angular.z);
	rate.sleep();
    }
}
