#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serial/serial.h"
#include "geometry_msgs/Twist.h"
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
int oldM = 0,M=0,oldA=0,A=0;
uint8_t  buff[9];
serial::Serial sp;
serial::Timeout to = serial::Timeout::simpleTimeout(500);
void chatterCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	if(sp.isOpen()){
		M = ((msg->linear.x)*336.0/(3.1415926*0.095));
		A = ((msg->angular.z)*1800.0/3.1415926);
		//sp.write(buff,4);
		if((M!=oldM)|(A!=oldA)){
			oldA = A;
			oldM = M;
			buff[0]=0x0a;
			for(int i=1;i<5;i++)buff[i] =  (M>>8*(4-i));
			for(int i=5;i<9;i++)buff[i] =  (A>>8*(8-i));
			sp.write(buff,9);
		}
		ROS_INFO("%d  %d",M,A);
	}else{
		try
		{
			sp.open();

		}
		catch (serial::IOException& e)
		{
			/* code for Catch */
			ROS_ERROR_STREAM("Unalble to open port.");
		}
	}
}

int main(int argc, char *argv[])
{

	sp.setPort("/dev/ttyUSB0");
	sp.setBaudrate(115200);
	sp.setTimeout(to);
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "debug");
	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle n;

	/**
	 * The subscribe() call is how you tell ROS that you want to receive messages
	 * on a given topic.  This invokes a call to the ROS
	 * master node, which keeps a registry of who is publishing and who
	 * is subscribing.  Messages are passed to a callback function, here
	 * called chatterCallback.  subscribe() returns a Subscriber object that you
	 * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
	 * object go out of scope, this callback will automatically be unsubscribed from
	 * this topic.
	 *
	 * The second parameter to the subscribe() function is the size of the message
	 * queue.  If messages are arriving faster than they are being processed, this
	 * is the number of messages that will be buffered up before beginning to throw
	 * away the oldest ones.
	 */
	ros::Subscriber sub = n.subscribe("/cmd_vel", 1000, chatterCallback);

	/**
	/navigation_velocity_smoother/raw_cmd_vel
	 * ros::spin() will enter a loop, pumping callbacks.  With this version, all
	 * callbacks will be called from within this thread (the main one).  ros::spin()
	 * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
	 */
	ros::spin();

	return 0;
}