#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"

#include <sstream>
#include "serial/serial.h"
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char *argv[])
{
	serial::Serial sp;
	serial::Timeout to = serial::Timeout::simpleTimeout(500);
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
	ros::init(argc, argv, "odometry_publisher");
	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle nh;
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
	tf::TransformBroadcaster odom_broadcaster;
	double dist,oldist;
	double x = 0.0;
	double y = 0.0;
	double th = 0.0;

	double dx = 0.0;
	double dy = 0.0;
	double dth = 0.0;
	
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	/**
	 * The advertise() function is how you tell ROS that you want to
	 * publish on a given topic name. This invokes a call to the ROS
	 * master node, which keeps a registry of who is publishing and who
	 * is subscribing. After this advertise() call is made, the master
	 * node will notify anyone who is trying to subscribe to this topic name,
	 * and they will in turn negotiate a peer-to-peer connection with this
	 * node.  advertise() returns a Publisher object which allows you to
	 * publish messages on that topic through a call to publish().  Once
	 * all copies of the returned Publisher object are destroyed, the topic
	 * will be automatically unadvertised.
	 *
	 * The second parameter to advertise() is the size of the message queue
	 * used for publishing messages.  If messages are published more quickly
	 * than we can send them, the number here specifies how many messages to
	 * buffer up before throwing some away.
	 */
	//ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

	ros::Rate loop_rate(20);

	/**
	 * A count of how many messages we have sent. This is used to create
	 * a unique string for each message.
	 */
	uint8_t flag = 67;//0x43
	int count = 0;
	int fre1=0,fre2=0,odom1=0,odom2=0;
	double angle_th=0.0,old_angle_th=0.0;
	while (nh.ok())
	{
		/**
		 * This is a message object. You stuff it with data, and then publish it.
		 */
		 //get data
		if(sp.isOpen()){
			 sp.write(&flag,1);
			 size_t n = sp.available();
			 if(n!=0){
				 uint8_t buffer[1024];
				try{
					n = sp.read(buffer,n);
				}catch(serial::SerialException& e){
				ROS_ERROR_STREAM("Unable to read data");			
				}
				 
				//  for(int i=0;i<n;i++){
				// 	 std::cout << std::hex << (buffer[i] & 0xff) << " ";
				//  }
				//  std::cout << std::endl;
				if(n==17){
					// fre1 = buffer[1] + ((int)buffer[2]<<8) + ((int)buffer[3] << 16) + ((int)buffer[4] << 24);
					// fre2 = buffer[5] + ((int)buffer[6]<<8) + ((int)buffer[7] << 16) + ((int)buffer[8] << 24);
					// odom1 = buffer[9] + ((int)buffer[10]<<8) + ((int)buffer[11] << 16) + ((int)buffer[12] << 24);
					// odom2 = buffer[13] + ((int)buffer[14]<<8) + ((int)buffer[15] << 16) + ((int)buffer[16] << 24);
					// angle_th = (buffer[17] + ((long)buffer[18]<<8) + ((long)buffer[19] << 16) + ((long)buffer[20] << 24) + ((long)buffer[21] << 32) + ((long)buffer[22] << 40) + ((long)buffer[23] << 48) + ((long)buffer[24] << 56))/1000.0;
					// if(buffer[25]==13){
					// 	angle_th = 0-angle_th;
					// }
					// ROS_INFO("%d %d %d %d %f",fre1,fre2,odom1,odom2,angle_th);
					// for(int i=0;i<n;i++){
					// 	std::cout << std::hex << (buffer[i] & 0xff) << " ";
					// }
					// std::cout << std::endl;
					
				}
			 }
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
		 dist = ((odom1/336.0 + odom2/336.0)/2.0);
		 //
		current_time = ros::Time::now();
		double dt = (current_time - last_time).toSec();
		angle_th = (angle_th*3.1415926)/180.0;
		dx = (dist - oldist)*cos(angle_th)*0.2104867042;
		dy = (dist - oldist)*sin(angle_th)*0.2104867042;
		dth = angle_th - old_angle_th;
		x += dx;
		y += dy;
		oldist = dist;
		old_angle_th = angle_th;

		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(angle_th);
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = ros::Time::now();
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_footprint";

		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		//send the transform
		odom_broadcaster.sendTransform(odom_trans);

		//publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";

		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;
		
		 odom.child_frame_id = "base_link";
		 odom.twist.twist.linear.x = dx/dt;
		 odom.twist.twist.linear.y = dy/dt;
		 odom.twist.twist.angular.z = dth/dt;

		odom_pub.publish(odom);
		last_time = current_time;
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
