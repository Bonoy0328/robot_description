#include "ros/ros.h"
#include "std_msgs/String.h"
#include <algorithm>
#include <sstream>
#include "serial/serial.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include  "eigen3/Eigen/Core"
#include <eigen3/Eigen/Geometry>
#include  "eigen3/Eigen/Dense"

Eigen::Quaterniond q;

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char *argv[])
{
	boost::array<double, 36> odom_pose_covariance = {
    {1e-9, 0, 0, 0, 0, 0, 
    0, 1e-3, 1e-9, 0, 0, 0, 
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0, 
    0, 0, 0, 0, 1e6, 0, 
    0, 0, 0, 0, 0, 1e-9}};
boost::array<double, 36> odom_twist_covariance = {
    {1e-9, 0, 0, 0, 0, 0, 
    0, 1e-3, 1e-9, 0, 0, 0, 
    0, 0, 1e6, 0, 0, 0, 
    0, 0, 0, 1e6, 0, 0, 
    0, 0, 0, 0, 1e6, 0, 
    0, 0, 0, 0, 0, 1e-9}};
	// double q;
	double ax=0.0,ay=0.0,az=0.0,wx=0.0,wy=0.0,wz=0.0,q0=0.0,q1=0.0,q2=0.0,q3=0.0;
	double vx_=0.0,vy_=0.0,vth_=0.0,x_=0.0,y_=0.0,th_=0.0;
	int odom_left=0,odom_right=0,vec_left=0,vec_right=0,vec_leftOld=0,vec_rightOld=0;
	ros::Time current_time, last_time,current_time_,last_time_,curr_time;
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
	ros::init(argc, argv, "IMU");

	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle nh;

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
	ros::Publisher IMU_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 1000);
	ros::Rate loop_rate(50);
	ros::Publisher pub_ = nh.advertise<nav_msgs::Odometry>("odom", 50);
	tf::TransformBroadcaster odom_broadcaster_;
	current_time_ = ros::Time::now();
	last_time_ = ros::Time::now();
	curr_time = ros::Time::now();
	last_time = ros::Time::now();
	/**
	 * A count of how many messages we have sent. This is used to create
	 * a unique string for each message.
	 */
	uint8_t flag[2] = {67,68};//0x43 0x44
	int count = 0,cnt1=0,cnt2=0;
	while (ros::ok())
	{
		if(sp.isOpen()){
		// ROS_INFO("Serial is Open");
		sp.write(flag,2);
		size_t n = sp.available();
		if(n!=0){
			uint8_t buffer[1024];
		try{
			n = sp.read(buffer,n);
		}catch(serial::SerialException& e){
		ROS_ERROR_STREAM("Unable to read data");			
		}
		
		if(n==51){
			vec_left = (int32_t)((uint32_t)buffer[6]|((uint32_t)buffer[7])<<8|((uint32_t)buffer[8])<<16|((uint32_t)buffer[9])<<24);
			vec_right = (int32_t)((uint32_t)buffer[2]|((uint32_t)buffer[3])<<8|((uint32_t)buffer[4])<<16|((uint32_t)buffer[5])<<24);
			odom_left = (int32_t)((uint32_t)buffer[14]|((uint32_t)buffer[15])<<8|((uint32_t)buffer[16])<<16|((uint32_t)buffer[17])<<24);
			odom_right = (int32_t)((uint32_t)buffer[10]|((uint32_t)buffer[11])<<8|((uint32_t)buffer[12])<<16|((uint32_t)buffer[13])<<24);
			if(odom_left==vec_leftOld)cnt1++;
			else cnt1=0;
			if(odom_right==vec_rightOld)cnt2++;
			else cnt2=0;
			if(cnt1>=5)vec_left=0;
			if(cnt2>=5)vec_right=0;
			vec_leftOld = odom_left;
			vec_rightOld = odom_right;
			ax = (int16_t)((((uint16_t)buffer[21])<<8)|buffer[20])/32768.0*16*9.8;
			ay = (int16_t)((((uint16_t)buffer[23])<<8)|buffer[22])/32768.0*16*9.8;
			az = (int16_t)((((uint16_t)buffer[25])<<8)|buffer[24])/32768.0*16*9.8;
			wx = (int16_t)((((uint16_t)buffer[32])<<8)|buffer[31])/32768.0*2000;
			wy = (int16_t)((((uint16_t)buffer[34])<<8)|buffer[33])/32768.0*2000;
			wz = (int16_t)((((uint16_t)buffer[36])<<8)|buffer[35])/32768.0*2000;
			q0 = (int16_t)((((uint16_t)buffer[43])<<8)|buffer[42])/32768.0;
			q1 = (int16_t)((((uint16_t)buffer[45])<<8)|buffer[44])/32768.0;
			q2 = (int16_t)((((uint16_t)buffer[47])<<8)|buffer[46])/32768.0;
			q3 = (int16_t)((((uint16_t)buffer[49])<<8)|buffer[48])/32768.0;
		}
		}
		}else{
			ROS_INFO("Serial is not open");
			try
			{
				sp.open();
			}
			catch (serial::IOException& e)
			{
				/* code for Catch */
				// ROS_ERROR_STREAM("Unalble to open port.");
			}
		}	
		
		// double dt = (curr_time - last_time).toSec();
		// vx_ += ax*dt;
		// vy_	+= ay*dt;
		// x_ += vx_*dt;
		// y_ +=vy_*dt;
		// ROS_INFO("x=%2.9f y=%2.9f",x_,y_);
		sensor_msgs::Imu imu_data;
		imu_data.header.stamp = ros::Time::now();
		imu_data.header.frame_id = "imu_frame";
<<<<<<< HEAD
		q = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
		imu_data.orientation.x = q1;
		imu_data.orientation.y = q2;
		imu_data.orientation.z = q3;
		imu_data.orientation.w = q0;
=======
		// q = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
		q = Quaterniond(q0,q1,q2,q3,q4);
		q.normalized();
		imu_data.orientation.x = q.x();
		imu_data.orientation.y = q.y();
		imu_data.orientation.z = q.z();
		imu_data.orientation.w = q.w();
>>>>>>> 1169346d14ff4f90553a169f3cc3646a0b839a1f

		imu_data.linear_acceleration.x = ax;
		imu_data.linear_acceleration.y = ay;
		imu_data.linear_acceleration.z = az;

		imu_data.angular_velocity.x = wx;
		imu_data.angular_velocity.y = wy;
		imu_data.angular_velocity.z = wz;
		IMU_pub.publish(imu_data);
		// vx_ = (vec_left + vec_right)/(2.0*336.0)*3.1415926*0.095;
		// vth_ = ((vec_right - vec_left)/336.0*3.1415926*0.095)/0.0228;
		// curr_time = ros::Time::now();
		// double dt = (curr_time - last_time).toSec();
   		// double delta_x = (vx_ * cos(th_) - vy_ * sin(th_)) * dt;
    	// double delta_y = (vx_ * sin(th_) + vy_ * cos(th_)) * dt;
    	// double delta_th = vth_ * dt;
		// x_ += delta_x;
		// y_ += delta_y;
		// th_ += delta_th;
		// geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_);
		// geometry_msgs::TransformStamped odom_trans;
		// odom_trans.header.stamp = current_time;
		// odom_trans.header.frame_id = "odom";
		// odom_trans.child_frame_id = "base_link";

		// odom_trans.transform.translation.x = x_;
		// odom_trans.transform.translation.y = y_;
		// odom_trans.transform.translation.z = 0.0;
		// odom_trans.transform.rotation = odom_quat;

		// //send the transform
		// odom_broadcaster_.sendTransform(odom_trans);

		// //next, we'll publish the odometry message over ROS
		// nav_msgs::Odometry odom;
		// odom.header.stamp = current_time;
		// odom.header.frame_id = "odom";

		// //set the position
		// odom.pose.pose.position.x = x_;
		// odom.pose.pose.position.y = y_;
		// odom.pose.pose.position.z = 0.0;
		// odom.pose.pose.orientation = odom_quat;
		// odom.pose.covariance = odom_pose_covariance;
		// //set the velocity
		// odom.child_frame_id = "base_link";
		// odom.twist.twist.linear.x = vx_;
		// odom.twist.twist.linear.y = vy_;
		// odom.twist.twist.angular.z = vth_;
		// odom.twist.covariance = odom_twist_covariance;

		// //publish the message
		// pub_.publish(odom);
		// last_time = curr_time;
		


		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}