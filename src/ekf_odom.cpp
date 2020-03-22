#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
class SubAndPub
{
public:
    SubAndPub(){
        pub_ = n_.advertise<nav_msgs::Odometry>("odom_ekf", 1000);
        sub_ = n_.subscribe("/robot_pose_ekf/odom_combined", 1000, &SubAndPub::callback,this);
    }
    void callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
        // tf::TransformBroadcaster odom_broadcaster_;
        ros::Time current_time_;
        // geometry_msgs::TransformStamped odom_trans;
        current_time_ = ros::Time::now();
        // odom_trans.header.stamp = current_time_;
        // odom_trans.header.frame_id = "odom_ekf";
        // odom_trans.child_frame_id  = "base_footprint";

        // geometry_msgs::Quaternion odom_quat;
        // odom_quat = msg->pose.pose.orientation;
        // odom_trans.transform.translation.x = msg->pose.pose.position.x;
        // odom_trans.transform.translation.y = msg->pose.pose.position.y;
        // odom_trans.transform.translation.z = msg->pose.pose.position.z;
        // odom_trans.transform.rotation = odom_quat;
        // odom_broadcaster_.sendTransform(odom_trans);
        nav_msgs::Odometry msgl;
        msgl.header.stamp = current_time_;
        msgl.header.frame_id = "odom_ekf";
		msgl.pose = msg->pose;

        pub_.publish(msgl);
		

        // ROS_INFO("jinruhuidiao");
    }
private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
};



// void chatterCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
// {
    
//     ros::NodeHandle nh;
//     ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("odom_ekf", 1000);
//     tf::TransformBroadcaster odom_broadcaster_;
//     ros::Time current_time_;
//     geometry_msgs::TransformStamped odom_trans;
//     current_time_ = ros::Time::now();
//     odom_trans.header.stamp = current_time_;
//     odom_trans.header.frame_id = "odom_ekf";
//     odom_trans.child_frame_id  = "base_footprint";

//     geometry_msgs::Quaternion odom_quat;
//     odom_quat = msg->pose.pose.orientation;
//     odom_trans.transform.translation.x = msg->pose.pose.position.x;
//     odom_trans.transform.translation.y = msg->pose.pose.position.y;
//     odom_trans.transform.translation.z = msg->pose.pose.position.z;
//     odom_trans.transform.rotation = odom_quat;
//     odom_broadcaster_.sendTransform(odom_trans);

//     nav_msgs::Odometry msgl;
// 	msgl.header.stamp = current_time_;
// 	msgl.header.frame_id = "odom_ekf";

// 	msgl.pose.pose.position.x = msg->pose.pose.position.x;
// 	msgl.pose.pose.position.y = msg->pose.pose.position.y;
// 	msgl.pose.pose.position.z = msg->pose.pose.position.z;
// 	msgl.pose.pose.orientation = odom_quat;
// 	msgl.pose.covariance = msg->pose.covariance;

//     pub.publish(msgl);
//     ROS_INFO("jinruhuidiao");
// }

int main(int argc, char *argv[])
{
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
	ros::init(argc, argv, "ekf_odom_pub");
	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	// ros::NodeHandle nh;

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
	// ros::Subscriber sub = nh.subscribe("/robot_pose_ekf/odom_combined", 1000, chatterCallback);
    
	/**
	/navigation_velocity_smoother/raw_cmd_vel
	 * ros::spin() will enter a loop, pumping callbacks.  With this version, all
	 * callbacks will be called from within this thread (the main one).  ros::spin()
	 * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
	 */
     SubAndPub subAndPubObject;
	ros::spin();

	return 0;
}