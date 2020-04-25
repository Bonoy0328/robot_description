#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "robot_tf_broadcaster");
    ros::NodeHandle nh;
    tf::Quaternion imu_quat,fix_quat;
    imu_quat = tf::createQuaternionFromYaw(3.1415926/2.0);
    fix_quat = tf::createQuaternionFromYaw(0.0);
    static tf::TransformBroadcaster br1;
    tf::Transform tf_baseLink,tf_cameraLink,tf_imuLink,tf_laserLink,tf_odom;
    tf_imuLink.setOrigin(tf::Vector3(0,0,0.1));
    tf_imuLink.setRotation(imu_quat);
    tf_baseLink.setOrigin(tf::Vector3(0,0,0.2));
    tf_baseLink.setRotation(fix_quat);
    tf_cameraLink.setOrigin(tf::Vector3(0,0,0.3));
    tf_cameraLink.setRotation(fix_quat);
    tf_laserLink.setOrigin(tf::Vector3(0,0,0.4));
    tf_laserLink.setRotation(fix_quat);
    tf_odom.setOrigin(tf::Vector3(0,0,0.1));
    tf_odom.setRotation(fix_quat);
    ros::Rate r(1000);
    while(nh.ok()){
        br1.sendTransform(tf::StampedTransform(tf_imuLink,ros::Time::now(),"base_link","imu_link"));
        br1.sendTransform(tf::StampedTransform(tf_baseLink,ros::Time::now(),"base_footprint","base_link"));
        br1.sendTransform(tf::StampedTransform(tf_cameraLink,ros::Time::now(),"base_link","camera_link"));
        br1.sendTransform(tf::StampedTransform(tf_laserLink,ros::Time::now(),"base_link","laser_link"));
        //  br1.sendTransform(tf::StampedTransform(tf_odom,ros::Time::now(),"odom_ekf","odom_combined"));
        r.sleep();
    }
    return 0;
}