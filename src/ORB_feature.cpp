#include "ros/ros.h"    
#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"

void showView(const sensor_msgs::ImageConstPtr msgImg){
    cv_bridge::CvImagePtr cvImagePtr;
    try{
        cvImagePtr = cv_bridge::toCvCopy(msgImg,sensor_msgs::image_encodings::BGR8);
    }catch(cv_bridge::Exception e){
        ROS_ERROR_STREAM("Cv_bridge Exception:" << e.what());
        return;
    }
    cv::Mat cvColorImgMat = cvImagePtr->image;
    cv::Mat cvGrayImgMat;
    cv::cvtColor(cvColorImgMat,cvGrayImgMat,CV_BGR2GRAY);
    cv::imshow("colorview",cvColorImgMat);
    cv::imshow("grayview",cvGrayImgMat);
    cv::waitKey(5);
}
int main(int argc, char const *argv[])
{
    ros::init(argc,argv,"grayView");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    cv::namedWindow("colorview",cv::WINDOW_NORMAL);
    cv::moveWindow("colorview",100,100);
    cv::namedWindow("grayview",cv::WINDOW_NORMAL);
    cv::moveWindow("grayview",600,100);
    image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw",1,showView);
    ros::spin();
    return 0;
}
