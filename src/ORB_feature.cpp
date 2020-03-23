#include "ros/ros.h"    
#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include <chrono>
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
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    detector->detect(cvColorImgMat,keypoints);
    // descriptor->compute(cvColorImgMat,keypoints,descriptors);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    ROS_INFO("extract ORB cost = %f seconds",time_used.count());
    cv::Mat outimg;
    cv::drawKeypoints(cvColorImgMat,keypoints,outimg,cv::Scalar::all(-1),cv::DrawMatchesFlags::DEFAULT);
    cv::cvtColor(cvColorImgMat,cvGrayImgMat,CV_BGR2GRAY);
    // cv::imshow("colorview",cvColorImgMat);
    cv::imshow("ORB feature",outimg);
    // cv::imshow("grayview",cvGrayImgMat);
    cv::waitKey(5);
}
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"grayView");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    cv::namedWindow("colorview",cv::WINDOW_NORMAL);
    cv::moveWindow("colorview",100,100);
    cv::namedWindow("grayview",cv::WINDOW_NORMAL);
    cv::moveWindow("grayview",600,100);
    image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw",2,showView);
    ros::spin();
    return 0;
}
