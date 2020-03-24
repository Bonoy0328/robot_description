#include "ros/ros.h"    
#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include <chrono>
class ORB_feature
{
private:
    cv::Mat cvColorImgMat;
    cv::Mat cvColorImgMat2;
    uint8_t flag=0;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it;
    image_transport::Subscriber sub;
public:
    ORB_feature()
        :it(nh_){
        sub = it.subscribe("/camera/rgb/image_raw",1,&ORB_feature::callback,this);
    }
    void callback(const sensor_msgs::ImageConstPtr& msgImg){
        cv_bridge::CvImagePtr cvImagePtr;
        try{
            cvImagePtr = cv_bridge::toCvCopy(msgImg,sensor_msgs::image_encodings::BGR8);
        }catch(cv_bridge::Exception e){
            ROS_ERROR_STREAM("Cv_bridge Exception:" << e.what());
            return;
        }
        cvColorImgMat = cvImagePtr->image;
        if(flag==0){
            cvColorImgMat2 = cvImagePtr->image;
            flag =2;
        }
        std::vector<cv::KeyPoint> keypoints,keypoints2;
        cv::Mat descriptors,descriptors2;
        cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
        cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        detector->detect(cvColorImgMat,keypoints);
        detector->detect(cvColorImgMat2,keypoints2);
        descriptor->compute(cvColorImgMat,keypoints,descriptors);
        descriptor->compute(cvColorImgMat2,keypoints2,descriptors2);
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        ROS_INFO("extract ORB cost = %f seconds",time_used.count());
        cvColorImgMat2 = cvColorImgMat;
        // cv::Mat outimg,outimg2;
        // cv::drawKeypoints(cvColorImgMat,keypoints,outimg,cv::Scalar::all(-1),cv::DrawMatchesFlags::DEFAULT);
        // cv::drawKeypoints(cvColorImgMat2,keypoints2,outimg2,cv::Scalar::all(-1),cv::DrawMatchesFlags::DEFAULT);
        // cv::imshow("colorview",outimg2);
        // cv::imshow("ORB feature",outimg);
        
        //match
        std::vector<cv::DMatch> matches;
        t1 = std::chrono::steady_clock::now();
        try
        {
            matcher->match(descriptors,descriptors2,matches);
        }
        catch(const cv::Exception& e)
        {
            ROS_INFO(e.what());
        }
        

        t2 = std::chrono::steady_clock::now();
        time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        ROS_INFO("match ORB cost %f seconds",time_used.count());

        //choise better match
        auto min_max = std::minmax_element(matches.begin(),matches.end(),
            [](const cv::DMatch &m1,const cv::DMatch &m2){return m1.distance < m2.distance;});
        double min_dist = min_max.first->distance;
        double max_dist = min_max.second->distance;
        std::vector<cv::DMatch> good_matches;
        for (int i = 0; i < descriptors.rows; i++){
            if(matches[i].distance <= std::max(2 * min_dist,30.0))
                good_matches.push_back(matches[i]);
        }

        //draw the answer
        cv::Mat img_match;
        cv::Mat img_goodmatch;
        cv::drawMatches(cvColorImgMat,keypoints,cvColorImgMat2,keypoints2,good_matches,img_goodmatch);
        cv::imshow("good matches",img_goodmatch);
        
        // cv::imshow("grayview",cvGrayImgMat);
        cv::waitKey(5);
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"grayView");
    // ros::NodeHandle nh;
    // image_transport::ImageTransport it(nh);
    // cv::namedWindow("colorview",cv::WINDOW_NORMAL);
    // cv::moveWindow("colorview",100,100);
    // cv::namedWindow("grayview",cv::WINDOW_NORMAL);
    // cv::moveWindow("grayview",600,100);
    ORB_feature orb;
    ros::spin();
    return 0;
}
