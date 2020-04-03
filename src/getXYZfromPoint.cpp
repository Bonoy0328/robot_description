#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <boost/foreach.hpp>
#include <chrono>
#include <pcl/io/pcd_io.h>
#include "pcl_ros/point_cloud.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <sophus/se3.hpp>
#include <Eigen/Core>
class getXYZfromPoint
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    sensor_msgs::Image image_;
    cv::Mat cvColorImgMat;
    cv::Mat cvColorImgMat2;
    pcl::PCLPointCloud2 pcl_pc2;
    uint8_t flag=0;
    long int cnt = 0;
    // cv::Mat color = cv::Mat::zeros(cv::Size(640,480,3),CV_64FC1);
public:
    getXYZfromPoint(){
        // sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>("/camera/depth_registered/points",1,&getXYZfromPoint::callback,this);
        sub = nh.subscribe("/camera/depth_registered/points",5,&getXYZfromPoint::callback,this);
    };
    
    void callback(const sensor_msgs::PointCloud2ConstPtr& point){
        cnt = 0;
        cv_bridge::CvImagePtr cvImagePtr;
        pcl_conversions::toPCL(*point,pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
        //now we know image(x,y) → point2(y*640+x)
        // BOOST_FOREACH(const pcl::PointXYZ& pt,temp_cloud->points){
        //     cnt++;
        //     if(isnan(pt.x)||isnan(pt.y)||isnan(pt.z))
        //     continue;
        //     ROS_INFO("cnt %d",cnt);
        //     ROS_INFO("%f %f %f",pt.x,pt.y,pt.z);
        // }
        ROS_INFO("Cloud:width = %d,height = %d",point->width,point->height);
        try
        {
            pcl::toROSMsg(*point,image_);
        }
        catch(std::runtime_error e)
        {
            ROS_ERROR_STREAM("Error in converting cloud to image message: " << e.what());
        }
        try{
            cvImagePtr = cv_bridge::toCvCopy(image_,sensor_msgs::image_encodings::BGR8);
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
        
        //chose better
        auto min_max = std::minmax_element(matches.begin(),matches.end(),
            [](const cv::DMatch &m1,const cv::DMatch &m2){return m1.distance < m2.distance;});
        double min_dist = min_max.first->distance;
        double max_dist = min_max.second->distance;
        std::vector<cv::DMatch> good_matches;
        for (int i = 0; i < descriptors.rows; i++){
            if(matches[i].distance <= std::max(2 * min_dist,30.0))
                good_matches.push_back(matches[i]);
        }
        ROS_INFO("一共找到了 %d 组匹配点",good_matches.size());

        //获取3D点
        std::vector<Point3f> pts_3d;
        std::vector<Point2f> pts_2d;
        BOOST_FOREACH(cv::DMatch m,good_matches){
            cv::Point3f d = temp_cloud->points[int(keypoints_1[m.queryIdx].pt.y)*640 + int(keypoints_1[m.queryIdx].pt.x)]
            if(isnan(d.x)||isnan(d.y)||isnan(d.z))
                continue;
            // pts_3d.push_back(d)
            ROS_INFO("%f %f %f",d.x,d.y,d.z);

        }
        // for (cv::DMatch m:good_matches){
        //     double d = temp_cloud->points[int(keypoints_1[m.queryIdx].pt.y)*640 + int(keypoints_1[m.queryIdx].pt.x)]
        //     ROS_INFO("%f %f %f",pt.x,pt.y,pt.z);
        //     ROS_INFO()
        // }
        

        // //draw answer
        // cv::Mat img_match;
        // cv::Mat img_goodmatch;
        // cv::drawMatches(cvColorImgMat,keypoints,cvColorImgMat2,keypoints2,good_matches,img_goodmatch);
        // cv::imshow("good matches",img_goodmatch);
        // // cv::imshow("grayview",cvGrayImgMat);
        // cv::waitKey(5);
        // ROS_INFO("%f %f %f",point->points[240000].b,point->points[240000].g,point->points[240000].r);
    }
};
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"getXYZRGBfrompoints");
    getXYZfromPoint get;
    ros::spin();
    return 0;
}



