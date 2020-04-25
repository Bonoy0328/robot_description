#include "ros/ros.h"  
// 话题同步处理
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
// opencv接口
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/calib3d/calib3d.hpp>
//ros消息类型头文件
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
//系统指令头文件
#include <boost/thread/thread.hpp>
#include <boost/foreach.hpp>
// pcl点云库
#include <pcl/io/pcd_io.h>
#include "pcl_ros/point_cloud.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
//内部时钟
#include <chrono>
#include "sensor_msgs/Imu.h"
#include "eigen3/Eigen/Core"
#include <sophus/se3.hpp>
#include <iostream>
sensor_msgs::Image image_;
cv::Mat cvColorImgMat;
cv::Mat cvColorImgMat2;
pcl::PCLPointCloud2 pcl_pc2;
pcl::PCLPointCloud2 pcl_pc2l;
uint8_t flag=0;
long int cnt = 0;
typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;
typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d;


void bundleAdjustmentGaussNewton(const VecVector3d &points_3d,const VecVector2d &points_2d,const cv::Mat &K,Sophus::SE3d &pose) {
    typedef Eigen::Matrix<double,6,1> Vector6d;
    const int iterations = 10;
    double cost = 0,lastCost = 0;
    double fx = K.at<double>(0,0);
    double fy = K.at<double>(1,1);
    double cx = K.at<double>(0,2);
    double cy = K.at<double>(1,2);
    for(int iter = 0;iter < iterations; iter++){
        Eigen::Matrix<double,6,6> H = Eigen::Matrix<double,6,6>::Zero();
        Vector6d b = Vector6d::Zero();
        cost = 0;
        //compute cost
        for(int i = 0;i < points_3d.size(); i++){
            Eigen::Vector3d pc = pose * points_3d[i];
            double inv_z = 1.0 / pc[2];
            double inv_z2 = inv_z * inv_z;
            Eigen::Vector2d proj(fx * pc[0] / pc[2] + cx,fy * pc[1] / pc[2] + cy);
            Eigen::Vector2d e = points_2d[i] - proj;
            cost+=e.squaredNorm();

            Eigen::Matrix<double,2,6> J;
            J << -fx * inv_z,
            0,
            fx * pc[0] * inv_z2,
            fx * pc[0] * pc[1] * inv_z2,
            -fx - fx * pc[0] * pc[0] * inv_z2,
            fx * pc[1] * inv_z,
            0,
            -fy * inv_z,
            fy * pc[1] * inv_z2,
            fy + fy * pc[1] * pc[1] * inv_z2,
            -fy * pc[0] * pc[1] * inv_z2,
            -fy * pc[0] * inv_z;

            H += J.transpose() * J;
            b += -J.transpose() * e;
        }
        Vector6d dx;
        dx = H.ldlt().solve(b);
        if(isnan(dx[0])){
            std::cout << "result is nan!" << std::endl;
            break;
        }
        
        if(iter > 0 && cost >= lastCost){
            // cost increase, update is not good
            std::cout << "cost: " << cost << ", last cost: " << lastCost << std::endl;
            break;
        }

        //update estimation
        pose = Sophus::SE3d::exp(dx) * pose;
        lastCost = cost;
        std::cout << "iteration " << iter << " cost=" << std::setprecision(12) << cost << std::endl;
        if(dx.norm() < 1e-6){
            break;
        }
    }
    std::cout << "pose by g-n \n" << pose.matrix() <<std::endl;
}



void callback(const sensor_msgs::ImuConstPtr& imu,const sensor_msgs::PointCloud2ConstPtr& point){
    std::cout <<"this is quternion"   << imu-> orientation.x << std::endl;
    cv_bridge::CvImagePtr cvImagePtr;
    pcl_conversions::toPCL(*point,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_last(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    ROS_INFO("Cloud:width = %d,height = %d",point->width,point->height);
    try{
        pcl::toROSMsg(*point,image_);
    }catch(std::runtime_error e){
        ROS_ERROR_STREAM("Error in converting cloud to image message: " << e.what());
    }
    try{
        cvImagePtr = cv_bridge::toCvCopy(image_,sensor_msgs::image_encodings::BGR8);
    }catch(cv_bridge::Exception e){
        ROS_ERROR_STREAM("Cv_bridge Exception:" << e.what());
        return;
    }    
    cvColorImgMat = cvImagePtr->image;

    if(flag==0){//如果是第一次进入，则和第一帧解算相同
        // pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_last(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud_last);
        cvColorImgMat2 = cvImagePtr->image;
        flag =2;
    }else{
        // pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_last(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2l,*temp_cloud_last);            
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
    
    // cvColorImgMat2 = cvColorImgMat;
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

    //获取3D点
    cv::Point3f d;
    std::vector<cv::Point3f> pts_3d,pts_3d_last;
    std::vector<cv::Point2f> pts_2d,pts_2d_last;
    BOOST_FOREACH(cv::DMatch m,good_matches){
        d.x = temp_cloud_last->points[int(keypoints2[m.queryIdx].pt.y)*640 + int(keypoints2[m.queryIdx].pt.x)].x;
        d.y = temp_cloud_last->points[int(keypoints2[m.queryIdx].pt.y)*640 + int(keypoints2[m.queryIdx].pt.x)].y;
        d.z = temp_cloud_last->points[int(keypoints2[m.queryIdx].pt.y)*640 + int(keypoints2[m.queryIdx].pt.x)].z;
        if(isnan(d.x)||isnan(d.y)||isnan(d.z))
            continue;
        pts_3d.push_back(d);
        pts_2d.push_back(keypoints[m.queryIdx].pt);
        // ROS_INFO("%f %f %f",d.x,d.y,d.z);
        // ROS_INFO("%d %d",int(keypoints[m.queryIdx].pt.x),int(keypoints[m.queryIdx].pt.y));
    }
    std::cout << "3d-2s pairs: " << pts_3d.size() << std::endl;
    cv::Mat K = (cv::Mat_<double>(3, 3) << 525.0, 0, 319.5, 0, 525.0, 239.5, 0, 0, 1);
    
    //Opencv 解位姿
    // t1 = std::chrono::steady_clock::now();
    // cv::Mat r, t;
    // cv::solvePnP(pts_3d, pts_2d, K, cv::Mat(), r, t,false,CV_ITERATIVE); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
    // cv::Mat R;
    // cv::Rodrigues(r, R); // r为旋转向量形式，用Rodrigues公式转换为矩阵
    // t2 = std::chrono::steady_clock::now();
    // time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    // std::cout << "solve pnp in opencv cost time: " << time_used.count() << " seconds." << std::endl;

    // std::cout << "R=" << std::endl << R << std::endl;
    // std::cout << "t=" << std::endl << t << std::endl;

    //gaussNewton 解位姿
    VecVector3d pts_3d_eigen;
    VecVector2d pts_2d_eigen;
    for (size_t i = 0; i < pts_3d.size(); ++i) {
        pts_3d_eigen.push_back(Eigen::Vector3d(pts_3d[i].x, pts_3d[i].y, pts_3d[i].z));
        pts_2d_eigen.push_back(Eigen::Vector2d(pts_2d[i].x, pts_2d[i].y));
    }
    std::cout << "calling bundle adjustment by gauss newton " << std::endl;
    Sophus::SE3d pose_gn;
    t1 = std::chrono::steady_clock::now();
    bundleAdjustmentGaussNewton(pts_3d_eigen,pts_2d_eigen,K,pose_gn);
    t2 = std::chrono::steady_clock::now();
    time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "solve pnp by gauss newton cost time: " << time_used.count() << " seconds." << std::endl;
        
    //保存上一帧的数据
    pcl_pc2l = pcl_pc2;
    cvColorImgMat2 = cvColorImgMat;

    //draw the answer
    cv::Mat img_match;
    cv::Mat img_goodmatch;
    cv::drawMatches(cvColorImgMat,keypoints,cvColorImgMat2,keypoints2,good_matches,img_goodmatch);
    cv::imshow("good matches",img_goodmatch);
    // cv::imshow("grayview",cvGrayImgMat);
    cv::waitKey(5);
}
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"grayView");
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::Imu> sub(nh_,"/imu_data",1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub2(nh_,"/camera/depth_registered/points",1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu,sensor_msgs::PointCloud2> MySync;
    message_filters::Synchronizer<MySync> sync(MySync(10),sub,sub2);
    sync.registerCallback(boost::bind(&callback,_1,_2));
    // cv::namedWindow("colorview",cv::WINDOW_NORMAL);
    // cv::moveWindow("colorview",100,100);
    // cv::namedWindow("grayview",cv::WINDOW_NORMAL);
    // cv::moveWindow("grayview",600,100);
    ros::spin();
    return 0;
}
