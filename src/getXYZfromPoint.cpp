#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <boost/foreach.hpp>
#include <chrono>
#include <pcl/io/pcd_io.h>
// #include <pcl/common/impl/io.hpp>
#include "pcl_ros/point_cloud.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <sophus/se3.hpp>
#include <iostream>
#include <fstream>
#include <pangolin/pangolin.h>
#include "geometry_msgs/PoseStamped.h"

using namespace std;
using namespace Eigen;
using namespace cv;
class getXYZfromPoint
{

private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
    sensor_msgs::Image image_;
    geometry_msgs::PoseStamped robot_pose;
    cv::Mat cvColorImgMat;
    cv::Mat cvColorImgMat2;
    cv::Mat r, t;
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::PCLPointCloud2 pcl_pc2l;
    uint8_t flag=0;
    long int cnt = 0;
    
    typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;
    typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d;
// cv::Mat color = cv::Mat::zeros(cv::Size(640,480,3),CV_64FC1);
public:
    getXYZfromPoint(){
        // sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>("/camera/depth_registered/points",1,&getXYZfromPoint::callback,this);
        sub = nh.subscribe("/camera/depth_registered/points",5,&getXYZfromPoint::callback,this);
        pub = nh.advertise<geometry_msgs::PoseStamped>("robot_pose",1);
    }
    void callback(const sensor_msgs::PointCloud2ConstPtr& point){
        cv::Point3f d;
        std::vector<cv::Point3f> pts_3d,pts_3d_last;
        std::vector<cv::Point2f> pts_2d,pts_2d_last;
        vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses;
        cnt = 0;
        //当前帧数据
        cv_bridge::CvImagePtr cvImagePtr;
        pcl_conversions::toPCL(*point,pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_last(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
        //now we know image(x,y) → point2(y*640+x)
        // BOOST_FOREACH(const pcl::PointXYZ& pt,temp_cloud->points){
        //     cnt++;
        //     if(isnan(pt.x)||isnan(pt.y)||isnan(pt.z))
        //     continue;
        //     ROS_INFO("cnt %d",cnt);
        //     ROS_INFO("%f %f %f",pt.x,pt.y,pt.z);
        // }
        ROS_INFO("Cloud:width = %d,height = %d",temp_cloud->width,temp_cloud->height);
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

        //上一帧的数据
        if(flag==0){//如果是第一次进入，则和第一帧解算相同
            // pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_last(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud_last);
            cvColorImgMat2 = cvImagePtr->image;
            flag =2;
        }else{
            // pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_last(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromPCLPointCloud2(pcl_pc2l,*temp_cloud_last);      
            // cvColorImgMat2 = cvImagePtr->image;      
        }
        // cvColorImgMat2 = cvImagePtr->image;
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

        //match
        std::vector<cv::DMatch> matches;
        t1 = std::chrono::steady_clock::now();
        try
        {
            matcher->match(descriptors,descriptors2,matches);
        }
        catch(const cv::Exception& e)
        {
            cout<< e.what() <<endl;
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
        cout << "一共找到了 %d 组匹配点" << good_matches.size() <<endl;
        ofstream ofs;
        ofs.open("/home/bonoy/data/ORBfeature.txt",ios::app);
        ofs << "time " << time_used.count() << "num" << good_matches.size() <<endl;
        ofs.close();
        //获取3D点
        // BOOST_FOREACH(cv::DMatch m,good_matches){
            cv::DMatch m;
            for(int i=0;i<good_matches.size();i++){
            m = good_matches[i];
            d.x = temp_cloud_last->points[int(keypoints2[m.queryIdx].pt.y)*640 + int(keypoints2[m.queryIdx].pt.x)].x;
            d.y = temp_cloud_last->points[int(keypoints2[m.queryIdx].pt.y)*640 + int(keypoints2[m.queryIdx].pt.x)].y;
            d.z = temp_cloud_last->points[int(keypoints2[m.queryIdx].pt.y)*640 + int(keypoints2[m.queryIdx].pt.x)].z;      
            if(isnan(d.x)||isnan(d.y)||isnan(d.z))
                continue;
            pts_3d.push_back(d);
            pts_2d.push_back(keypoints[m.trainIdx].pt);
            // cout << "X: "<<d.x << " Y: " << d.y << " Z: "<< d.z << endl;
            // cout << "x:" << int(keypoints2[m.queryIdx].pt.x) << " y:" << int(keypoints2[m.queryIdx].pt.y) <<endl;
            // cout << "u:" << int(525.0*(d.x/d.z)+319.5) << " v:" << int(525*(d.y/d.z)+239.5) <<endl;
            // // ROS_INFO("u %d v %d",i,121);
        }
        std::cout << "3d-2s pairs: " << pts_3d.size() << std::endl;
        cv::Mat K = (cv::Mat_<double>(3, 3) << 525.0, 0, 319.5, 0, 525.0, 239.5, 0, 0, 1);
        //Opencv 解位姿
        t1 = std::chrono::steady_clock::now();
        try{
            cv::solvePnP(pts_3d, pts_2d, K, cv::Mat(), r, t, false,CV_ITERATIVE); 
        }catch(const cv::Exception& e)
        {
            cout<< e.what() <<endl;
        }
        // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
        cv::Mat R(3,3,CV_64FC1);
        Eigen::Matrix3d rotation_matrix;
        Eigen::Vector3d eigent;
        cv::Rodrigues(r, R); // r为旋转向量形式，用Rodrigues公式转换为矩阵
        t2 = std::chrono::steady_clock::now();
        time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        std::cout << "solve pnp in opencv cost time: " << time_used.count() << " seconds." << std::endl;

        std::cout << "R=" << std::endl << R << std::endl;
        std::cout << "t=" << std::endl << t << std::endl;
        cv::cv2eigen(R,rotation_matrix);
        cv::cv2eigen(t,eigent);
        Eigen::Quaterniond qua(rotation_matrix);
        robot_pose.header.stamp = point->header.stamp;
        robot_pose.header.frame_id = "robot_pose";
        robot_pose.pose.position.x = eigent(0);
        robot_pose.pose.position.y = eigent(1);
        robot_pose.pose.position.z = eigent(2);
        robot_pose.pose.orientation.x = qua.x();
        robot_pose.pose.orientation.y = qua.y();
        robot_pose.pose.orientation.z = qua.z();
        robot_pose.pose.orientation.w = qua.w();
        pub.publish(robot_pose);

        // Isometry3d Twr(qua);
        // Twr.pretranslate(eigent);
        // poses.push_back(Twr);
        // DrawTrajectory(poses);
        //gaussNewton 解位姿
        // getXYZfromPoint::VecVector3d pts_3d_eigen;
        // getXYZfromPoint::VecVector2d pts_2d_eigen;
        // for (size_t i = 0; i < pts_3d.size(); ++i) {
        //     pts_3d_eigen.push_back(Eigen::Vector3d(pts_3d[i].x, pts_3d[i].y, pts_3d[i].z));
        //     pts_2d_eigen.push_back(Eigen::Vector2d(pts_2d[i].x, pts_2d[i].y));
        // }
        // std::cout << "calling bundle adjustment by gauss newton " << std::endl;
        // Sophus::SE3d pose_gn;
        // t1 = std::chrono::steady_clock::now();
        // getXYZfromPoint::bundleAdjustmentGaussNewton(pts_3d_eigen,pts_2d_eigen,K,pose_gn);
        // t2 = std::chrono::steady_clock::now();
        // time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        // std::cout << "solve pnp by gauss newton cost time: " << time_used.count() << " seconds." << std::endl;
        //保存上一帧的数据
        // pcl_pc2l = pcl_pc2;
        pcl::copyPointCloud(*pcl_pc2,*pcl_pc2l);
        cvColorImgMat.copyTo(cvColorImgMat2);
        //draw answer
        cv::Mat img_match;
        cv::Mat img_goodmatch;
        cv::drawMatches(cvColorImgMat,keypoints,cvColorImgMat2,keypoints2,good_matches,img_goodmatch);
        cv::imshow("good matches",img_goodmatch);
        // cv::imshow("grayview",cvGrayImgMat);
        cv::waitKey(5);
        // ROS_INFO("%f %f %f",point->points[240000].b,point->points[240000].g,point->points[240000].r);
    }
void bundleAdjustmentGaussNewton(const getXYZfromPoint::VecVector3d &points_3d,const getXYZfromPoint::VecVector2d &points_2d,const cv::Mat &K,Sophus::SE3d &pose) {
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
        // return pose;
    }
void DrawTrajectory(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses){
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
    pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
  );

  pangolin::View &d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
    .SetHandler(new pangolin::Handler3D(s_cam));
  while (pangolin::ShouldQuit() == false) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glLineWidth(2);
    for (size_t i = 0; i < poses.size(); i++) {
      // 画每个位姿的三个坐标轴
      Vector3d Ow = poses[i].translation();
      Vector3d Xw = poses[i] * (0.1 * Vector3d(1, 0, 0));
      Vector3d Yw = poses[i] * (0.1 * Vector3d(0, 1, 0));
      Vector3d Zw = poses[i] * (0.1 * Vector3d(0, 0, 1));
      glBegin(GL_LINES);
      glColor3f(1.0, 0.0, 0.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(Xw[0], Xw[1], Xw[2]);
      glColor3f(0.0, 1.0, 0.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(Yw[0], Yw[1], Yw[2]);
      glColor3f(0.0, 0.0, 1.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(Zw[0], Zw[1], Zw[2]);
      glEnd();
    }
    // 画出连线
    for (size_t i = 0; i < poses.size(); i++) {
      glColor3f(0.0, 0.0, 0.0);
      glBegin(GL_LINES);
      auto p1 = poses[i], p2 = poses[i + 1];
      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      glEnd();
    }
    pangolin::FinishFrame();
    pangolin::Quit();
    //usleep(5000);   // sleep 5 ms
  }
}
};
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"getXYZRGBfrompoints");
    getXYZfromPoint get;
    ros::spin();
    return 0;
}



