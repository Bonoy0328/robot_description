#include "ros/ros.h"
#include "pcl_ros/point_cloud.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <pcl/io/pcd_io.h>
#include <boost/foreach.hpp>
class getXYZfromPoint
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    sensor_msgs::Image image_;
    cv::Mat cvColorImgMat_;
    // cv::Mat color = cv::Mat::zeros(cv::Size(640,480,3),CV_64FC1);
public:
    getXYZfromPoint(){
        // sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>("/camera/depth_registered/points",1,&getXYZfromPoint::callback,this);
        sub = nh.subscribe("/camera/depth_registered/points",5,&getXYZfromPoint::callback,this);
    };
    void callback(const sensor_msgs::PointCloud2ConstPtr& point){
        cv_bridge::CvImagePtr cvImagePtr;
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
        cvColorImgMat_ = cvImagePtr->image;
        cv::imshow("view",cvColorImgMat_);
        cv::waitKey(5);
        //row = width col = height index = x*width + y
        // BOOST_FOREACH(const pcl::PointXYZ& pt,point->points){
            // ROS_INFO("%f %f %f",pt.x,pt.y,pt.z);
            // uint32_t rgb = (uint32_t)(pt.rgb);
            // uint8_t r = (rgb >> 16)& 0x0000ff;
            // uint8_t g = (rgb >> 8)& 0x0000ff;
            // uint8_t b = (rgb) & 0x0000ff;
            // if(r!=0||g!=0||b!=0)
            // ROS_INFO("%d %d %d",r,g,b);
        // }
        // for(int i=0;i<480;i++){
        //     for(int j=0;j<640;j++){
        //         color[i][j][0] = point->points[i*480 + j].b;
        //         color[i][j][1] = point->points[i*480 + j].g;
        //         color[i][j][2] = point->points[i*480 + j].r;
        //     }
        // }
        // cv::imshow("BGR",color);
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



