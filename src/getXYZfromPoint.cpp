#include "ros/ros.h"
#include "pcl_ros/point_cloud.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <boost/foreach.hpp>
class getXYZfromPoint
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    cv::Mat color = cv::Mat::zeros(cv::Size(640,480,3),CV_64FC1);
public:
    getXYZfromPoint(){
        sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB>>("/camera/depth_registered/points",1,&getXYZfromPoint::callback,this);
    };
    void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& point){
        ROS_INFO("Cloud:width = %d,height = %d",point->width,point->height);
        //row = width col = height index = x*width + y
        // BOOST_FOREACH(const pcl::PointXYZRGB& pt,point->points){
        //     color[:,:,0] =
        // }
        for(int i=0;i<480;i++){
            for(int j=0;j<640;j++){
                color[i][j][0] = point->points[i*480 + j].b;
                color[i][j][1] = point->points[i*480 + j].g;
                color[i][j][2] = point->points[i*480 + j].r;
            }
        }
        cv::imshow("BGR",color);
        cv::waitKey(5);
            // ROS_INFO("%f %f %f",point->points[240000].x,point->points[240000].y,point->points[240000].z);
    }
};
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"getXYZRGBfrompoints");
    getXYZfromPoint get;
    ros::spin();
    return 0;
}



