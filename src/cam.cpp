#include <ros/ros.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>





using namespace cv;
using namespace std;
image_transport::Publisher pub ;
float x_bot, y_bot, yaw;
Mat img, imgf;
int isvalid(Mat A, int i, int j)
{
    if(i<A.rows && j< A.cols &&i>=0&&j>=0) return 1;
    return 0;
}
void DrawRotatedRectangle(Mat image, Point centerPoint,Size rectangleSize, double rotationDegrees, Scalar color)
{
    RotatedRect rotatedRectangle(centerPoint, rectangleSize, rotationDegrees);
    Point2f vertices2f[4];
    rotatedRectangle.points(vertices2f);
    Point vertices1[4],vertices2[4];    
    vertices1[0]=vertices2f[0];
    vertices1[1]=vertices2f[1];
    vertices1[3]=(vertices2f[0]+vertices2f[3])/2;
    vertices1[2]=(vertices2f[1]+vertices2f[2])/2;
    // cout << vertices1[0]<< " " << vertices1[1]<<" "<< vertices1[2]<< " " << vertices1[3]<<endl;

    vertices2[0]=(vertices2f[0]+vertices2f[3])/2;
    vertices2[1]=(vertices2f[1]+vertices2f[2])/2;
    vertices2[2]=vertices2f[2];
    vertices2[3]=vertices2f[3];
    fillConvexPoly(image,vertices1,4,Scalar(255,0,0));
    fillConvexPoly(image,vertices2,4,Scalar(0,255,255));
}

void odoms(const nav_msgs::Odometry::ConstPtr& data)
{
     x_bot = (data->pose.pose.position.x)*10;
     y_bot = (data->pose.pose.position.y)*10;
    
    // quarternion to euler conversion
    float siny = 2.0 * (data->pose.pose.orientation.w *
                   data->pose.pose.orientation.z +
                   data->pose.pose.orientation.x *
                   data->pose.pose.orientation.y);
    float cosy = 1.0 - 2.0 * (data->pose.pose.orientation.y *
                         data->pose.pose.orientation.y +
                         data->pose.pose.orientation.z *
                         data->pose.pose.orientation.z);

    yaw = atan2(siny, cosy) ;//yaw in radians
    imgf=img.clone();
    if(!isvalid(imgf,y_bot+96,imgf.cols-x_bot-46)) cout<<"Out of Bounds"<<endl;
    else DrawRotatedRectangle(imgf, Point(y_bot+96,imgf.cols-x_bot-46),Size(10,10),90+yaw*(180/CV_PI),Scalar(255,255,0));
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "planner_husky");

    ros::NodeHandle nh;
    img = imread("/home/vib2810/catkin_ws/src/pixel/src/maze.png");
    if( !img.data ) { printf("Error loading A \n"); return -1; }
    
    // waitKey(0);
    ros::Subscriber odom = nh.subscribe("/odometry/filtered",1, odoms);
    image_transport::ImageTransport it(nh);   
    pub = it.advertise("cam", 1);
    imgf=imread("/home/vib2810/catkin_ws/src/pixel/src/first.png");
    ros::Rate r(300);
    namedWindow("maze",WINDOW_NORMAL);
    while(ros::ok())
    {
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgf).toImageMsg();
        imshow("maze",imgf);
        waitKey(1);
        pub.publish(msg);  
        cout<< "Publishing"<< endl;
        cout<< "."<< endl;
        ros::spinOnce();
        r.sleep();
        // imwrite("/home/vib2810/catkin_ws/src/pixel/src/first.png",imgf);
    }


}