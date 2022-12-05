#include <ros/ros.h>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include "std_msgs/String.h"
#include <std_msgs/Bool.h>
#include "geometry_msgs/Twist.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
using namespace std;

ros::Publisher track_pub;
ros::Subscriber track_sub;
ros::Subscriber sub_start;
bool start;
double v_1 = 2;
// This is the callback function that will get called when a new image has arrived on the "camera/image" topic. 
//Although the image may have been sent in some arbitrary transport-specific message type, 
//notice that the callback need only handle the normal sensor_msgs/Image type.
void imageCb(const sensor_msgs::ImageConstPtr &msg)
{

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        //converting a ROS sensor_msgs/Image message into a CvImage
        //We want to modify the data in-place. We have to make a copy of the ROS message data.

        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Mat image_src, image_hsv, image_binary;
    image_src = cv_ptr->image;
    //convert from bgr to hsv to 
    cvtColor(image_src, image_hsv, COLOR_BGR2HSV);
    inRange(image_hsv, Scalar(20, 95, 95), Scalar(30, 255, 255), image_binary);
    vector<Vec3f> circles;
    HoughCircles(image_binary, circles, HOUGH_GRADIENT, 1,
                 image_binary.rows / 16, // change this value to detect circles with different distances to each other
                 100, 15, 1, 0            // change the last two parameters                                     // (min_radius & max_radius) to detect larger circles
    );
    Point image_center = Point(image_binary.rows / 2, image_binary.cols / 2);
    ////Loads an image and blur it to reduce the noise
    ////Applies the Hough Circle Transform to the blurred image .
    ////Display the detected circle in a window.
    circle(image_binary, image_center, 1, Scalar(0, 50, 100), 3, LINE_AA);
    for (size_t i = 0; i < circles.size(); i++)
    {
        Vec3i c = circles[i];
        Point c_center = Point(c[0], c[1]);
        // circle center
        circle(image_binary, c_center, 1, Scalar(0, 100, 100), 3, LINE_AA);
        // circle outline
        int radius = c[2];
        circle(image_binary, c_center, radius, Scalar(255, 0, 255), 3, LINE_AA);
    }

    waitKey(3);
    double lv = 0;
    double av = 0;
    if (circles.size())
    {

        Vec3i yellow_ball = circles[0];
        // cout << yellow_ball[2] << endl;
        if (yellow_ball[2] < 100)
        {
            lv = 1.8;
        }
        else
        {
            lv = 0;
        }

        if (yellow_ball[0] > image_binary.rows / 2)
        {
            av = 0.4;
        }

        else if (yellow_ball[0] < image_binary.rows / 2)
        {
            av = -0.4;
        }
        
        else
        {
            av = 0;
        }
        v_1 = av*2;
    }
    else
    {
        av = v_1;
    }
    geometry_msgs::Twist tw;
    if (start)
    {
        tw.linear.x = lv;
        tw.angular.z = av;
        track_pub.publish(tw);
    }
}

void startCb(const std_msgs::Bool &msg)
{
    start = msg.data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "track");
    ros::NodeHandle n;
    sub_start = n.subscribe("run_tracking", 1000, startCb);
    track_sub = n.subscribe("/vrep/image", 1000, imageCb);
    track_pub = n.advertise<geometry_msgs::Twist>("/vrep/cmd_vel", 1000);
    ros::spin();
    return 0;
}