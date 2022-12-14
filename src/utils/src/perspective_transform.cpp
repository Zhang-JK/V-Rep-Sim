#include "perspective_transform.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;

Mat perspective_transform(Mat image, Point2f *points)
{
    // Define the destination image
    Mat dst = Mat::zeros(256, 256, CV_8UC3);

    // Define the 4 points of the destination image
    vector<Point2f> dstPoints;
    dstPoints.push_back(Point2f(0, 0));
    dstPoints.push_back(Point2f(256, 0));
    dstPoints.push_back(Point2f(256, 256));
    dstPoints.push_back(Point2f(0, 256));

    // Get the perspective transform matrix
    vector<Point2f> srcPoints = vector<Point2f>(points, points + 4);

    Mat m = getPerspectiveTransform(srcPoints, dstPoints);

    // Apply the perspective transform
    warpPerspective(image, dst, m, dst.size());

    return dst;
}

Mat extractHuman(Mat image, Point2f& centerP, bool show)
{
    Mat showImg = image.clone();
    Mat clone = image.clone();
    cvtColor(image, image, CV_BGR2GRAY);
    GaussianBlur(image, image, Size(3, 3), 0);
    Canny(image, image, 50, 250);
    Mat kernal = getStructuringElement(MORPH_RECT, Size(1, 1));
    dilate(image, image, Mat(), Point(-1, -1), 2);
    if (show)
        imshow("processed-debug", image);
    std::vector<std::vector<Point>> contours;
    findContours(image, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    RotatedRect rect;
    int maxArea = 0;
    Point2f poi[4];
    for (int i = 0; i < contours.size(); i++)
    {
        RotatedRect temp = minAreaRect(contours[i]);
        Point2f poit[4];
        temp.points(poit);
        if (show) 
            for (int j = 0; j < 4; j++)
                line(showImg, poit[j], poit[(j + 1) % 4], Scalar(255), 2);
        approxPolyDP(contours[i], contours[i], 10, true);
        if (contourArea(contours[i]) > 80 * 80)
        {
            if (show)
                ROS_INFO_STREAM("ratio: " << temp.size.width / temp.size.height);
            if (temp.size.width / temp.size.height > 2.3 || temp.size.width / temp.size.height < 0.8)
                continue;
            if (contourArea(contours[i]) > maxArea)
                maxArea = contourArea(contours[i]);
            else
                continue;
            rect = temp;
            rect.points(poi);
            break;
        }
    }
    if (show)
        imshow("selected-debug", showImg);
    if (contours.size() == 0)
        return Mat();

    if (rect.size.width < rect.size.height)
    {
        Point2f temp = poi[0];
        poi[0] = poi[1];
        poi[1] = poi[2];
        poi[2] = poi[3];
        poi[3] = temp;
    }
    if (rect.angle < -45)
    {
        swap(poi[0], poi[2]);
        swap(poi[1], poi[3]);
    }

    centerP = rect.center;
    return perspective_transform(clone, poi);
}
