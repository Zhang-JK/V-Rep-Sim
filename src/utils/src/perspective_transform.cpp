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
    Canny(image, image, 100, 200);
    Mat kernal = getStructuringElement(MORPH_RECT, Size(1, 1));
    dilate(image, image, Mat(), Point(-1, -1), 2);
    if (show)
        imshow("processed-debug", image);
    std::vector<std::vector<Point>> contours;
    findContours(image, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    RotatedRect rect;
    Point2f poi[4];
    for (int i = 0; i < contours.size(); i++)
    {
        approxPolyDP(contours[i], contours[i], 10, true);
        if (contourArea(contours[i]) > 120 * 120)
        {
            rect = minAreaRect(contours[i]);
            rect.points(poi);
            for (int j = 0; j < 4; j++)
                line(showImg, poi[j], poi[(j + 1) % 4], Scalar(255), 2);
            if (show)
                imshow("selected-debug", showImg);
            break;
        }
    }
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
