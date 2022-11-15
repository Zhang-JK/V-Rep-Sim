// Include the ROS library
#include <ros/ros.h>

// Include opencv2
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/face.hpp>

// Include CvBridge, Image Transport, Image msg
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// OpenCV Window Name
static const std::string OPENCV_WINDOW = "Image window";

// Topics
static const std::string IMAGE_TOPIC = "/vrep/image";

// Publisher
ros::Publisher pub;

cv::Ptr<cv::face::FaceRecognizer> model;
std::vector<std::string> nameList({"Person 1", "Person 2", "Person 3", "Person 4", "Person 5"});

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat imgR = cv::Mat::zeros(256, 256, CV_8UC3);
    cv::resize(img, imgR, imgR.size());
    cv::Mat imgRG = cv::Mat::zeros(256, 256, CV_8U);
    cv::cvtColor(imgR, imgRG, CV_BGR2GRAY);
    int label = -1;
    double confidence = 0;
    model->predict(imgRG, label, confidence);
    if (confidence > 20) label = -1;
    if (label != -1) ROS_INFO_STREAM("Predicted label: " << label << " with confidence: " << confidence << " -- " << model->getThreshold());
    cv::putText(img, label == -1 ? "Nothing" : nameList[label], cv::Point(10, 35), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(118, 185, 0), 2);
    cv::imshow(OPENCV_WINDOW, img);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "face_detection_node");
  ros::NodeHandle privateNh("~");
  ros::NodeHandle nh;
  int num_components = 5;
  double threshold = 10.0;
  std::string path = "";
  privateNh.param<int>("eigen_components", num_components, 200);
  privateNh.param<double>("face_threshold", threshold, 10.0);
  privateNh.param<std::string>("project_dir", path, "/home/laojk/Code/ELEC3210-Project/src/");
  privateNh.getParam("nameList", nameList);
  cv::namedWindow(OPENCV_WINDOW);
  model = cv::face::LBPHFaceRecognizer::create();
  model->read(path + "face_detection/model/face_model.xml");

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe(IMAGE_TOPIC, 1, imageCallback);
  ros::spin();
  cv::destroyWindow(OPENCV_WINDOW);
}
