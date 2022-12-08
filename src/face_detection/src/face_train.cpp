// Include the ROS library
#include <ros/ros.h>

// Include opencv2
#include <opencv2/face.hpp>
#include <opencv2/imgcodecs.hpp>
#include <vector>
using namespace cv::face;
using namespace cv;

// Include CvBridge, Image Transport, Image msg
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "face_train");
    int num_components = 5;
    double threshold = 10.0;
    std::string path = "";
    ros::NodeHandle nh;
    nh.param<int>("eigen_components", num_components, 200);
    nh.param<double>("face_threshold", threshold, 10.0);
    nh.param<std::string>("project_dir", path, "/home/laojk/Code/ELEC3210-Project/src/");
    Ptr<FaceRecognizer> model = LBPHFaceRecognizer::create();
    std::vector<Mat> images;
    std::vector<int> labels;
    Mat trainData[31];
    for (int i = 0; i < 5; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            trainData[i*6+j] = Mat::zeros(256, 256, CV_8UC3);
            resize(imread(path + "face_detection/images/p" + std::to_string(i + 1) + "_" + std::to_string(j + 1) + ".png", IMREAD_GRAYSCALE), trainData[i*6+j], trainData[i*6+j].size());
            images.push_back(trainData[i*6+j]);
            labels.push_back(i);
        }
    }
    model->train(images, labels);
    model->save(path + "face_detection/model/face_model.xml");
}