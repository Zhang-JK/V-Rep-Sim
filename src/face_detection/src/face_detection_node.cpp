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

#include <perspective_transform.hpp>
#include <sensor_msgs/JoyFeedbackArray.h>

#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>

using namespace cv;

// OpenCV Window Name
static const std::string OPENCV_WINDOW = "Face Detection";

// Topics
static const std::string IMAGE_TOPIC = "/vrep/image";

// Publisher
ros::Publisher pub;
ros::Publisher markerPub;
std::vector<float> laserRes;

visualization_msgs::Marker marker;
sensor_msgs::JoyFeedbackArray feedback;

Ptr<face::FaceRecognizer> model;
std::vector<std::string> nameList({"Person 1", "Person 2", "Person 3", "Person 4", "Person 5"});

double findAngle(int pos, double width = 256)
{
	int relPos = pos - width / 2;
	static double h = width / 2 / tan(22.5 / 180 * M_PI);
	return atan(relPos / h);
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
	try
	{
		Mat raw = cv_bridge::toCvShare(msg, "bgr8")->image;
		Point2f centerP;
		Mat human = extractHuman(raw, centerP, true);
		int label = -1;
		double confidence = 0;
		if (human.size().area() > 80 * 80)
		{
			cvtColor(human, human, CV_BGR2GRAY);
			model->predict(human, label, confidence);
			if (confidence > 45)
				label = -1;
		}
		if (label != -1) 
		{
			putText(human, nameList[label], Point(10, 35), FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255, 255, 255), 2);
			imshow(OPENCV_WINDOW, human);

			double angle = findAngle(centerP.x, 512);
			int tranAng = (M_PI / 2 - angle) / (M_PI) * 901;
			if (tranAng < 0) tranAng = 0;
			if (tranAng > 901) tranAng = 901;
			if (laserRes.size() > 1) {
				marker.pose.position.y = - laserRes[tranAng] * cos(angle);
				marker.pose.position.x = laserRes[tranAng] * sin(angle);
				ROS_INFO_STREAM("Angle: " << angle << ", DisX: " << laserRes[tranAng] * sin(angle) << ", DisY: " << laserRes[tranAng] * cos(angle));
				marker.header.stamp = ros::Time();
				marker.id = label;
				marker.text = nameList[label];
				markerPub.publish(marker);
				ROS_INFO_STREAM("Find " << nameList[label] << ", at " << laserRes[tranAng] << " in laser_link with confidence: " << confidence);
			}
			
			feedback.array[0].intensity = (45 - confidence) / 25.0f;
			if (feedback.array[0].intensity > 1) feedback.array[0].intensity = 1;
		}
		else {
			imshow(OPENCV_WINDOW, Mat(Size(256, 256), CV_8UC3, Scalar(256, 256, 256)));
			feedback.array[0].intensity = 0;
		}
		pub.publish(feedback);
		waitKey(5);
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	laserRes = msg.get()->ranges;
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
	namedWindow(OPENCV_WINDOW);
	model = face::LBPHFaceRecognizer::create();
	model->read(path + "face_detection/model/face_model.xml");
	pub = nh.advertise<sensor_msgs::JoyFeedbackArray>("/joy/set_feedback", 1);
	markerPub = nh.advertise<visualization_msgs::Marker>( "/marker", 0 );
	marker.header.frame_id = "laser_link";
	marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker.action = visualization_msgs::Marker::MODIFY;
	marker.scale.z = 0.3;			
	marker.color.a = 1.0;
	marker.color.r = 0xD0 / 255.0;
	marker.color.g = 0x10 / 255.0;
	marker.color.b = 0x4C / 255.0;
	feedback.array = std::vector<sensor_msgs::JoyFeedback>(1);
	feedback.array[0].id = 0;
	feedback.array[0].type = 1;

	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe(IMAGE_TOPIC, 1, imageCallback);
	ros::Subscriber laserSub = nh.subscribe("/vrep/scan", 1, laserCallback);
	imshow(OPENCV_WINDOW, Mat(Size(256, 256), CV_8UC3, Scalar(256, 256, 256)));
	waitKey(30);
	ros::spin();
	destroyWindow(OPENCV_WINDOW);
}
