#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Joy.h"

enum {KEYBOARD, JOYCON} mode;

void keyboardCB(const geometry_msgs::Twist::ConstPtr &msg);
void joyConCB(const sensor_msgs::Joy::ConstPtr &msg);

ros::Publisher speedPub;
ros::Publisher laserPub;
ros::Publisher cameraPub;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_controller_node");
    ros::NodeHandle n;

     speedPub = n.advertise<geometry_msgs::Twist>("/vrep/cmd_vel", 20);
     laserPub = n.advertise<std_msgs::Bool>("/vrep/laser_switch", 20);
     cameraPub = n.advertise<std_msgs::Bool>("/vrep/camera_switch", 20);

    ros::Subscriber keySub = n.subscribe("/cmd_vel", 20, keyboardCB);
    ros::Subscriber joySub = n.subscribe("/joy", 20, joyConCB);

    ros::Rate loop_rate(1000);
    int count = 0;

    ROS_INFO_STREAM("Robot Controller Node STARTED.");
    ros::spin();
    return 0;
}

void keyboardCB(const geometry_msgs::Twist::ConstPtr &msg)
{
    
}

void joyConCB(const sensor_msgs::Joy::ConstPtr &msg)
{
    ROS_INFO_STREAM("JOY REC");
}