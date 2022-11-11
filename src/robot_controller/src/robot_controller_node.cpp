#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Joy.h"

enum
{
    KEYBOARD,
    JOYCON
} mode;
bool useLaser = true;
bool useCamera = true;
bool useTri = false;

void keyboardCB(const geometry_msgs::Twist::ConstPtr &msg);
void joyConCB(const sensor_msgs::Joy::ConstPtr &msg);

ros::Publisher speedPub;
ros::Publisher laserPub;
ros::Publisher cameraPub;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_controller_node");
    ros::NodeHandle n;
    mode = KEYBOARD;

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
    if (mode == KEYBOARD)
    {
        static geometry_msgs::Twist speed;
        speed.angular = msg.get()->angular;
        speed.linear = msg.get()->linear;
        speedPub.publish(speed);
    }
    else
    {
        ROS_WARN_STREAM("YOU ARE IN JOY CON MODE");
    }
}

void joyConCB(const sensor_msgs::Joy::ConstPtr &msg)
{
    if (msg.get()->buttons[0] == 1)
    {
        if (mode == JOYCON)
        {
            mode = KEYBOARD;
            ROS_INFO_STREAM("Switch to KEYBOARD Mode");
        }
        else
        {
            mode = JOYCON;
            ROS_INFO_STREAM("Switch to JON CON Mode");
        }
    }

    if (mode != JOYCON)
    {
        ROS_WARN_STREAM("YOU ARE IN KEYBOARD MODE");
        return ;
    }

    if (msg.get()->buttons[1] == 1)
    {
        useCamera = !useCamera;
        static std_msgs::Bool cam;
        cam.data = useCamera;
        cameraPub.publish(cam);
        if (useCamera) ROS_INFO_STREAM("Switch on the Camera");
        else ROS_INFO_STREAM("Switch off the Camera");
    }

    if (msg.get()->buttons[2] == 1)
    {
        useLaser = !useLaser;
        static std_msgs::Bool laser;
        laser.data = useLaser;
        laserPub.publish(laser);
        if (useLaser) ROS_INFO_STREAM("Switch on the Laser");
        else ROS_INFO_STREAM("Switch off the Laser");
    }

    if (msg.get()->buttons[3] == 1)
    {
        useTri = !useTri;
        if (useTri) ROS_INFO_STREAM("Switch to Trigger Control");
        else ROS_INFO_STREAM("Switch to Joy Control");
    }

    static geometry_msgs::Twist speed;
    float ang = 0;
    float lin = 0;
    if (useTri) {
        ang = abs(msg.get()->axes[3]) < 0.05 ? 0 : msg.get()->axes[3]*2;
        lin = (msg.get()->axes[2] - msg.get()->axes[5]) * 0.7;
    }
    else {
        ang = abs(msg.get()->axes[0]) < 0.05 ? 0 : msg.get()->axes[0]*2;
        lin = abs(msg.get()->axes[4]) < 0.05 ? 0 : msg.get()->axes[4]*1.5;
    }
    speed.angular.z = ang;
    speed.linear.x = lin;
    speedPub.publish(speed);
}