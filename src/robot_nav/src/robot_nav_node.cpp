#include <ros/ros.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>

ros::Publisher odomPub;
void odomCB(const geometry_msgs::Twist::ConstPtr &msg);
static int counter = 0;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_nav_node");
    ros::NodeHandle n;

    odomPub = n.advertise<geometry_msgs::TwistWithCovarianceStamped>("/cmd_twist", 20);

    ros::Subscriber keySub = n.subscribe("/vrep/cmd_vel", 20, odomCB);

    ROS_INFO_STREAM("Robot Nav Node STARTED.");
    ros::spin();
    return 0;
}
static float ing_test = 0;
void odomCB(const geometry_msgs::Twist::ConstPtr &msg)
{
    static geometry_msgs::TwistWithCovarianceStamped ts;
    float vL = (msg.get()->linear.x*6 - msg.get()->angular.z*0.6) * 0.195 / 2;
    float vR = (msg.get()->linear.x*6 + msg.get()->angular.z*0.6) * 0.195 / 2;
    ts.twist.twist.angular.z = (vR - vL) / 0.331;
    ts.twist.twist.linear.x = (vR + vL) / 2;
    ts.header.frame_id = "";
    ts.header.seq = counter++;
    static ros::Time last = ros::Time::now();
    ts.header.stamp = ros::Time::now();
    ing_test += (ts.header.stamp - last).toSec() * ts.twist.twist.angular.z;
    last = ts.header.stamp;
    odomPub.publish(ts);
    ROS_INFO_STREAM(ing_test/3.14*180.0f << "  " << ros::Time::isSimTime() << "  " << ros::Time::isSystemTime());
}
