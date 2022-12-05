#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
using namespace std;


void pos_Cb(const geometry_msgs::PoseStampedConstPtr &pos_msg)
{

    double x_position = pos_msg->pose.position.x;
    double y_position = pos_msg->pose.position.y;
    cout << "x:"<<x_position << " " <<"y:"<< y_position<<endl;
    if(x_position<=5.2){
        if(y_position<-6.5){
            cout<<"POS:C"<<endl;
        }
        else if(y_position>-3.1){
            cout<<"POS:A"<<endl;
        }
        else{
            cout<<"POS:B"<<endl;
        }
    }
    else{
        cout<<"POS:D"<<endl;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "location");
    ros::NodeHandle n;
    ros::Subscriber mapmeta_sub = n.subscribe("/slam_out_pose", 1000, pos_Cb);
    ros::spin();
    return 0;
}