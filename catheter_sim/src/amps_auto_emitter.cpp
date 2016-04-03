#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <iostream>
using namespace std;

int main(int argc, char **argv) 
{ 
    ros::init(argc,argv,"amps_auto_emitter"); 
    ros::NodeHandle n;
    ros::Publisher amps_pub = n.advertise<std_msgs::Float32>("amps", 1);

    std_msgs::Float32 amps;
    amps.data = 0;
    while(amps.data <0.2 && amps.data > -0.2)
    {
        amps_pub.publish(amps);
        amps.data += 0.0005;
        ROS_INFO("The current is %f amps", amps.data);
        ros::Duration(0.1).sleep(); // the duration should be longer than 0.02
    }


    return 0;
}