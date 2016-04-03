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
    while(ros::ok())
    {
    	cout<<"input the current (-1~1 amps) or 100 to quit: ";
    	cin>>amps.data;
    	if (amps.data <1.001 && amps.data > -1.001 )
    		amps_pub.publish(amps);
    	if (amps.data ==100)
    		break;
    }

    return 0;
}
