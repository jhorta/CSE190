#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sstream>
#include <iostream>

using namespace std;

void respondToRequest( const std_msgs::String::ConstPtr & msg ) {
  if( msg->data.c_str()[0] == '1' )
    cout << "Received option 1";
  if( msg->data.c_str()[0] == '2' )
    cout << "Received option 2";
  cout << endl;
}

int main( int argc, char **argv ) {
	ros::init(argc, argv, "motion_detector");
  ros::NodeHandle node;

  ros::Subscriber sub = node.subscribe("/chatter",1000, respondToRequest);

  ros::spin();

  return 0;
}
