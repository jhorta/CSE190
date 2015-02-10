#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sstream>
#include <iostream>

using namespace std;
int main( int argc, char **argv ) {
	ros::init(argc, argv, "motion_node_keyboard");
  ros::NodeHandle node;

  ros::Publisher pub = node.advertise<std_msgs::String>("/chatter", 1000);

  ros::Rate loop_rate(10);

  cout << "Press key 1 to enter Farneback Optical Flow Algorithm.\n"
    << "Press key 2 to enter Improved Mixture of Gaussian (MOG2) Background Subtraction Algorithm: ";
  int input;
  int prev = 0;
  while(ros::ok()) {
    std_msgs::String msg;
    std::stringstream ss;
    do {
      cin.clear();
      cin.ignore(10000,'\n');
      cin >> input;
      if( input == 1 ) {
        if( prev != input )
          cout << "Requesting Farneback Optical Flow Algorithm service." << endl;
        break;
      }
      else if ( input == 2 ) {
        if( prev != input )
          cout << "Requesting Improved Mixture of Gaussian (MOG2) Background Subtraction Algorithm service." << endl;
        break;
      }
      else {
        cout << "Entered invalid input." << endl;
      }
    } while( input != 1 || input != 2 );
    // ss << input;
    ss << input << endl;
    msg.data = ss.str();

    /*ros::Publisher move_pub = node.advertise<geometry_msgs::Twist>
      ("/cmd_vel_mux/input/teleop", 1);
      ros::Subscriber scan_pub = node.subscribe
      ("scan", 1, scanCallback); 
      ros::spin();
      */
    if( prev != input ) {
      pub.publish(msg);
      prev = input;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
