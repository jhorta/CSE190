#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sstream>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

static const std::string OPENCV_WINDOW = "Image window";
image_transport::Publisher image_pub;

void imageCb(const sensor_msgs::Image::ConstPtr& msg)
{
  ros::NodeHandle node;
  image_transport::ImageTransport it(node);
  image_pub = it.advertise("/image_converter/output_video", 1);
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  } 
  // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    // Output modified video stream
    image_pub.publish(cv_ptr->toImageMsg());
}

void render( const sensor_msgs::Image::ConstPtr & msg ) {
  // cout << "RENDER IT YOURSELF!!!" << endl;
  imageCb( msg );
}
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
  ros::Subscriber sub_cam = node.subscribe("/camera/visible/image", 1, render);


/*  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // ...
}

ros::NodeHandle nh;
image_transport::ImageTransport it(nh);
image_transport::Subscriber sub = it.subscribe("in_image_base_topic", 1, imageCallback);
image_transport::Publisher pub = it.advertise("out_image_base_topic", 1);
image_transport::Subscriber sub = it.subscribe("/camera/visible/image", 1, imageCallback);
image_transport::Publisher pub = it.advertise("/camera/visible/image", 1); */

  ros::Subscriber sub = node.subscribe("/chatter",1000, respondToRequest);
  cv::namedWindow(OPENCV_WINDOW);
  ros::spin();
  cv::destroyWindow(OPENCV_WINDOW);

  return 0;
}
