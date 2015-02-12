#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>

using namespace std;
using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";
image_transport::Publisher image_pub;
Mat prev, current, flow, cflow, frame;
Mat curr, gen;

int option = 0;
void drawOptFlowMap(const Mat& flow, Mat& cflowmap, int step,
    double scale, const Scalar& color)
{
  for(int y = 0; y < cflowmap.rows; y += step)
    for(int x = 0; x < cflowmap.cols; x += step)
    {
      const Point2f& fxy = flow.at<Point2f>(y, x);
      line(cflowmap, Point(x,y), Point(cvRound(x+fxy.x), cvRound(y+fxy.y)),
          color);
      circle(cflowmap, Point(x,y), 2, color, -1);
    }
}
void processVideo(const sensor_msgs::Image::ConstPtr& msg) {
  //create Background Subtractor objects
  Ptr<BackgroundSubtractor> pMOG2 = createBackgroundSubtractorMOG2(); //MOG2 approach
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  curr = cv_ptr->image;
  pMOG2->apply(curr, gen);
  //stringstream ss;
  //rectangle(curr, cv::Point(10, 2), cv::Point(100,20),
  //            cv::Scalar(255,255,255), -1);
  //ss << 
  imshow("flow", gen);

}

void imageGaussian(const sensor_msgs::Image::ConstPtr& msg) {
  namedWindow("Frame");
  namedWindow("FG Mask MOG 2");



  processVideo(msg);  // THIS IS WHERE WE NEED TO PUT OUR VIDEO READ THISPLSSSSSSSSS

  //destroy GUI windows
  destroyAllWindows();
  return ;
}
void imageCb(const sensor_msgs::Image::ConstPtr& msg)
{
  ros::NodeHandle node;
  image_transport::ImageTransport it(node);
  image_pub = it.advertise("/image_converter/output_video", 1);
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  current = cv_ptr->image;
  Mat foobar;
  resize(current, foobar, Size(current.size().width, current.size().height) );
  current = foobar;

  if( prev.data ) {
    calcOpticalFlowFarneback(prev, current, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
    cvtColor(prev, cflow, CV_GRAY2BGR);
    drawOptFlowMap(flow, cflow, 16, 1.5, CV_RGB(0, 255, 0));
    imshow("flow", cflow);
    cout << "FUUUUUUUUUUUUUUUUUUUUUCK!!!!!!" << endl;
  }

  // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(10);
    
    // Output modified video stream
    image_pub.publish(cv_ptr->toImageMsg());
    swap(prev, current);
}

void render( const sensor_msgs::Image::ConstPtr & msg ) {
  // cout << "RENDER IT YOURSELF!!!" << endl;
  if( option == 0 )
  imageGaussian( msg );
  if( option == 1 )
  imageCb( msg );
}
void respondToRequest( const std_msgs::String::ConstPtr & msg ) {
  if( msg->data.c_str()[0] == '1' ) {
    cout << "Received option 1";
    option = 1;
  }
  if( msg->data.c_str()[0] == '2' ) {
    cout << "Received option 2";
    option = 0;
  }
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
