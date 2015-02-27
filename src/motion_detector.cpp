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

image_transport::Publisher image_pub;
Mat prev, current, flow, cflow, frame;
Mat pre, next, curr, gen;
Ptr<BackgroundSubtractor> pMOG2;
RNG rng(12345);
int x_start = 10, x_stop;
int y_start = 350, y_stop = 530;
int max_deviation = 20;
Scalar color(0,255,255);
int dilation_size = 5;
image_transport::Publisher pub;

int option = 1;
void drawOptFlowMap(const Mat& flow, Mat& cflowmap, int step,
    double threshold, const Scalar& color)
{
  for(int y = 0; y < cflowmap.rows; y += step)
    for(int x = 0; x < cflowmap.cols; x += step)
    {
      const Point2f& fxy = flow.at<Point2f>(y, x);
      if( sqrt((fxy.x*fxy.x)+(fxy.y*fxy.y)) > threshold ) {
        cflowmap.at<uchar>(Point(x,y)) = 255;
      }
      else {
        cflowmap.at<uchar>(Point(x,y)) = 0;
        prev =prev;
      }

    }
}
void processVideo(const sensor_msgs::Image::ConstPtr& msg) {

  cv_bridge::CvImagePtr cv_ptr, other;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    other = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  curr = cv_ptr->image;
  pMOG2->apply(curr, gen);
  Mat element = getStructuringElement( MORPH_ELLIPSE, Size( 2*dilation_size+1, 2*dilation_size+1 ), Point( dilation_size, dilation_size ) );

  /// Apply the specified morphology operation
  morphologyEx( gen, curr, 2, element );
  /**************************************************************************/
  /*// Size for the images
  CvMemStorage* storage;
  CvSize imgSize;
  CvSeq* contour;
  next = gen; */
  Mat threshold_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  
  // Use p-theorem
  threshold( curr, threshold_output, 100, 255, THRESH_BINARY );
  findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  vector<vector<Point> > contours_poly( contours.size() );
  vector<Rect> boundRect( contours.size() );
  vector<Point2f>center( contours.size() );
  vector<float>radius( contours.size() );

  for( int i = 0; i < contours.size(); i++ )
  { approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
    boundRect[i] = boundingRect( Mat(contours_poly[i]) );
    minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
  }
  Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
  for( int i = 0; i< contours.size(); i++ )
  {
    if( cv::norm(boundRect[i].tl() - boundRect[i].br() ) > 100 ) {
      Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
      //drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
      rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
      rectangle( other->image, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
    }
  }
  /**************************************************************************/
  //imshow("flow", other->image);
  pub.publish(other->toImageMsg());
  cv::waitKey(30);

}

void imageGaussian(const sensor_msgs::Image::ConstPtr& msg) {

  processVideo(msg);  // THIS IS WHERE WE NEED TO PUT OUR VIDEO READ THISPLSSSSSSSSS

  return;
}
void imageCb(const sensor_msgs::Image::ConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr, other;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    other = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  swap(current,cv_ptr->image);

  if( prev.data ) {
    calcOpticalFlowFarneback(prev, current, flow, 0.5, 3, 15, 10, 5, 1.2, 0);
    // cvtColor(prev, cflow, CV_GRAY2BGR);
    drawOptFlowMap(flow, cflow, 1, 2, CV_RGB(0, 255, 0));
      // Draw an example circle on the video stream
    // imshow("flow", cflow);
    //cv_ptr->image = cflow;
  // Mat element = getStructuringElement( MORPH_ELLIPSE, Size( 2*dilation_size+1, 2*dilation_size+1 ), Point( dilation_size, dilation_size ) );

  /// Apply the specified morphology operation
  // morphologyEx( cv_ptr->image, curr, 2, element );
  /**************************************************************************/
  /*// Size for the images
  CvMemStorage* storage;
  CvSize imgSize;
  CvSeq* contour;
  next = gen; */
  Mat threshold_output;
  vector<vector<Point> > contours;
  vector<uchar> hierarchy;
  
  // Use p-theorem
  /*// threshold( cv_ptr->image, threshold_output, 100, 255, THRESH_BINARY );
  findContours( cflow, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  vector<vector<Point> > contours_poly( contours.size() );
  vector<Rect> boundRect( contours.size() );
  vector<Point2f>center( contours.size() );
  vector<float>radius( contours.size() );

  for( int i = 0; i < contours.size(); i++ )
  { approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
    boundRect[i] = boundingRect( Mat(contours_poly[i]) );
    minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
  }
  Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
  for( int i = 0; i< contours.size(); i++ )
  {
    if( cv::norm(boundRect[i].tl() - boundRect[i].br() ) > 150 ) {
      Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
      //drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
      //rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
      rectangle( cv_ptr->image, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
    }
  }  */
  /**************************************************************************/
    swap(cv_ptr->image, cflow);
  }

    // Update GUI Window
    // cv::imshow("flow", cflow);
    // sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cflow).toImageMsg();
    prev = current;
    pub.publish(cv_ptr->toImageMsg());
    //pub.publish(other->toImageMsg());
    cv::waitKey(30);
    
    // Output modified video stream
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
  prev = 0;
}

int main( int argc, char **argv ) {
	ros::init(argc, argv, "motion_detector");
  pMOG2 = createBackgroundSubtractorMOG2(); //MOG2 approach


  /*  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
      ros::NodeHandle nh; */
  ros::NodeHandle node;
  image_transport::ImageTransport it(node);
  image_transport::Subscriber mySub = it.subscribe("/camera/visible/image", 1, render);
  pub = it.advertise("/raw_image", 1);

  ros::Subscriber sub = node.subscribe("/chatter",1000, respondToRequest);
  ros::spin();

  return 0;
}
