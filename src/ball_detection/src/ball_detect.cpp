
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "core_msgs/ball_position.h"
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include <iostream>
#include <string>
#include <algorithm>
#include <stdlib.h>
#include <math.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv_modules.hpp>
#include <stdio.h>

#define PI 3.14159265

using namespace std;
using namespace cv;

// Setting Thresholds for red and blue part of image.
// Changable to fit your enviroment. If you want to use bgr, it should be different.

int low_h2_r=169, high_h2_r=180;
int low_h_r=0, low_s_r=134, low_v_r=106;
int high_h_r=8, high_s_r=255, high_v_r=255;
int low_h_b=100, low_s_b=126, low_v_b=60;
int high_h_b=121, high_s_b=255, high_v_b=255;
int low_h_g=50, low_s_g=126, low_v_g=60;
int high_h_g=70, high_s_g=255, high_v_g=255;      // 임의로 초록색 범위 설정 실험후 조정 필요할듯?
int low_b_b=150;

// Initialization of variable for camera calibration paramters.
// You should change this if you changed the size of the image.

float intrinsic_data[9] = {1206.8897719532354, 0.0, 960.5, 0.0, 1206.8897719532354, 540.5, 0.0, 0.0, 1.0};
float distortion_data[5] = {0, 0, 0, 0, 0};

// Initialization of variable for text drawing
String text;
int iMin_tracking_ball_size = 5; // Minimum ball radius size by pixels. If ball is smaller than this, it won't be searched.
float fball_diameter = 0.14 ; // Initialization of variable for dimension of the target(real ball diameter by meter)

// Setting Mat variables for images.
Mat buffer;
Mat buffer2;
Mat result;

// Setting Publishers
ros::Publisher pub;
ros::Publisher pub_markers;

core_msgs::ball_position msg1;

// Declaring functions for image erode and dilaation.
void morphOps(Mat &thresh) {
  Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
  Mat dilateElement = getStructuringElement( MORPH_RECT,Size(3,3));
  morphologyEx(thresh, thresh, MORPH_CLOSE, erodeElement);
  morphologyEx(thresh, thresh, MORPH_OPEN, erodeElement);
}

// Declaration of functions that calculates the ball position from pixel position.
vector<float> pixel2point(Point center, int radius) {
  vector<float> position;
  float x, y, u, v, Xc, Yc, Zc;
  x = center.x;//.x;// .at(0);
  y = center.y;//.y;//
  u = (x-intrinsic_data[2])/intrinsic_data[0];
  v = (y-intrinsic_data[5])/intrinsic_data[4];
  Zc = (intrinsic_data[0]*fball_diameter)/(2*(float)radius) ;
  Xc = u*Zc ;
  Yc = v*Zc ;
  Xc = roundf(Xc * 1000) / 1000;
  Yc = roundf(Yc * 1000) / 1000;
  Zc = roundf(Zc * 1000) / 1000;
  position.push_back(Xc);
  position.push_back(Yc);
  position.push_back(Zc);
  return position;
}

// Changing int variable to string.
string intToString(int n) {
  stringstream s;
  s << n;
  return s.str();
}

// Changing float variable to string.
string floatToString(float f) {
  ostringstream buffer;
  buffer << f;
  return buffer.str();
}

void ball_detect()
{
  // Declare intrinsic and distortions by using the variable declared before.
  Mat intrinsic = Mat(3, 3, CV_32F, intrinsic_data);
  Mat distCoeffs = Mat(1, 5, CV_32F, distortion_data);

  // Declare another Mat variable to keep the image.
  Mat frame;
  frame = buffer;

  Mat calibrated_frame;
  // Undistort frame images and save to calibrated frame.
  undistort(frame, calibrated_frame, intrinsic, distCoeffs);

  // Defining Mat variables for Threshold images.

  Mat hsv_frame;
  Mat hsv_frame_red;
  Mat hsv_frame_red1;
  Mat hsv_frame_red2;
  Mat hsv_frame_blue;
  Mat hsv_frame_green;

  // Making clone of original image for drawing circles.
  result=calibrated_frame.clone();

  //Change RGB frame to HSV frame
  cvtColor(calibrated_frame, hsv_frame, cv::COLOR_BGR2HSV);

  //Threshold
  inRange(hsv_frame, Scalar(low_h_r,low_s_r,low_v_r), Scalar(high_h_r,high_s_r,high_v_r), hsv_frame_red1);
  inRange(hsv_frame, Scalar(low_h2_r,low_s_r,low_v_r), Scalar(high_h2_r,high_s_r,high_v_r), hsv_frame_red2);
  inRange(hsv_frame, Scalar(low_h_b,low_s_b,low_v_b), Scalar(high_h_b,high_s_b,high_v_b), hsv_frame_blue);
  inRange(hsv_frame, Scalar(low_h_g,low_s_g,low_v_g), Scalar(high_h_g,high_s_g,high_v_g), hsv_frame_green);
  addWeighted(hsv_frame_red1, 1.0, hsv_frame_red2, 1.0, 0.0, hsv_frame_red);


  //Blur and erode, dilate
  Mat erodeElement = getStructuringElement( MORPH_RECT,Size(5,5));
  Mat hsv_frame_red_1, hsv_frame_red_2, hsv_frame_blue_1, hsv_frame_blue_2, hsv_frame_green_1, hsv_frame_green_2;

  morphologyEx(hsv_frame_red, hsv_frame_red_1, MORPH_CLOSE, erodeElement);
  morphologyEx(hsv_frame_red_1, hsv_frame_red_2, MORPH_OPEN, erodeElement);
  GaussianBlur(hsv_frame_red_2, hsv_frame_red, cv::Size(9, 9), 2, 2);

  morphologyEx(hsv_frame_blue, hsv_frame_blue_1, MORPH_CLOSE, erodeElement);
  morphologyEx(hsv_frame_blue_1, hsv_frame_blue_2, MORPH_OPEN, erodeElement);
  GaussianBlur(hsv_frame_blue_2, hsv_frame_blue, cv::Size(9, 9), 2, 2);

  morphologyEx(hsv_frame_green, hsv_frame_green_1, MORPH_CLOSE, erodeElement);
  morphologyEx(hsv_frame_green_1, hsv_frame_green_2, MORPH_OPEN, erodeElement);
  GaussianBlur(hsv_frame_green_2, hsv_frame_green, cv::Size(9, 9), 2, 2);

  //Canny Edge Detection
  int lowThreshold=100;
  int ratio=3;
  int kernel_size=3;

  Mat img_canny_blue;
  Mat img_canny_red;
  Mat img_canny_green;

  Canny(hsv_frame_blue, img_canny_blue, lowThreshold, lowThreshold*ratio, kernel_size);
  Canny(hsv_frame_red, img_canny_red, lowThreshold, lowThreshold*ratio, kernel_size);
  Canny(hsv_frame_green, img_canny_green, lowThreshold, lowThreshold*ratio, kernel_size);
  //Finding Contours for blue threshold image
  vector<Vec4i> hierarchy_b;
  vector<vector<Point> > contours_b;
  findContours(img_canny_blue, contours_b, hierarchy_b, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));

  vector<Vec4i> hierarchy_r;
  vector<vector<Point> > contours_r;
  findContours(img_canny_red, contours_r, hierarchy_r, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));

  vector<Vec4i> hierarchy_g;
  vector<vector<Point> > contours_g;
  findContours(img_canny_green, contours_g, hierarchy_g, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));

  // Define variables for contour poly, center of circles, radius of circles
  vector<vector<Point> > contours_poly_b( contours_b.size() );
  vector<Point2f> center_b( contours_b.size() );
  vector<float> radius_b( contours_b.size() );

  vector<vector<Point> > contours_poly_r( contours_r.size() );
  vector<Point2f> center_r( contours_r.size() );
  vector<float> radius_r( contours_r.size() );

  vector<vector<Point> > contours_poly_g( contours_g.size() );
  vector<Point2f> center_g( contours_g.size() );
  vector<float> radius_g( contours_g.size() );

  /*Finding blue balls by contours
    Find polygon from contours and find the minimun size enclosing circle of that polygon.
  */

  for( size_t i = 0; i < contours_b.size(); i++ )
  {
    approxPolyDP( contours_b[i], contours_poly_b[i], 1, true );
    minEnclosingCircle( contours_poly_b[i], center_b[i], radius_b[i] );
  }

  for( size_t i = 0; i < contours_r.size(); i++ )
  {
    approxPolyDP( contours_r[i], contours_poly_r[i], 1, true );
    minEnclosingCircle( contours_poly_r[i], center_r[i], radius_r[i] );
  }

  for( size_t i = 0; i < contours_g.size(); i++ )
  {
    approxPolyDP( contours_g[i], contours_poly_g[i], 1, true );
    minEnclosingCircle( contours_poly_g[i], center_g[i], radius_g[i] );
  }

  // Declare message variable to publish
//  core_msgs::ball_position msg;
  int bball_num = 0;
  int rball_num = 0;
  int gball_num = 0;

  for (size_t i=0; i<contours_b.size(); i++)
  {
    if(radius_b[i] > iMin_tracking_ball_size)
    {
      //declare colors. Scalar(blue, green, red)
      Scalar color = Scalar(255,0,0);
      Scalar color_g = Scalar(0,255,0);

      // find the pixel point of the circle cneter, and the pixel radius of an circle

      float px_b = center_b[i].x;
      float py_b = center_b[i].y;
      float pr_b = radius_b[i];

      // change the pixel value to real world value

      vector<float> ball_pos_b;
      ball_pos_b = pixel2point(center_b[i],radius_b[i]);

      //draw the circle at the result Mat matrix
      //putText puts text at the matrix, puts text, at the point of an image

      float isx = ball_pos_b[0];
      float isy = ball_pos_b[1];
      float isz = ball_pos_b[2];

      string sx = floatToString(isx);
      string sy = floatToString(isy);
      string sz = floatToString(isz);

      string text;
      text = "x: " + sx +", y: " + sy + ", z: " + sz;
      putText(result, text, center_b[i], 2, 1, color_g, 2);
      circle(result, center_b[i], (int)radius_b[i], color, 2, 8, 0);
      bball_num = bball_num + 1;

      //push back variables of real ball position to the message variable
      msg1.blue_x.push_back(ball_pos_b[0]);
      msg1.blue_y.push_back(ball_pos_b[2]);
    }
  }

  // do same procedure for red balls

  for (size_t i=0; i<contours_r.size(); i++)
  {
    if(radius_r[i] > iMin_tracking_ball_size)
    {
      Scalar color = Scalar(0,0,255);
      Scalar color_g = Scalar(0,255,0);

      float px_r = center_r[i].x;
      float py_r = center_r[i].y;
      float pr_r = radius_r[i];

      vector<float> ball_pos_r;
      ball_pos_r = pixel2point(center_r[i],radius_r[i]);


      float isx = ball_pos_r[0];
      float isy = ball_pos_r[1];
      float isz = ball_pos_r[2];

      string sx = floatToString(isx);
      string sy = floatToString(isy);
      string sz = floatToString(isz);

      string text;
      text = "x: " + sx +", y: " + sy + ", z: " + sz;
      putText(result, text, center_r[i], 2, 1, color_g, 2);
      circle(result, center_r[i], (int)radius_r[i], color, 2, 8, 0);
      rball_num = rball_num + 1;
      msg1.red_x.push_back(ball_pos_r[0]);
      msg1.red_y.push_back(ball_pos_r[2]);
    }
  }

  for (size_t i=0; i<contours_g.size(); i++)
  {
    if(radius_g[i] > iMin_tracking_ball_size)
    {
      //declare colors. Scalar(blue, green, red)
      Scalar color = Scalar(255,0,0);
      Scalar color_g = Scalar(0,255,0);

      // find the pixel point of the circle cneter, and the pixel radius of an circle

      float px_g = center_g[i].x;
      float py_g = center_g[i].y;
      float pr_g = radius_g[i];

      // change the pixel value to real world value

      vector<float> ball_pos_g;
      ball_pos_g = pixel2point(center_g[i],radius_g[i]);

      //draw the circle at the result Mat matrix
      //putText puts text at the matrix, puts text, at the point of an image

      float isx = ball_pos_g[0];
      float isy = ball_pos_g[1];
      float isz = ball_pos_g[2];

      string sx = floatToString(isx);
      string sy = floatToString(isy);
      string sz = floatToString(isz);

      string text;
      text = "x: " + sx +", y: " + sy + ", z: " + sz;
      putText(result, text, center_g[i], 2, 1, color_g, 2);
      circle(result, center_g[i], (int)radius_g[i], color, 2, 8, 0);
      gball_num = gball_num + 1;

      //push back variables of real ball position to the message variable
      msg1.green_x.push_back(ball_pos_g[0]);
      msg1.green_y.push_back(ball_pos_g[2]);
    }
  }

  msg1.blue_num = bball_num;
  msg1.red_num = rball_num;
  msg1.green_num = gball_num;
  //show what is published at the terminal
  cout<<msg1.blue_num<<endl;
  for (int i=0 ; i<bball_num ; i++)
  {
    cout<<msg1.blue_x[i]<<endl;
    cout<<msg1.blue_y[i]<<endl;
  }

  cout<<msg1.red_num<<endl;
  for (int i=0 ; i<rball_num ; i++)
  {
    cout<<msg1.red_x[i]<<endl;
    cout<<msg1.red_y[i]<<endl;
  }

  cout<<msg1.green_num<<endl;
  for (int i=0 ; i<gball_num ; i++)
  {
    cout<<msg1.green_x[i]<<endl;
    cout<<msg1.green_y[i]<<endl;
  }
//  pub.publish(msg); //publish a message
  imshow("result", result);
}

void ball_check(){
//  core_msgs::ball_position msg;
//  ROS_INFO("%s", buffer2.at<Vec3b>(320,240)[0].c_str());          //확인해보려고 출력해보려했는데 오류떠서 지워놨습니다.
  if(buffer2.at<Vec3b>(320,240)[0]>low_b_b){            //두번째 카메라에서 (320,240)의 b값이 최소값보다 큰지 확인한건데 원하는 위치에 맞게 조정 및 파란색만 잘 확인 될지
    msg1.still_blue = 1;
  }
  else{
    msg1.still_blue = 0;
  }
//  pub.publish(msg); //publish a message
}

void imageCallback1(const sensor_msgs::ImageConstPtr& msg) {
  if(msg->height==480&&buffer.size().width==320) {  //check the size of the image received. if the image have 640x480, then change the buffer size to 640x480.
    std::cout<<"resized"<<std::endl;
    cv::resize(buffer,buffer,cv::Size(640,480));
  }
  try {buffer = cv_bridge::toCvShare(msg, "bgr8")->image;} //transfer the image data into buffer
  catch (cv_bridge::Exception& e) {ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());}
  ball_detect();
  waitKey(1);
}

void imageCallback2(const sensor_msgs::ImageConstPtr& msg) {
  if(msg->height==480&&buffer.size().width==320) {  //check the size of the image received. if the image have 640x480, then change the buffer size to 640x480.
    std::cout<<"resized"<<std::endl;
    cv::resize(buffer,buffer,cv::Size(640,480));
  }
  try {buffer2 = cv_bridge::toCvShare(msg, "bgr8")->image;} //transfer the image data into buffer
  catch (cv_bridge::Exception& e) {ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());}
  ball_check();
  waitKey(1);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "ball_detect_node"); //init ros nodd
  ros::NodeHandle nh; //create node handler
  image_transport::ImageTransport it(nh); //create image transport and connect it to node hnalder
  image_transport::Subscriber sub1 = it.subscribe("/camera1/rgb/image_raw", 1, imageCallback1);            //카메라 패키지의 이름을 변경할 수 있다고 가정하고 작성함
  image_transport::Subscriber sub2 = it.subscribe("/camera2/rgb/image_raw", 1, imageCallback2);
  pub = nh.advertise<core_msgs::ball_position>("/position", 1); //setting publisher
  ros::Rate loop_rate(30);
  namedWindow("result", WINDOW_NORMAL);
  while(ros::ok()){
     ros::spinOnce();
     pub.publish(msg1);
     loop_rate.sleep();
  }

  return 0;
}
