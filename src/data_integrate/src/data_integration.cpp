#include <arpa/inet.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <algorithm>
#include <boost/thread.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>

#include <ros/package.h>
#include <ros/ros.h>
#include "core_msgs/ball_position.h"
#include "core_msgs/line_info.h"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"

#include "opencv2/opencv.hpp"

#define RAD2DEG(x) ((x)*180. / M_PI)

boost::mutex map_mutex;

int lidar_size;
float lidar_degree[400];
float lidar_distance[400];
float lidar_obs;

int32_t blue_num, red_num, green_num, section;
bool is_bump, still_blue;
float blue_x[20], red_x[20], green_x[20];
float blue_y[20], red_y[20], green_y[20];
float blue_distance[20], red_distance[20], green_distance[20];
int near_ball;

int action;

int len;
int n;

#define RAD2DEG(x) ((x)*180. / M_PI)

void lidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  map_mutex.lock();

  int count = scan->angle_max / scan->angle_increment;
  lidar_size = count;
  for (int i = 0; i < count; i++)
  {
    lidar_degree[i] = RAD2DEG(scan->angle_min + scan->angle_increment * i);
    lidar_distance[i] = scan->ranges[i];
  }
  map_mutex.unlock();
}

void camera_Callback_1(const core_msgs::ball_position::ConstPtr& position)
{
  blue_num = position->blue_num;
  red_num = position->red_num;
  // Note: There is only 1 green ball, but we track # of green balls anyway for consistency
  green_num = position->green_num;
  still_blue = position->still_blue;

  for (int i = 0; i < blue_num; i++)
  {
    blue_x[i] = position->blue_x[i];
    blue_y[i] = position->blue_y[i];
    blue_distance[i] = position->blue_z[i];
  }
  for (int i = 0; i < red_num; i++)
  {
    red_x[i] = position->red_x[i];
    red_y[i] = position->red_y[i];
    red_distance[i] = position->red_z[i];
  }
  for (int i = 0; i < green_num; i++)
  {
    green_x[i] = position->green_x[i];
    green_y[i] = position->green_y[i];
    green_distance[i] = position->green_z[i];
  }
}

void camera_Callback_2(const core_msgs::line_info::ConstPtr& line_info)
{
  is_bump = line_info->is_bump;
  section = line_info->section;
}

//-------- 운전 관련 함수 --------//
// 왼쪽 바퀴, 오른쪽 바퀴에 신호를 보내는 함수들입니다.

class WheelController
{
public:
  WheelController(const ros::Publisher& fl_wheel, const ros::Publisher& fr_wheel, const ros::Publisher& bl_wheel,
                  const ros::Publisher& br_wheel)
    : fl_wheel_(fl_wheel), fr_wheel_(fr_wheel), bl_wheel_(bl_wheel), br_wheel_(br_wheel)
  {
  }

  /**
   * 로봇의 선속도와 각속도를 설정한다.
   *
   * @param linear_speed 선속도. 전진은 +, 후진은 -, 정지는 0.
   *    단위는 m/s
   * @param angular_speed 각속도. 반시계방향(좌회전)은 +, 시계방향(우회전)은 -, 직진/정지는 0.
   *    단위는 rad/s
   */
  void setSpeed(double linear_speed, double angular_speed)
  {
    linear_speed_ = linear_speed;
    angular_speed_ = angular_speed;

    // turtlebot의 바퀴 반지름은 0.033m이다.
    // TODO: 나중에 우리 로봇의 바퀴 크기에 맞춰 비례상수를 다시 계산해야 한다.
    const double LINEAR_SPEED_FACTOR = 1 / 0.033;
    // TODO: 로봇의 각속도를 바퀴의 회전속도로 변환하는 비례상수는 지금은 알 수 없다.
    // 일단 1로 놓고 나중에 제대로 구해보자.
    const double ANGULAR_SPEED_FACTOR = 1.0;

    setWheelSpeeds(linear_speed_ * LINEAR_SPEED_FACTOR - angular_speed_ * ANGULAR_SPEED_FACTOR,
                   linear_speed_ * LINEAR_SPEED_FACTOR + angular_speed_ * ANGULAR_SPEED_FACTOR,
                   linear_speed_ * LINEAR_SPEED_FACTOR - angular_speed_ * ANGULAR_SPEED_FACTOR,
                   linear_speed_ * LINEAR_SPEED_FACTOR + angular_speed_ * ANGULAR_SPEED_FACTOR);
  }

  /**
   * 로봇의 선속도를 설정한다. 각속도는 기존의 값을 유지한다.
   *
   * @param linear_speed 선속도. 전진은 +, 후진은 -, 정지는 0.
   *    단위는 m/s
   */
  void setLinearSpeed(double linear_speed)
  {
    setSpeed(linear_speed, angular_speed_);
  }

  /**
   * 로봇의 각속도를 설정한다. 선속도는 기존의 값을 유지한다.
   *
   * @param angular_speed 각속도. 반시계방향(좌회전)은 +, 시계방향(우회전)은 -, 직진/정지는 0.
   *    단위는 rad/s
   */
  void setAngularSpeed(double angular_speed)
  {
    setSpeed(linear_speed_, angular_speed);
  }

private:
  const ros::Publisher& fl_wheel_;
  const ros::Publisher& fr_wheel_;
  const ros::Publisher& bl_wheel_;
  const ros::Publisher& br_wheel_;

  double linear_speed_ = 0.0;
  double angular_speed_ = 0.0;

  /**
   * 4개의 바퀴에 각각의 속도를 지정한다.
   */
  void setWheelSpeeds(double fl_speed, double fr_speed, double bl_speed, double br_speed) const
  {
    std_msgs::Float64 fl_wheel_msg;
    std_msgs::Float64 fr_wheel_msg;
    std_msgs::Float64 bl_wheel_msg;
    std_msgs::Float64 br_wheel_msg;

    fl_wheel_msg.data = fl_speed;
    fr_wheel_msg.data = fr_speed;
    bl_wheel_msg.data = bl_speed;
    br_wheel_msg.data = br_speed;

    fl_wheel_.publish(fl_wheel_msg);
    fr_wheel_.publish(fr_wheel_msg);
    bl_wheel_.publish(bl_wheel_msg);
    br_wheel_.publish(br_wheel_msg);
  }
};

enum class CameraLinePosition : int32_t
{
  NONE = 0,
  FAR_LEFT = 1,
  LEFT = 2,
  CENTER = 3,
  RIGHT = 4,
  FAR_RIGHT = 5,
};

enum class TurnDirection
{
  LEFT,
  RIGHT,
};

const double LINEAR_SPEED_FAST = 1.0;
const double LINEAR_SPEED_SLOW = 0.2;
const double ANGULAR_SPEED_FAST = M_PI / 2;
const double ANGULAR_SPEED_SLOW = M_PI / 6;

/**
 * 라인트레이서 모드 실행
 */
void updateLineTracerState(WheelController& wheel_controller, CameraLinePosition cameraLinePosition,
                           TurnDirection& last_turn_direction)
{
  double linear_speed = 0;
  double angular_speed = 0;

  if (is_bump)
    linear_speed = LINEAR_SPEED_SLOW;
  else
    linear_speed = LINEAR_SPEED_FAST;

  switch (cameraLinePosition)
  {
    case CameraLinePosition::CENTER:
      angular_speed = 0;
      break;
    case CameraLinePosition::LEFT:
      last_turn_direction = TurnDirection::LEFT;
      angular_speed = ANGULAR_SPEED_SLOW;
      break;
    case CameraLinePosition::RIGHT:
      last_turn_direction = TurnDirection::RIGHT;
      angular_speed = -ANGULAR_SPEED_SLOW;
      break;
    case CameraLinePosition::FAR_LEFT:
      last_turn_direction = TurnDirection::LEFT;
      // is_bump에 상관없이 선속도는 느리게 유지한다.
      linear_speed = LINEAR_SPEED_SLOW;
      angular_speed = ANGULAR_SPEED_FAST;
      break;
    case CameraLinePosition::FAR_RIGHT:
      last_turn_direction = TurnDirection::RIGHT;
      // is_bump에 상관없이 선속도는 느리게 유지한다.
      linear_speed = LINEAR_SPEED_SLOW;
      angular_speed = -ANGULAR_SPEED_FAST;
      break;
    case CameraLinePosition::NONE:
      // 로봇이 검은 줄을 놓쳤다.
      // is_bump에 상관없이 선속도는 느리게 유지한다.
      linear_speed = LINEAR_SPEED_SLOW;
      // 가장 최근에 회전했던 방향의 반대방향으로 지속적으로 회전하면서 검은 줄을 찾아보자.
      if (last_turn_direction == TurnDirection::RIGHT)
        angular_speed = ANGULAR_SPEED_FAST;
      else
        angular_speed = -ANGULAR_SPEED_FAST;
      break;
    default:
      std::cerr << "[LOGIC ERROR] Unknown camera line position: " << static_cast<int>(cameraLinePosition) << std::endl;
      break;
  }

  wheel_controller.setSpeed(linear_speed, angular_speed);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "data_integation");
  ros::NodeHandle n;

  ros::Subscriber sub_lidar = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_Callback);
  ros::Subscriber sub_upper_webcam = n.subscribe<core_msgs::ball_position>("/position", 1000, camera_Callback_1);
  ros::Subscriber sub_lower_webcam = n.subscribe<core_msgs::line_info>("/line_info", 1000, camera_Callback_2);
  ros::Publisher fl_wheel = n.advertise<std_msgs::Float64>("/run_2/FLwheel_velocity_controller/command", 10);
  ros::Publisher fr_wheel = n.advertise<std_msgs::Float64>("/run_2/FRwheel_velocity_controller/command", 10);
  ros::Publisher bl_wheel = n.advertise<std_msgs::Float64>("/run_2/BLwheel_velocity_controller/command", 10);
  ros::Publisher br_wheel = n.advertise<std_msgs::Float64>("/run_2/BRwheel_velocity_controller/command", 10);

  WheelController wheelController(fl_wheel, fr_wheel, bl_wheel, br_wheel);

  auto last_turn_direction = TurnDirection::LEFT;

  while (ros::ok())
  {
    updateLineTracerState(wheelController, static_cast<CameraLinePosition>(section), last_turn_direction);
    ros::Duration(0.025).sleep();
    ros::spinOnce();
  }

  return 0;
}
