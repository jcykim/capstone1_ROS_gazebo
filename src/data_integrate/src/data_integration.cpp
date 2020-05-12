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
#include "core_msgs/lower_webcam.h"

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

int ball_number;
float ball_X[20];
float ball_Y[20];
float ball_distance[20];
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
void camera_Callback(const core_msgs::ball_position::ConstPtr& position)
{
  int count = position->size;
  ball_number = count;
  for (int i = 0; i < count; i++)
  {
    ball_X[i] = position->img_x[i];
    ball_Y[i] = position->img_y[i];
    // std::cout << "degree : "<< ball_degree[i];
    // std::cout << "   distance : "<< ball_distance[i]<<std::endl;
    ball_distance[i] = ball_X[i] * ball_X[i] + ball_Y[i] * ball_X[i];
  }
}

//-------- 운전 관련 함수 --------//
// 왼쪽 바퀴, 오른쪽 바퀴에 신호를 보내는 함수들입니다.

class WheelController
{
public:
  /** 전진 시 바퀴가 취하는 속도 */
  static constexpr double FORWARD_SPEED = 1;
  /** 전진 시 좌/우 보정을 위해 추가하는 속도 */
  static constexpr double FORWARD_SPEED_ADJUSTMENT = 0.1;
  /** 후진 시 바퀴가 취하는 속도 */
  static constexpr double BACKWARD_SPEED = -0.5;
  /** 정지 상태에서 선회할 때 각 바퀴가 취하는 속도(의 절대값) */
  static constexpr double TURN_SPEED = 0.5;
  /** U턴 시 안쪽 바퀴가 취하는 속도 */
  static constexpr double UTURN_INNER_SPEED = 0.5;
  /** U턴 시 바깥쪽 바퀴가 취하는 속도 */
  static constexpr double UTURN_OUTER_SPEED = 1;

  WheelController(const ros::Publisher& fl_wheel, const ros::Publisher& fr_wheel, const ros::Publisher& bl_wheel,
                  const ros::Publisher& br_wheel)
    : fl_wheel_(fl_wheel), fr_wheel_(fr_wheel), bl_wheel_(bl_wheel), br_wheel_(br_wheel)
  {
  }

  /**
   * 일직선으로 전진한다.
   */
  void goForward() const
  {
    setWheelSpeeds(FORWARD_SPEED, FORWARD_SPEED, FORWARD_SPEED, FORWARD_SPEED);
  }

  /**
   * 일직선으로 전진하되, 왼쪽으로 약간 보정한다.
   */
  void goForwardAdjustLeft() const
  {
    setWheelSpeeds(FORWARD_SPEED, FORWARD_SPEED + FORWARD_SPEED_ADJUSTMENT, FORWARD_SPEED,
                   FORWARD_SPEED + FORWARD_SPEED_ADJUSTMENT);
  }

  /**
   * 일직선으로 전진하되, 오른쪽으로 약간 보정한다.
   */
  void goForwardAdjustRight() const
  {
    setWheelSpeeds(FORWARD_SPEED + FORWARD_SPEED_ADJUSTMENT, FORWARD_SPEED, FORWARD_SPEED + FORWARD_SPEED_ADJUSTMENT,
                   FORWARD_SPEED);
  }

  /**
   * 일직선으로 후진한다.
   */
  void goBackward() const
  {
    setWheelSpeeds(BACKWARD_SPEED, BACKWARD_SPEED, BACKWARD_SPEED, BACKWARD_SPEED);
  }

  /**
   * 제자리에서 좌회전한다. (반시계 방향)
   */
  void turnLeft() const
  {
    setWheelSpeeds(-TURN_SPEED, TURN_SPEED, -TURN_SPEED, TURN_SPEED);
  }

  /**
   * 제자리에서 우회전한다. (시계 방향)
   */
  void turnRight() const
  {
    setWheelSpeeds(TURN_SPEED, -TURN_SPEED, TURN_SPEED, -TURN_SPEED);
  }

  /**
   * 부드럽게 곡선을 그리며 좌회전한다.
   */
  void smoothTurnLeft() const
  {
    setWheelSpeeds(UTURN_INNER_SPEED, UTURN_OUTER_SPEED, UTURN_INNER_SPEED, UTURN_OUTER_SPEED);
  }

  /**
   * 부드럽게 곡선을 그리며 우회전한다.
   */
  void smoothTurnRight() const
  {
    setWheelSpeeds(UTURN_OUTER_SPEED, UTURN_INNER_SPEED, UTURN_OUTER_SPEED, UTURN_INNER_SPEED);
  }

  /**
   * 제자리에 정지한다.
   */
  void stop() const
  {
    setWheelSpeeds(0, 0, 0, 0);
  }

private:
  const ros::Publisher& fl_wheel_;
  const ros::Publisher& fr_wheel_;
  const ros::Publisher& bl_wheel_;
  const ros::Publisher& br_wheel_;

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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "data_integation");
  ros::NodeHandle n;

  ros::Subscriber sub_lidar = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_Callback);
  ros::Subscriber sub_upper_webcam = n.subscribe<core_msgs::ball_position>("/position", 1000, camera_Callback_1);
  ros::Subscriber sub_lower_webcam = n.subscribe<core_msgs::lower_webcam>("/position", 1000, camera_Callback_2);
  ros::Publisher fl_wheel = n.advertise<std_msgs::Float64>("/run_2/FLwheel_velocity_controller/command", 10);
  ros::Publisher fr_wheel = n.advertise<std_msgs::Float64>("/run_2/FRwheel_velocity_controller/command", 10);
  ros::Publisher bl_wheel = n.advertise<std_msgs::Float64>("/run_2/BLwheel_velocity_controller/command", 10);
  ros::Publisher br_wheel = n.advertise<std_msgs::Float64>("/run_2/BRwheel_velocity_controller/command", 10);

  WheelController wheelController(fl_wheel, fr_wheel, bl_wheel, br_wheel);

  while (ros::ok)
  {
    // 운전 코드는 요기에!
    ros::Duration(0.025).sleep();
    ros::spinOnce();
  }

  return 0;
}
