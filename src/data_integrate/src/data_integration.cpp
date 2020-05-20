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

enum class CameraLinePosition : int32_t
{
  NONE = 0,
  FAR_LEFT = 1,
  LEFT = 2,
  CENTER = 3,
  RIGHT = 4,
  FAR_RIGHT = 5,
};

enum class LineTracerMode
{
  STRAIGHT,
  TURNING,
};

enum class LineTracerMoveDirection
{
  STRAIGHT,
  LINE_LEFT,
  LINE_RIGHT,
  TURN_LEFT,
  TURN_RIGHT,
};

/**
 * 라인트레이서 모드 실행
 */
void updateLineTracerState(LineTracerMode& mode, LineTracerMoveDirection& moveDirection,
                           CameraLinePosition cameraLinePosition)
{
  int straight_count = 0, curve_count = 0;
  if(straight_count==3 && curve_count==3){
    std::cout << "Line Tracer mode is over." << endl;
    return; // 탈출 조건 설정해야함!!!! 
  }
  else{
    if (mode == LineTracerMode::STRAIGHT)
    {
      if (cameraLinePosition == CameraLinePosition::FAR_LEFT)
      {
        // 왼쪽으로 U턴을 시작한다
        mode = LineTracerMode::TURNING;
        moveDirection = LineTracerMoveDirection::TURN_LEFT;
        straight_count += 1;
        return;
      }
      if (cameraLinePosition == CameraLinePosition::FAR_RIGHT)
      {
        // 오른쪽으로 U턴을 시작한다
        mode = LineTracerMode::TURNING;
        moveDirection = LineTracerMoveDirection::TURN_RIGHT;
        straight_count += 1;
        return;
      }

      switch (moveDirection)
      {
        case LineTracerMoveDirection::STRAIGHT:
          switch (cameraLinePosition)
          {
            case CameraLinePosition::CENTER:
              // 직진하는 상태를 유지한다
              break;
            case CameraLinePosition::LEFT:
              // 왼쪽으로 보정한다
              moveDirection = LineTracerMoveDirection::LINE_LEFT;
              break;
            case CameraLinePosition::RIGHT:
              // 오른쪽으로 보정한다
              moveDirection = LineTracerMoveDirection::LINE_RIGHT;
              break;
            case CameraLinePosition::NONE:
              // 갑자기 검은 선이 사라짐...어떻게 처리해야 할까?
              break;
            case CameraLinePosition::FAR_LEFT:
            case CameraLinePosition::FAR_RIGHT:
              // 이 상태는 나올 수 없음
              break;
            default:
              // 잘못된 상태, 논리적 오류?
              break;
          }
          break;

        case LineTracerMoveDirection::LINE_LEFT:
          switch (cameraLinePosition)
          {
            case CameraLinePosition::LEFT:
              // 왼쪽으로 보정하는 상태를 유지한다
              break;
            case CameraLinePosition::CENTER:
              // 직진 상태로 복귀한다
              moveDirection = LineTracerMoveDirection::STRAIGHT;
              break;
            case CameraLinePosition::RIGHT:
            case CameraLinePosition::NONE:
              // 오른쪽으로 보정한다
              moveDirection = LineTracerMoveDirection::LINE_RIGHT;
            case CameraLinePosition::FAR_LEFT:
            case CameraLinePosition::FAR_RIGHT:
              // 이 상태는 나올 수 없음
              break;
            default:
              // 잘못된 상태, 논리적 오류?
              break;
          }
          break;

        case LineTracerMoveDirection::LINE_RIGHT:
          switch (cameraLinePosition)
          {
            case CameraLinePosition::RIGHT:
              // 오른쪽으로 보정하는 상태를 유지한다
              break;
            case CameraLinePosition::CENTER:
              // 직진 상태로 복귀한다
              moveDirection = LineTracerMoveDirection::STRAIGHT;
              break;
            case CameraLinePosition::LEFT:
            case CameraLinePosition::NONE:
              // 왼쪽으로 보정한다
              moveDirection = LineTracerMoveDirection::LINE_LEFT;
            case CameraLinePosition::FAR_LEFT:
            case CameraLinePosition::FAR_RIGHT:
              // 이 상태는 나올 수 없음
              break;
            default:
              // 잘못된 상태, 논리적 오류?
              break;
          }
          break;

        case LineTracerMoveDirection::TURN_LEFT:
        case LineTracerMoveDirection::TURN_RIGHT:
          // 이 상태는 나올 수 없음
          break;
        default:
          // 잘못된 상태, 논리적 오류?
          break;
      }
    }
    else if (mode == LineTracerMode::TURNING)
    {
      switch (moveDirection)
      {
        case LineTracerMoveDirection::TURN_LEFT:
          switch (cameraLinePosition)
          {
            case CameraLinePosition::LEFT:
            case CameraLinePosition::FAR_LEFT:
              // 왼쪽으로 U턴을 계속 한다
              break;
            case CameraLinePosition::CENTER:
            case CameraLinePosition::RIGHT:
            case CameraLinePosition::FAR_RIGHT:
            case CameraLinePosition::NONE:
              // 직진 모드로 되돌아간다
              mode = LineTracerMode::STRAIGHT;
              moveDirection = LineTracerMoveDirection::STRAIGHT;
              curve_count += 1;
              break;
            default:
              // 잘못된 상태, 논리적 오류?
              break;
          }

        case LineTracerMoveDirection::TURN_RIGHT:
          switch (cameraLinePosition)
          {
            case CameraLinePosition::RIGHT:
            case CameraLinePosition::FAR_RIGHT:
              // 오른쪽으로 U턴을 계속 한다
              break;
            case CameraLinePosition::CENTER:
            case CameraLinePosition::LEFT:
            case CameraLinePosition::FAR_LEFT:
            case CameraLinePosition::NONE:
              // 직진 모드로 되돌아간다
              mode = LineTracerMode::STRAIGHT;
              moveDirection = LineTracerMoveDirection::STRAIGHT;
              curve_count += 1;
              break;
            default:
              // 잘못된 상태, 논리적 오류?
              break;
          }

        case LineTracerMoveDirection::STRAIGHT:
        case LineTracerMoveDirection::LINE_LEFT:
        case LineTracerMoveDirection::LINE_RIGHT:
          // 이 상태는 나올 수 없음
          break;
        default:
          // 잘못된 상태, 논리적 오류?
          break;
      }
    }
    else
    {
      // 잘못된 상태, 논리적 오류?
      return;
    }
    }
  
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

  auto lineTracerMode = LineTracerMode::STRAIGHT;
  auto lineTracerMoveDirection = LineTracerMoveDirection::STRAIGHT;

  while (ros::ok)
  {
    updateLineTracerState(lineTracerMode, lineTracerMoveDirection, static_cast<CameraLinePosition>(section));

    switch (lineTracerMoveDirection)
    {
      case LineTracerMoveDirection::STRAIGHT:
        wheelController.goForward();
        break;
      case LineTracerMoveDirection::LINE_LEFT:
        wheelController.goForwardAdjustLeft();
        break;
      case LineTracerMoveDirection::LINE_RIGHT:
        wheelController.goForwardAdjustRight();
        break;
      case LineTracerMoveDirection::TURN_LEFT:
        wheelController.smoothTurnLeft();
        break;
      case LineTracerMoveDirection::TURN_RIGHT:
        wheelController.smoothTurnRight();
        break;
      default:
        // 잘못된 상태, 논리적 오류?
        break;
    }

    ros::Duration(0.025).sleep();
    ros::spinOnce();
  }

  return 0;
}
