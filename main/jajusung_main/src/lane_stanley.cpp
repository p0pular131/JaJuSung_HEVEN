#define _USE_MATH_DEFINES

#include "ros/ros.h"
#include "jajusung_main/lane_info.h"
#include "jajusung_main/HevenCtrlCmd.h"
#include <math.h>
#define STANLEY_K 0.25
#define VELOCITY 20
#define VEL 20

double delta = 0;
ros::Publisher cmd_pub;

int normalize_delta(int delta)
{
  delta = delta>=28 ? 28 : delta;
  delta = delta<=-28 ? -28 : delta;
  return delta;
}

// 받아오는 data: left_x, right_x, left_theta, right_theta

void chatterCallback(const jajusung_main::lane_info::ConstPtr& msg)
{
  double left_x = msg->left_x;
  double right_x = msg->right_x;
  double left_theta = msg->left_theta;
  double right_theta = msg->right_theta;

  double scale_factor = 4.0/(left_x+right_x);
  left_x *= scale_factor;
  right_x *= scale_factor;

  double lat_err = (left_x - right_x)/2.0;
  double head_err = ((left_theta-90.0)+(right_theta-90.0))/2.0;

  if (left_x == 160.0) {
    lat_err = (2.0 - right_x);
    head_err = 90.0 - right_theta;
  }
  else if (right_x == 160.0) {
    lat_err = -(2.0 - left_x);
    head_err = -(90.0 - left_theta);
  }
  else {
    lat_err = (left_x - right_x)/2;
    head_err = ((left_theta-90.0)+(right_theta-90.0))/2.0;
  }
  head_err *= M_PI/180.0;

  double delta = (head_err + atan(STANLEY_K*lat_err/((VELOCITY/3.6))))*180/M_PI; // rad to deg

  // ROS_INFO("%f", left_x);
  // ROS_INFO("%f", right_x);
  // ROS_INFO("%f", left_theta);
  // ROS_INFO("%f", right_theta);
  // ROS_INFO("lateral error: %.4f", lat_err);
  // ROS_INFO("heading error: %.4f", head_err);
  ROS_INFO("delta: %.4f", delta);

  jajusung_main::HevenCtrlCmd drive_cmd;

  drive_cmd.velocity = VEL;
  drive_cmd.steering = normalize_delta(delta);
  drive_cmd.brake = 0;
  cmd_pub.publish(drive_cmd);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");		// 노드 이름 초기화

  ros::NodeHandle nh;

  // Subscriber
  ros::Subscriber sub = nh.subscribe("/lane_result", 100, chatterCallback);

  // Publisher
  cmd_pub = nh.advertise<jajusung_main::HevenCtrlCmd>("drive", 100);
  ros::spin();		// 큐에 요청된 콜백함수를 처리하며, 프로그램 종료시까지 반복함

  return 0;
}