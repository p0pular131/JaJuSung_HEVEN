#define _USE_MATH_DEFINES

#include "ros/ros.h"
#include "stanley_controller/lane_info.h"
#include "erp42_msgs/DriveCmd.h"
#include "erp42_msgs/ModeCmd.h"
#include <math.h>
#define STANLEY_K 0.25
#define VELOCITY 20
#define VEL 20

double delta = 0;
ros::Publisher cmd_pub;
ros::Publisher mod_pub;

int normalize_delta(int delta)
{
  delta = delta>=28 ? 28 : delta;
  delta = delta<=-28 ? -28 : delta;
  return delta;
}

// 받아오는 data: left_x, right_x, left_theta, right_theta

void chatterCallback(const stanley_controller::lane_info::ConstPtr& msg)
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

  erp42_msgs::DriveCmd drive_cmd;
  erp42_msgs::ModeCmd mode_cmd;

  drive_cmd.KPH = VEL;
  drive_cmd.Deg = normalize_delta(delta);
  mode_cmd.MorA = 0x01;
  mode_cmd.EStop = 0x00;
  mode_cmd.Gear = 0x00;
  cmd_pub.publish(drive_cmd);
  mod_pub.publish(mode_cmd);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");		// 노드 이름 초기화

  ros::NodeHandle nh;

  // Subscriber
  ros::Subscriber sub = nh.subscribe("/lane_result", 100, chatterCallback);

  // Publisher
  cmd_pub = nh.advertise<erp42_msgs::DriveCmd>("drive", 100);
  mod_pub = nh.advertise<erp42_msgs::ModeCmd>("mode", 100);
  ros::spin();		// 큐에 요청된 콜백함수를 처리하며, 프로그램 종료시까지 반복함

  return 0;
}