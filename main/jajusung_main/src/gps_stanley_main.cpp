#include "gps_stanley_h.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");

  GPS_STANLEY stanley_0818;

  ros::spin();

  return 0;
}