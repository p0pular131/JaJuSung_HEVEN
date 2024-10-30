#include "gps_stanley_usingekf_h.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");
  
  GPS_STANLEY stanley;
  
  ros::spin();

  return 0;
}