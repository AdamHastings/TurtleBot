#ifndef LOGITECH_JOY_H_
#define LOGITECH_JOY_H_

#include <ros/ros.h>
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include <iostream>

namespace logjoy {

class logitechJoyController {
public:
  logitechJoyController();
  ~logitechJoyController();
private:
  ros::NodeHandle* nh_;
  ros::NodeHandle* nh_private_;

  geometry_msgs::Twist vel_cmd_;

  ros::Subscriber joy_sub_;
  ros::Publisher cmd_pub_;

  void joyCallback(const sensor_msgs::Joy& msg);

  double linear_multiplier_;
  double angular_multiplier_;
};

} // end namespace logjoy

#endif
