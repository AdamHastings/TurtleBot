#include "turtlebot_commander/logitech_joy.h"

namespace logjoy {

logitechJoyController::logitechJoyController()
{
  nh_ = new ros::NodeHandle();
  nh_private_ = new ros::NodeHandle("~");

  nh_private_->param<double>("linear_multiplier", linear_multiplier_, 1.0);
  nh_private_->param<double>("angular_multiplier", angular_multiplier_, 1.0);

  memset(&vel_cmd_, 0, sizeof(vel_cmd_));

  joy_sub_ = nh_->subscribe("joy_throttled", 1, &logitechJoyController::joyCallback, this);
  cmd_pub_ = nh_->advertise<geometry_msgs::Twist>("/turtlebot_teleop_joystick/cmd_vel", 1);
}

logitechJoyController::~logitechJoyController()
{
  if (nh_) {
    nh_->shutdown();
    delete nh_;
  }
  if (nh_private_) {
    nh_private_->shutdown();
    delete nh_private_;
  }
}

void logitechJoyController::joyCallback(const sensor_msgs::Joy &msg)
{
  if (msg.buttons[0] == 1) // armed
  {
    vel_cmd_.linear.x = linear_multiplier_ * msg.axes[1];
    vel_cmd_.angular.z = angular_multiplier_ * msg.axes[2];
  }
  else
    memset(&vel_cmd_, 0, sizeof(vel_cmd_));

  cmd_pub_.publish(vel_cmd_);
}

} // end namespace logjoy

int main(int argc, char **argv)
{
  ros::init(argc, argv, "logitech_commander_node");
  logjoy::logitechJoyController ljc;

  ros::spin();
  return 0;
}
