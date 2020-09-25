#ifndef CAMERA_CONTROL_TWIST_JOY_CAMERA_CONTROL_TWIST_JOY_H_
#define CAMERA_CONTROL_TWIST_JOY_CAMERA_CONTROL_TWIST_JOY_H

namespace ros { class NodeHandle; }

namespace camera_control_twist_joy
{

/**
 * Class implementing a basic Joy -> Twist translation.
 */
class CameraControlTwistJoy
{
public:
  CameraControlTwistJoy(ros::NodeHandle* nh, ros::NodeHandle* nh_param);

private:
  struct Impl;
  Impl* pimpl_;
};

}  // namespace camera_control_twist_joy

#endif  // CAMERA_CONTROL_TWIST_JOY_CAMERA_CONTROL_TWIST_JOY_H
