#ifndef ROBOT_DRIVER_H
#define ROBOT_DRIVER_H

#include "api.h"
#include "okapi/api.hpp"
#include "okapi/impl/device/motor/motorGroup.hpp"
#include "arduinoSensors.hpp"
#include "robot_specifics.h"
using namespace okapi;

class RobotDriver {
public:
  // Use the Motor wrapper from okapi. The Motor from PROS has a big footgun
  // where using it in "reversed" mode globally affects all other code using
  // the motor, such as okapi, while the reversed flag in okapi only applies to
  // that motor Object.
  okapi::Motor frontLeftMotor;
  okapi::Motor frontRightMotor;
  okapi::Motor backLeftMotor;
  okapi::Motor backRightMotor;
  pros::Controller controller;

public:
  std::shared_ptr<okapi::ChassisController> chassis;

  RobotDriver(int8_t frontLeftMotorPort, int8_t frontRightMotorPort, int8_t backLeftMotorPort, int8_t backRightMotorPort);
  //Getters
  pros::Controller *getController();
  //Controller stuff
  void tankDrive();
  void arcadeDrive();
};

#endif
