#include "CPRobotDriver.hpp"
#include <vector>

/**
 *
 * CP ROBOT DRIVER
 *
 */
CPRobotDriver::CPRobotDriver(std::vector<CPRobotAbstractMotor *> driveMotors, DriveMode mode, std::vector<CPRobotControllerBind *> cb) : controller(pros::E_CONTROLLER_MASTER) {
  for (CPRobotAbstractMotor *motorSet : driveMotors) {
    this->driveMotors.push_back(motorSet);
  }
  this->driveMode = mode;
  for (CPRobotControllerBind *controllerBind : cb) {
    this->controllerBinds.push_back(controllerBind);
  }
}
void CPRobotDriver::setSpeed(int speed) {
  if (this->driveMotors[0] != NULL) {
    this->driveMotors[0]->setSpeed(speed);
  }
  if (this->driveMotors[1] != NULL) {
    this->driveMotors[1]->setSpeed(speed);
  }
}
void CPRobotDriver::controlCycle() {
  switch(this->driveMode) {
    case TANK: {
      this->driveMotors[0]->setSpeed(this->controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
      this->driveMotors[1]->setSpeed(this->controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
      break;
    }
    case ARCADE: {
      int vertical = this->controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
      int horizontal = this->controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
      this->driveMotors[0]->setSpeed((vertical + horizontal));
      this->driveMotors[1]->setSpeed((vertical - horizontal));
      break;
    }
    case XDRIVE: {
      int turn = this->controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
      int h = this->controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
      int v = this->controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
      this->driveMotors[0]->setSpeed(v + h + turn); //Front left (then clockwise, bird's eye view)
      this->driveMotors[1]->setSpeed(-v + h + turn); //Front right
      this->driveMotors[2]->setSpeed(-v - h + turn); //Back right
      this->driveMotors[3]->setSpeed(v - h + turn); //Back left
      break;
    }
  }
  for (CPRobotControllerBind *cb : this->controllerBinds) {
    cb->controlCycle(this->controller);
  }
}
pros::Controller CPRobotDriver::getController() {
  return controller;
}

/**
 *
 * CP ROBOT MOTOR
 *
 */
CPRobotMotor::CPRobotMotor(int portNum) : motor(std::abs(portNum), portNum < 0) {
  this->port = std::abs(portNum);
  this->dir = portNum < 0 ? -1 : 1;
  this->motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
}
int CPRobotMotor::getPort() { return this->port; }
int CPRobotMotor::getDirection() { return this->dir; }
void CPRobotMotor::setSpeed(int speed) {
  this->motor.move(speed);
}
void CPRobotMotor::moveTo(int position, int speed) {
  this->motor.move_absolute((double)position, speed);
}

/**
 *
 * CP ROBOT DIGITAL OUT
 *
 */
CPRobotDigitalOut::CPRobotDigitalOut(int portNum) : output(std::abs(portNum)) {
  this->port = std::abs(portNum);
  this->value = false;
}
void CPRobotDigitalOut::set(bool newValue) {
  this->value = newValue;
  this->output.set_value(this->value);
}
void CPRobotDigitalOut::toggle() {
  this->set(!this->value);
}
void CPRobotDigitalOut::setSpeed(int speed) {
  this->toggle();
}
void CPRobotDigitalOut::moveTo(int position, int speed) {
  this->toggle();
}

/**
 *
 * CP ROBOT MOTOR SET
 *
 */
CPRobotMotorSet::CPRobotMotorSet(std::initializer_list<CPRobotMotor *> newMotors) {
 for (CPRobotMotor *motor : newMotors) {
   this->motors.push_back(motor);
 }
}
std::string CPRobotMotorSet::listMotors() {
  std::string str = "";
  for (CPRobotMotor *motor : this->motors) {
    str.append(std::to_string(motor->getPort()));
    if (motor->getDirection() < 0) str.append(" (rev)");
    str.append(", ");
  }
  return str;
}
void CPRobotMotorSet::setSpeed(int speed) {
  for (CPRobotMotor *motor : this->motors) {
    motor->setSpeed(speed);
  }
}
void CPRobotMotorSet::moveTo(int position, int speed) {
  for (CPRobotMotor *motor : this->motors) {
    motor->moveTo(position, speed);
  }
}

/**
 *
 * CP ROBOT MOTOR LIST
 *
 */
CPRobotMotorList::CPRobotMotorList(std::initializer_list<int> ports) {
 for (int port : ports) {
   this->motors.insert(std::make_pair(std::abs(port), CPRobotMotor(port)));
 }
}
CPRobotMotor* CPRobotMotorList::get(int port) {
  return &(this->motors.find(std::abs(port))->second);
}

/**
 *
 * CP ROBOT DIGITAL OUT LIST
 *
 */
CPRobotDigitalOutList::CPRobotDigitalOutList(std::initializer_list<int> ports) {
 for (int port : ports) {
   this->outs.insert(std::make_pair(std::abs(port), CPRobotDigitalOut(port)));
 }
}
CPRobotDigitalOut* CPRobotDigitalOutList::get(int port) {
  return &(this->outs.find(std::abs(port))->second);
}

/**
 *
 * CP ROBOT MOTOR CONTROLLER BIND
 *
 */
CPRobotControllerBind::CPRobotControllerBind(CPRobotAbstractMotor *m, pros::controller_digital_e_t bp, pros::controller_digital_e_t bs, int i, std::vector<int> p, enum BindMode bm) {
  this->motors = m;
  this->buttonPrimary = bp;
  this->buttonSecondary = bs;
  this->positionIndex = 0;
  this->numPositions = 0;
  this->releasedButtonPrimary = true;
  this->bindMode = bm;
  this->speed = 127;
  for (int pos : p) {
    this->positions.push_back(pos);
    this->numPositions ++;
  }
  this->motors->moveTo(i, 127);
}
void CPRobotControllerBind::controlCycle(pros::Controller controller) {
  bool pressedPrimary = controller.get_digital(this->buttonPrimary);
  bool pressedSecondary = (this->buttonSecondary ? controller.get_digital(this->buttonSecondary) : false);
  switch(this->bindMode) {
    case TOGGLE:
      this->controlCycleToggle(pressedPrimary);
      break;
    case HOLD:
      this->controlCycleHold(pressedPrimary, pressedSecondary);
      break;
    case STEP:
      this->controlCycleStep(pressedPrimary, pressedSecondary);
      break;
  }
  this->releasedButtonPrimary = !pressedPrimary;
  this->releasedButtonSecondary = !pressedSecondary;
}
void CPRobotControllerBind::controlCycleStep(bool pressedPrimary, bool pressedSecondary) {
  if (pressedPrimary && this->releasedButtonPrimary) {
    this->positionIndex ++;
    if (this->positionIndex >= this->numPositions) this->positionIndex = 0;
    this->motors->moveTo(this->positions.at(this->positionIndex), this->speed);
  } else if (pressedSecondary && this->releasedButtonSecondary) {
    this->positionIndex --;
    if (this->positionIndex < 0) this->positionIndex = this->numPositions - 1;
    this->motors->moveTo(this->positions.at(this->positionIndex), this->speed);
  }
}
void CPRobotControllerBind::controlCycleHold(bool pressedPrimary, bool pressedSecondary) {
  if (pressedPrimary) {
    this->motors->setSpeed(this->speed);
  } else if (pressedSecondary) {
    this->motors->setSpeed(-this->speed);
  } else {
    this->motors->setSpeed(0);
  }
}
void CPRobotControllerBind::controlCycleToggle(bool pressedPrimary) {
  if (pressedPrimary && this->releasedButtonPrimary) {
    this->positionIndex = 1 - this->positionIndex;
    this->motors->setSpeed(this->speed * this->positionIndex);
  }
}
