#include "CPSLORobotDriver.hpp"
#include <vector>

namespace CPSLO {
  /**
   *
   * Robot
   *
   */
  Robot::Robot(std::vector<AbstractMotor *> driveMotors, DriveMode mode, std::vector<ControllerBind *> controllerBinds, int gyroPort) : controller(pros::E_CONTROLLER_MASTER), rotation(gyroPort) {
    this->rotationOffset = 0;
    for (AbstractMotor *motorSet : driveMotors) {
      this->driveMotors.push_back(motorSet);
    }
    this->driveMode = mode;
    for (ControllerBind *controllerBind : controllerBinds) {
      this->controllerBinds.push_back(controllerBind);
    }
  }
  void Robot::setSpeed(int speed) {
    if (this->driveMotors[0] != NULL) {
      this->driveMotors[0]->setSpeed(speed);
    }
    if (this->driveMotors[1] != NULL) {
      this->driveMotors[1]->setSpeed(speed);
    }
  }
  void Robot::controlCycle() {
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
        int turn = this->controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int h = this->controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        int v = this->controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        this->driveMotors[0]->setSpeed(v + h + turn); //Front left (then clockwise, bird's eye view)
        this->driveMotors[1]->setSpeed(-v + h + turn); //Front right
        this->driveMotors[2]->setSpeed(-v - h + turn); //Back right
        this->driveMotors[3]->setSpeed(v - h + turn); //Back left
        break;
      }
      case FIELD_RELATIVE_XDRIVE: {
        //Get measurements
        int turn = this->controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int x = this->controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        int y = this->controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        float angle = this->getRotation() / 180 * M_PI;
        //Convert to field relative
        int h = x * cos(angle) - y * sin(angle);
        int v = x * sin(angle) + y * cos(angle);
        //Same old XDrive math :)
        this->driveMotors[0]->setSpeed(v + h + turn); //Front left (then clockwise, bird's eye view)
        this->driveMotors[1]->setSpeed(-v + h + turn); //Front right
        this->driveMotors[2]->setSpeed(-v - h + turn); //Back right
        this->driveMotors[3]->setSpeed(v - h + turn); //Back left
        break;
      }
    }
    for (ControllerBind *cb : this->controllerBinds) {
      cb->controlCycle(this->controller);
    }
  }
  pros::Controller Robot::getController() {
    return controller;
  }
  double Robot::getRotation() {
    return this->getRawRotation() - this->rotationOffset;
  }
  double Robot::getRawRotation() {
    double heading = this->rotation.get_heading();
    return (heading == PROS_ERR_F ? 0 : heading);
  }
  void Robot::resetRotation() {
    this->rotationOffset = this->getRawRotation();
  }

  /**
   *
   *  MOTOR
   *
   */
  Motor::Motor(int portNum) : motor(std::abs(portNum), portNum < 0) {
    this->port = std::abs(portNum);
    this->dir = portNum < 0 ? -1 : 1;
    this->motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
  }
  int Motor::getPort() { return this->port; }
  int Motor::getDirection() { return this->dir; }
  void Motor::setSpeed(int speed) {
    this->motor.move(speed);
  }
  void Motor::moveTo(int position, int speed) {
    this->motor.move_absolute((double)position, speed);
  }

  /**
   *
   *  DIGITAL OUT
   *
   */
  DigitalOut::DigitalOut(int portNum) : output(std::abs(portNum)) {
    this->port = std::abs(portNum);
    this->value = false;
  }
  void DigitalOut::set(bool newValue) {
    this->value = newValue;
    this->output.set_value(this->value);
  }
  void DigitalOut::toggle() {
    this->set(!this->value);
  }
  void DigitalOut::setSpeed(int speed) {
    this->toggle();
  }
  void DigitalOut::moveTo(int position, int speed) {
    this->toggle();
  }

  /**
   *
   *  MOTOR SET
   *
   */
  MotorSet::MotorSet(std::initializer_list<Motor *> newMotors) {
   for (Motor *motor : newMotors) {
     this->motors.push_back(motor);
   }
  }
  std::string MotorSet::listMotors() {
    std::string str = "";
    for (Motor *motor : this->motors) {
      str.append(std::to_string(motor->getPort()));
      if (motor->getDirection() < 0) str.append(" (rev)");
      str.append(", ");
    }
    return str;
  }
  void MotorSet::setSpeed(int speed) {
    for (Motor *motor : this->motors) {
      motor->setSpeed(speed);
    }
  }
  void MotorSet::moveTo(int position, int speed) {
    for (Motor *motor : this->motors) {
      motor->moveTo(position, speed);
    }
  }

  /**
   *
   *  MOTOR LIST
   *
   */
  MotorList::MotorList(std::initializer_list<int> ports) {
   for (int port : ports) {
     this->motors.insert(std::make_pair(std::abs(port), Motor(port)));
   }
  }
  Motor* MotorList::get(int port) {
    return &(this->motors.find(std::abs(port))->second);
  }

  /**
   *
   *  DIGITAL OUT LIST
   *
   */
  DigitalOutList::DigitalOutList(std::initializer_list<int> ports) {
   for (int port : ports) {
     this->outs.insert(std::make_pair(std::abs(port), DigitalOut(port)));
   }
  }
  DigitalOut* DigitalOutList::get(int port) {
    return &(this->outs.find(std::abs(port))->second);
  }

  /**
   *
   *  MOTOR CONTROLLER BIND
   *
   */
  ControllerBind::ControllerBind(AbstractMotor *m, std::function<void()> f, pros::controller_digital_e_t bp, pros::controller_digital_e_t bs, int i, std::vector<int> p, int speed, enum BindMode bm) {
    this->motors = m;
    this->func = f;
    this->buttonPrimary = bp;
    this->buttonSecondary = bs;
    this->positionIndex = 0;
    this->numPositions = 0;
    this->releasedButtonPrimary = true;
    this->bindMode = bm;
    this->speed = speed;
    for (int pos : p) {
      this->positions.push_back(pos);
      this->numPositions ++;
    }
    if (this->motors != NULL) this->motors->moveTo(i, this->speed);
  }
  void ControllerBind::controlCycle(pros::Controller controller) {
    bool pressedPrimary = controller.get_digital(this->buttonPrimary);
    bool pressedSecondary = (this->buttonSecondary ? controller.get_digital(this->buttonSecondary) : false);
    if (this->motors != NULL) {
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
    }
    if (this->func != NULL && (pressedPrimary || pressedSecondary)) {
      this->func();
    }
    this->releasedButtonPrimary = !pressedPrimary;
    this->releasedButtonSecondary = !pressedSecondary;
  }
  void ControllerBind::controlCycleStep(bool pressedPrimary, bool pressedSecondary) {
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
  void ControllerBind::controlCycleHold(bool pressedPrimary, bool pressedSecondary) {
    if (pressedPrimary) {
      this->motors->setSpeed(this->speed);
    } else if (pressedSecondary) {
      this->motors->setSpeed(-this->speed);
    } else {
      this->motors->setSpeed(0);
    }
  }
  void ControllerBind::controlCycleToggle(bool pressedPrimary) {
    if (pressedPrimary && this->releasedButtonPrimary) {
      this->positionIndex = 1 - this->positionIndex;
      this->motors->setSpeed(this->speed * this->positionIndex);
    }
  }

  /**
   *
   * CONTROLLER BIND BUILDER
   *
   */
  ControllerBindBuilder::ControllerBindBuilder() {
    this->func = NULL;
    this->motors = NULL;
    this->bindMode = HOLD;
    this->buttonPrimary = pros::E_CONTROLLER_DIGITAL_A;
    this->buttonSecondary = pros::E_CONTROLLER_DIGITAL_A;
    this->initialPos = 0;
    this->speed = 127;
  }
  ControllerBind *ControllerBindBuilder::bind() {
    return new ControllerBind(this->motors, this->func, this->buttonPrimary, this->buttonSecondary, this->initialPos, this->positions, this->speed, this->bindMode);
  }
  ControllerBindBuilder *ControllerBindBuilder::useMotors(AbstractMotor *motors) {
    this->motors = motors;
    return this;
  }
  ControllerBindBuilder *ControllerBindBuilder::useFunction(std::function<void()> func) {
    this->func = func;
    return this;
  }
  ControllerBindBuilder *ControllerBindBuilder::on(pros::controller_digital_e_t buttonPrimary) {
    this->buttonPrimary = buttonPrimary;
      this->buttonSecondary = buttonPrimary;
    return this;
  }
  ControllerBindBuilder *ControllerBindBuilder::on(pros::controller_digital_e_t buttonPrimary, pros::controller_digital_e_t buttonSecondary) {
    this->buttonPrimary = buttonPrimary;
    this->buttonSecondary = buttonSecondary;
    return this;
  }
  ControllerBindBuilder *ControllerBindBuilder::withPositions(std::vector<int> positions) {
    for (int p : positions) {
      this->positions.push_back(p);
    }
    return this;
  }
  ControllerBindBuilder *ControllerBindBuilder::withInitialPosition(int initialPos) {
    this->initialPos = initialPos;
    return this;
  }
  ControllerBindBuilder *ControllerBindBuilder::withSpeed(int speed) {
    this->speed = speed;
    return this;
  }
  ControllerBindBuilder *ControllerBindBuilder::withBindMode(enum BindMode bindMode) {
    this->bindMode = bindMode;
    return this;
  }

}
