#include "api.h"
#include "okapi/api.hpp"
#include "okapi/impl/device/motor/motorGroup.hpp"
#include <vector>
// #include "arduinoSensors.hpp"

enum DriveMode { TANK, ARCADE, XDRIVE };
enum BindMode { TOGGLE, STEP, HOLD };

class CPRobotAbstractMotor {
public:
  virtual void setSpeed(int speed) = 0;
  virtual void moveTo(int position, int speed) = 0;
};

class CPRobotMotor: public CPRobotAbstractMotor {
private:
  int port;
  pros::Motor motor;
  int dir;
public:
  CPRobotMotor(int portNum);
  int getPort();
  int getDirection();
  void setSpeed(int speed);
  void moveTo(int position, int speed);
};

class CPRobotDigitalOut: public CPRobotAbstractMotor {
private:
  int port;
  bool value;
  pros::ADIDigitalOut output;
public:
  CPRobotDigitalOut(int portNum);
  void set(bool newValue);
  void toggle();
  void setSpeed(int speed);
  void moveTo(int position, int speed);
};

class CPRobotMotorSet: public CPRobotAbstractMotor {
private:
  std::vector<CPRobotMotor *> motors;
public:
  CPRobotMotorSet(std::initializer_list<CPRobotMotor *> motors);
  std::string listMotors();
  void setSpeed(int speed);
  void moveTo(int position, int speed);
};

class CPRobotMotorList {
private:
  std::map<int, CPRobotMotor> motors;
public:
  CPRobotMotorList(std::initializer_list<int> ports);
  CPRobotMotor* get(int port);
};

class CPRobotDigitalOutList {
private:
  std::map<int, CPRobotDigitalOut> outs;
public:
  CPRobotDigitalOutList(std::initializer_list<int> ports);
  CPRobotDigitalOut* get(int port);
};

class CPRobotControllerBind {
private:
  CPRobotAbstractMotor *motors;
  pros::controller_digital_e_t buttonPrimary;
  pros::controller_digital_e_t buttonSecondary;
  std::vector<int> positions;
  int numPositions;
  int positionIndex;
  bool releasedButtonPrimary;
  bool releasedButtonSecondary;
  enum BindMode bindMode;
  int speed;
  void controlCycleToggle(bool pressedPrimary);
  void controlCycleStep(bool pressedPrimary, bool pressedSecondary);
  void controlCycleHold(bool pressedPrimary, bool pressedSecondary);
public:
  CPRobotControllerBind(CPRobotAbstractMotor *m, pros::controller_digital_e_t bp, pros::controller_digital_e_t bs, int i, std::vector<int> p, enum BindMode bm);
  void controlCycle(pros::Controller controller);
};

class CPRobotDriver {
private:
  std::vector<CPRobotAbstractMotor *> driveMotors;
  DriveMode driveMode;
  pros::Controller controller;
  std::vector<CPRobotControllerBind *> controllerBinds;
public:
  CPRobotDriver(std::vector<CPRobotAbstractMotor *> driveMotors, DriveMode mode, std::vector<CPRobotControllerBind *> cb);
  void setSpeed(int speed);
  void controlCycle();
  pros::Controller getController();
};


// class CPRobotDriverBuilder {
// public:
//   CPRobotDriver Build();
//   CPRobotDriverBuilder WithMotors(int[]);
// };
