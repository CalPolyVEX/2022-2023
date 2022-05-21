#include "api.h"
#include "okapi/api.hpp"
#include "okapi/impl/device/motor/motorGroup.hpp"
#include <vector>
// #include "arduinoSensors.hpp"

namespace CPSLO {

  enum DriveMode { TANK, ARCADE, XDRIVE };
  enum BindMode { TOGGLE, STEP, HOLD };

  class AbstractMotor {
  public:
    virtual void setSpeed(int speed) = 0;
    virtual void moveTo(int position, int speed) = 0;
  };

  class Motor: public AbstractMotor {
  private:
    int port;
    pros::Motor motor;
    int dir;
  public:
    Motor(int portNum);
    int getPort();
    int getDirection();
    void setSpeed(int speed);
    void moveTo(int position, int speed);
  };

  class DigitalOut: public AbstractMotor {
  private:
    int port;
    bool value;
    pros::ADIDigitalOut output;
  public:
    DigitalOut(int portNum);
    void set(bool newValue);
    void toggle();
    void setSpeed(int speed);
    void moveTo(int position, int speed);
  };

  class MotorSet: public AbstractMotor {
  private:
    std::vector<Motor *> motors;
  public:
    MotorSet(std::initializer_list<Motor *> motors);
    std::string listMotors();
    void setSpeed(int speed);
    void moveTo(int position, int speed);
  };

  class MotorList {
  private:
    std::map<int, Motor> motors;
  public:
    MotorList(std::initializer_list<int> ports);
    Motor* get(int port);
  };

  class DigitalOutList {
  private:
    std::map<int, DigitalOut> outs;
  public:
    DigitalOutList(std::initializer_list<int> ports);
    DigitalOut* get(int port);
  };

  class ControllerBind {
  private:
    AbstractMotor *motors;
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
    ControllerBind(AbstractMotor *m, pros::controller_digital_e_t bp, pros::controller_digital_e_t bs, int i, std::vector<int> p, enum BindMode bm);
    void controlCycle(pros::Controller controller);
  };

  class Robot {
  private:
    std::vector<AbstractMotor *> driveMotors;
    DriveMode driveMode;
    pros::Controller controller;
    std::vector<ControllerBind *> controllerBinds;
  public:
    Robot(std::vector<AbstractMotor *> driveMotors, DriveMode mode, std::vector<ControllerBind *> cb);
    void setSpeed(int speed);
    void controlCycle();
    pros::Controller getController();
  };


  // class DriverBuilder {
  // public:
  //   Driver Build();
  //   DriverBuilder WithMotors(int[]);
  // };
}
