#include "main.h"
#include "CPSLORobotDriver.hpp"
// #include "arduinoSensors.hpp"

//We have to declare our pointers up here for any global variables
CPSLO::MotorList *motors = NULL;
// CPSLO::DigitalOutList *digitalOuts = NULL;
CPSLO::Robot *robot = NULL;
CPSLO::ControllerBind *resetRotationBind;
// CPSLO::MotorSet *left, *right, *forebar = NULL;
CPSLO::ControllerBind *intakeBind, *forebarBind, *clawAngleBind, *clawBind = NULL;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	//First we have to list out all the motors we're using. Negative port numbers denote reversed direction
	motors = new CPSLO::MotorList({4, 5, 6, 7});
	// digitalOuts = new CPSLO::DigitalOutList({1, 2});
	//Then we specify which motors should be treated as groups
	// left = new CPSLO::MotorSet({motors->get(2), motors->get(3)});
	// right = new CPSLO::MotorSet({motors->get(15), motors->get(17)});
	// forebar = new CPSLO::MotorSet({motors->get(5), motors->get(6)});
	//Then we pair any simple controls
	resetRotationBind = (new CPSLO::ControllerBindBuilder())
		->on(pros::E_CONTROLLER_DIGITAL_Y)
		->useFunction([] { robot->resetRotation(); })
		->bind();
	/*
	intakeBind = (new CPSLO::ControllerBindBuilder())
		->useMotors(motors->get(4))
		->on(pros::E_CONTROLLER_DIGITAL_R1, pros::E_CONTROLLER_DIGITAL_R2)
		->bind();
	forebarBind = (new CPSLO::ControllerBindBuilder())
		->useMotors(forebar)
		->on(pros::E_CONTROLLER_DIGITAL_L2, pros::E_CONTROLLER_DIGITAL_L1)
		->bind();
	clawAngleBind = (new CPSLO::ControllerBindBuilder())
		->useMotors(motors->get(19))
		->on(pros::E_CONTROLLER_DIGITAL_X, pros::E_CONTROLLER_DIGITAL_A)
		->withPositions(std::vector<int>{0, 900})
		->withBindMode(CPSLO::STEP)
		->bind();
	clawBind = (new CPSLO::ControllerBindBuilder())
		->useMotors(digitalOuts->get(1))
		->on(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_B)
		->withBindMode(CPSLO::TOGGLE)
		->bind();
	*/
	//(motors->get(4), pros::E_CONTROLLER_DIGITAL_R1, pros::E_CONTROLLER_DIGITAL_R2, 0, std::vector<int>{}, CPSLO::HOLD);
	// forebarBind = new CPSLO::ControllerBind(forebar, pros::E_CONTROLLER_DIGITAL_L2, pros::E_CONTROLLER_DIGITAL_L1, 0, std::vector<int>{}, CPSLO::HOLD);
	// clawAngleBind = new CPSLO::ControllerBind(motors->get(19), pros::E_CONTROLLER_DIGITAL_X, pros::E_CONTROLLER_DIGITAL_A, 0, std::vector<int>{0, 900}, CPSLO::STEP);
	// clawBind = new CPSLO::ControllerBind(digitalOuts->get(1), pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_B, 0, std::vector<int>{}, CPSLO::TOGGLE);
	//After setting up all our motors, we create vectors of drive motors and controller binds
	std::vector<CPSLO::AbstractMotor *> driveMotors({motors->get(4), motors->get(5), motors->get(6), motors->get(7)});
	// std::vector<CPSLO::ControllerBind *> binds({intakeBind, forebarBind, clawAngleBind, clawBind});
	std::vector<CPSLO::ControllerBind *> binds({resetRotationBind});
	//Finally, we initialize the robot driver instance, passing these two vectors and an enum to denote drive mode
	robot = new CPSLO::Robot(driveMotors, CPSLO::FIELD_RELATIVE_XDRIVE, binds, 10);
	//Initalize the LCD, and print
	pros::lcd::set_text(0, "Robot Initialized");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	pros::lcd::set_text(0, "Robot Disabled");
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	pros::lcd::set_text(0, "Competition Initialized");
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void autonomous() {
	pros::lcd::set_text(0, "Autonomous");
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol() {
	pros::lcd::set_text(0, "Op Control");

	while (1) {
		//Executes the drive code and any controller binds
		robot->controlCycle();
		// Required to avoid us taking up too much time from other tasks.
		pros::delay(20);
	}
}
