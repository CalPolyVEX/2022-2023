#include "main.h"

/*
const int8_t LEFT_WHEELS_PORT_1 = 2;
const int8_t LEFT_WHEELS_PORT_2 = 3;
const int8_t RIGHT_WHEELS_PORT_1 = 17;
const int8_t RIGHT_WHEELS_PORT_2 = 15;
*/

#define MOTOR_MAX_SPEED 100

const int8_t FRONT_LEFT_PORT = 5;
const int8_t FRONT_RIGHT_PORT = 2;
const int8_t BACK_LEFT_PORT = 4;
const int8_t BACK_RIGHT_PORT = 3;

const int8_t GYRO_PORT = 12;

pros::Imu gyro(GYRO_PORT);
double gyro_offset = 0;

double getRawRotation() {
	double heading = gyro.get_heading();
	return (heading == PROS_ERR_F ? 0 : heading);
}

double getRotation() {
	return getRawRotation() - gyro_offset;
}

void resetRotation() {
	gyro_offset = getRawRotation();
}
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
	resetRotation();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

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
 // 1000 units = 18 inches
 // 1000 units of turning = slightly more than 90 degrees
 // not much of a difference between move_absolute and move_relative 
void autonomous() {
	pros::Motor front_left_mtr(FRONT_LEFT_PORT);
	pros::Motor front_right_mtr(FRONT_RIGHT_PORT);
	pros::Motor back_left_mtr(BACK_LEFT_PORT);
	pros::Motor back_right_mtr(BACK_RIGHT_PORT);

	/* MOVE STRAIGHT
	front_left_mtr.move_absolute(-1000, MOTOR_MAX_SPEED);
	front_right_mtr.move_absolute(1000, MOTOR_MAX_SPEED);
	back_left_mtr.move_absolute(-1000, MOTOR_MAX_SPEED);
	back_right_mtr.move_absolute(1000, MOTOR_MAX_SPEED);
	*/

	front_left_mtr.move_absolute(1000, MOTOR_MAX_SPEED);
	front_right_mtr.move_absolute(1000, MOTOR_MAX_SPEED);
	back_left_mtr.move_absolute(1000, MOTOR_MAX_SPEED);
	back_right_mtr.move_absolute(1000, MOTOR_MAX_SPEED);
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
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	/*
	pros::Motor left_mtr1(LEFT_WHEELS_PORT_1, true);
	pros::Motor left_mtr2(LEFT_WHEELS_PORT_2, true);
	pros::Motor right_mtr1(RIGHT_WHEELS_PORT_1);
	pros::Motor right_mtr2(RIGHT_WHEELS_PORT_2);
	*/
	pros::Motor front_left_mtr(FRONT_LEFT_PORT);
	pros::Motor front_right_mtr(FRONT_RIGHT_PORT);
	pros::Motor back_left_mtr(BACK_LEFT_PORT);
	pros::Motor back_right_mtr(BACK_RIGHT_PORT);

	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);

		//field centric x-drive
		int x = master.get_analog(ANALOG_LEFT_X);
		int y = master.get_analog(ANALOG_LEFT_Y);
		int r = master.get_analog(ANALOG_RIGHT_X);
		double angle = getRotation() * M_PI / 180; // converting to radians

		int h = x * cos(angle) - y * sin(angle);
		int v = x * sin(angle) + y * cos(angle);

		front_left_mtr.move(v+h+r);
		front_right_mtr.move(-v+h+r);
		back_left_mtr.move(v-h+r);
		back_right_mtr.move(-v-h+r);

		std::string print_angle = std::to_string(angle * 180 / M_PI);
		pros::lcd::set_text(2, "Angle: " + print_angle);

		//x-drive
		/*
		intx = master.get_analog(ANALOG_LEFT_X);
		int y = master.get_analog(ANALOG_LEFT_Y);
		int r = master.get_analog(ANALOG_RIGHT_X);
		front_left_mtr.move(y+x+r);
		front_right_mtr.move(-y+x+r);
		back_left_mtr.move(y-x+r);
		back_right_mtr.move(-y-x+r);
		*/

		//tank drive
		/*
		int left = master.get_analog(ANALOG_LEFT_Y);
		int right = master.get_analog(ANALOG_RIGHT_Y);
		left_mtr1.move(left);
		left_mtr2.move(left);
		right_mtr1.move(right);
		right_mtr2.move(right);
		*/

		//arcade drive
		/*
		int power = master.get_analog(ANALOG_LEFT_Y);
		int turn = master.get_analog(ANALOG_RIGHT_X);
		int left = power + turn;
		int right = power - turn;
		left_mtr1.move(left);
		left_mtr2.move(left);
		right_mtr1.move(right);
		right_mtr2.move(right);
		*/

		//left_mtr = left;
		//right_mtr = right;
		pros::delay(2);
	}
}
