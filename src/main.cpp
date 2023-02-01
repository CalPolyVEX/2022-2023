#include "main.h"

/*
const int8_t LEFT_WHEELS_PORT_1 = 2;
const int8_t LEFT_WHEELS_PORT_2 = 3;
const int8_t RIGHT_WHEELS_PORT_1 = 17;
const int8_t RIGHT_WHEELS_PORT_2 = 15;
*/
const int8_t FRONT_LEFT_PORT = 1;
const int8_t FRONT_RIGHT_PORT = 2;
const int8_t BACK_LEFT_PORT = 4;
const int8_t BACK_RIGHT_PORT = 3;

const int8_t GYRO_PORT = 12;

const int8_t SHOOTER_PORT1 = 14;
const int8_t SHOOTER_PORT2 = 15;

const int8_t INTAKE_PORT1 = 5;
const int8_t INTAKE_PORT2 = 7;

const char INDEXER_PORT = 'A'; // three wire

#define MOTOR_MAX_SPEED 100

//Component declaration
pros::Motor front_left_mtr(FRONT_LEFT_PORT);
pros::Motor front_right_mtr(FRONT_RIGHT_PORT);
pros::Motor back_left_mtr(BACK_LEFT_PORT);
pros::Motor back_right_mtr(BACK_RIGHT_PORT);

pros::Motor Shooter1(SHOOTER_PORT1);
pros::Motor Shooter2(SHOOTER_PORT2);
pros::Motor Intake1(INTAKE_PORT1);
pros::Motor Intake2(INTAKE_PORT2);

pros::Imu gyro(GYRO_PORT);
double gyro_offset = 0;

// pnuematics
pros::ADIDigitalOut indexer_piston(INDEXER_PORT);

//helper functions to work with field-centric x-drive
double getRawRotation() { //get data from the Vex IMU
	double heading = gyro.get_heading();
	return (heading == PROS_ERR_F ? 0 : heading);
}

double getRotation() { //subtracts an offset from the gyro value to stabilize
	return getRawRotation() - gyro_offset;
}

void resetRotation() {
	gyro_offset = getRawRotation();
}

void moveForward(int dist){
	int rot = (dist*1000)/18;

	front_left_mtr.move_relative(rot, MOTOR_MAX_SPEED);
	front_right_mtr.move_relative(-rot, MOTOR_MAX_SPEED);
	back_left_mtr.move_relative(rot, MOTOR_MAX_SPEED);
	back_right_mtr.move_relative(-rot, MOTOR_MAX_SPEED);

	pros::delay(500);
}

void moveReverse(int dist2){
	int rot = (dist2*1000)/18;

	front_left_mtr.move_relative(-rot, MOTOR_MAX_SPEED);
	front_right_mtr.move_relative(rot, MOTOR_MAX_SPEED);
	back_left_mtr.move_relative(-rot, MOTOR_MAX_SPEED);
	back_right_mtr.move_relative(rot, MOTOR_MAX_SPEED);

	pros::delay(500);
}

void moveLeft(int dist){
	int rot = (dist*1000)/18;

	front_left_mtr.move_relative(-rot, MOTOR_MAX_SPEED);
	front_right_mtr.move_relative(-rot, MOTOR_MAX_SPEED);
	back_left_mtr.move_relative(rot, MOTOR_MAX_SPEED);
	back_right_mtr.move_relative(rot, MOTOR_MAX_SPEED);

	pros::delay(500);
}

void moveRight(int dist){
	int rot = (dist*1000)/18;

	front_left_mtr.move_relative(rot, MOTOR_MAX_SPEED);
	front_right_mtr.move_relative(rot, MOTOR_MAX_SPEED);
	back_left_mtr.move_relative(-rot, MOTOR_MAX_SPEED);
	back_right_mtr.move_relative(-rot, MOTOR_MAX_SPEED);

	pros::delay(500);
}

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

void initialize() {
	pros::lcd::initialize();

	pros::lcd::register_btn1_cb(on_center_button);
	resetRotation();
}

void disabled() {}

void competition_initialize() {
	gyro.reset();
}

void autonomous() {

	/* MOVE STRAIGHT*/
	moveReverse(10);
	moveForward(15);
	moveLeft(10);
	moveRight(10);

	// front_left_mtr.move_relative(1000, MOTOR_MAX_SPEED);
	// front_right_mtr.move_relative(1000, MOTOR_MAX_SPEED);
	// back_left_mtr.move_relative(1000, MOTOR_MAX_SPEED);
	// back_right_mtr.move_relative(1000, MOTOR_MAX_SPEED);
}

void opcontrol() {
  bool indexer_piston_state = false;
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	int motorOn = 0;
	int rpm;
	while (true) {

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

		pros::lcd::set_text(3, "RPM1: " + std::to_string(18*Shooter2.get_actual_velocity())); //print rpm of shooter motors for testing
		pros::lcd::set_text(4, "RPM2: " + std::to_string(18*Shooter1.get_actual_velocity()));
		pros::lcd::set_text(5, "Current1: " + std::to_string(Shooter2.get_current_draw()));
		pros::lcd::set_text(6, "Current2: " + std::to_string(Shooter1.get_current_draw()));


		if(master.get_digital(DIGITAL_R1)) { //turn on shooter for high speed
			motorOn = 1;
			rpm = 166;
		}
		else if(master.get_digital(DIGITAL_R2)) { //turn off shooter
			motorOn = 0;
		}
		else if(master.get_digital(DIGITAL_L1)) { //turn on shooter for lower speed
			motorOn = 1;
			rpm = 140;
		}

		//set motor voltages to values to either max or zero voltage
		if (motorOn == 0) {
			Shooter1.move_voltage(0);
			Shooter2.move_voltage(0);
		}
		else if (motorOn == 1) {
 			Shooter1.move_velocity(rpm);
 			Shooter2.move_velocity(-rpm);
		}

		if(master.get_digital_new_press(DIGITAL_UP)){
			rpm = rpm + 5;
			if(rpm > 200){
				rpm = 200;
			}
			pros::lcd::set_text(1, "RPM_Get: " + std::to_string(rpm));
	 		Shooter1.move_velocity(rpm);
	 		Shooter2.move_velocity(-(rpm));
		}
		else if(master.get_digital_new_press(DIGITAL_DOWN)){
			rpm = rpm - 5;
			if(rpm < 0){
				rpm = 0;
			}
			pros::lcd::set_text(1, "RPM_Get: " + std::to_string(rpm));
	 		Shooter1.move_velocity(rpm);
	 		Shooter2.move_velocity(-(rpm));
		}

		if(master.get_digital_new_press(DIGITAL_A)) {
			gyro.reset(); //manual reset of gyro for testing
		}

		if(master.get_digital(DIGITAL_X))
		{
			Intake1.move_velocity(200);
			Intake2.move_velocity(-200);
		}
		if(master.get_digital(DIGITAL_Y))
		{
			Intake1.move_velocity(0);
			Intake2.move_velocity(0);  
		}
    
    if (master.get_digital( DIGITAL_RIGHT)) {
      indexer_piston.set_value(indexer_piston_state);
      indexer_piston_state = !indexer_piston_state;
    }


		//X-drive
		/*
		intx = master.get_analog(ANALOG_LEFT_X);
		int y = master.get_analog(ANALOG_LEFT_Y);
		int r = master.get_analog(ANALOG_RIGHT_X);
		front_left_mtr.move(y+x+r);
		front_right_mtr.move(-y+x+r);
		back_left_mtr.move(y-x+r);
		back_right_mtr.move(-y-x+r);
		*/

		pros::delay(2);
	}
}
