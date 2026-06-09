#include "main.h"
// Motors

// Make sure that upon building the robot, the proper ports are picked and the proper motors are reversed. -Kazi 6/8/26
pros::Motor forwardLeft(1);
pros::Motor backLeft(2);
pros::Motor forwardRight(3);
pros::Motor backRight(4);
pros::Motor middleLeft(5); //5.5w
pros::Motor middleRight(6); //5.5w

pros::Controller controllerMaster(pros::E_CONTROLLER_MASTER); // Controller
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
void autonomous() {}

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
	while (true) {
		// Drive control
		// The code below seperates the 3 controller axis' used for driving into variables that are then substituted into each motor
		// output being left stick controls vertical and horizontal and right stick controlling turning
		int32_t powerVertical = controllerMaster.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y); 
		int32_t powerHorizontal = controllerMaster.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X); // these are defined as int32_t because thats what the controller returns
		int32_t powerTurn = controllerMaster.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		forwardLeft.move( (powerVertical + powerHorizontal) + powerTurn);
		forwardRight.move((powerVertical - powerHorizontal) - powerTurn); // motor.move also expects int_32
		backLeft.move((powerVertical - powerHorizontal) + powerTurn);
		backRight.move( (powerVertical + powerHorizontal) - powerTurn);
		
		pros::delay(20);                               // Run for 20 ms then update
	}
}