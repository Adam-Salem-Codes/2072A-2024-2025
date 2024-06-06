#include "main.h"
#include "lemlib/api.hpp"
#include "util.h"

using namespace pros;

ASSET(path_txt); // TODO: Zach, add the path name here. Add the path to static folder.

Motor front_left_motor(1, E_MOTOR_GEAR_BLUE, false);   // left_motor_group
Motor middle_left_motor(1, E_MOTOR_GEAR_BLUE, false);  // left_motor_group
Motor back_left_motor(1, E_MOTOR_GEAR_BLUE, false);	   // left_motor_group
Motor front_right_motor(4, E_MOTOR_GEAR_BLUE, false);  // right_motor_group
Motor middle_right_motor(5, E_MOTOR_GEAR_BLUE, false); // right_motor_group
Motor back_right_motor(6, E_MOTOR_GEAR_BLUE, false);   // right_motor_group

// left motor group
pros::MotorGroup left_motor_group({front_left_motor, middle_left_motor, back_left_motor});
// right motor group
pros::MotorGroup right_motor_group({front_right_motor, middle_right_motor, back_right_motor});

lemlib::Drivetrain drivetrain(&left_motor_group,		  // left motor group
							  &right_motor_group,		  // right motor group
							  10,						  // TRACK WIDTH, NEEDS MEASUREMENT LATER...
							  lemlib::Omniwheel::NEW_325, // using new 4" omnis
							  450,						  // 450 rpm
							  2							  // horizontal drift is 2 (for now)
);
Imu imu(10);							// IMU on port 10
Rotation vertical_tracking_wheel(11);	// tracking wheel on port 11
Rotation horizontal_tracking_wheel(12); // tracking wheel on port 12

lemlib::OdomSensors sensors(&vertical_tracking_wheel,	// vertical tracking wheel 1, set to null
							nullptr,					// vertical tracking wheel 2, set to nullptr as we are using IMEs
							&horizontal_tracking_wheel, // horizontal tracking wheel 1
							nullptr,					// horizontal tracking wheel 2, set to nullptr as we don't have a second one
							&imu						// inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10,  // proportional gain (kP)
											  0,   // integral gain (kI)
											  3,   // derivative gain (kD)
											  3,   // anti windup
											  1,   // small error range, in inches
											  100, // small error range timeout, in milliseconds
											  3,   // large error range, in inches
											  500, // large error range timeout, in milliseconds
											  20   // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2,   // proportional gain (kP)
											  0,   // integral gain (kI)
											  10,  // derivative gain (kD)
											  3,   // anti windup
											  1,   // small error range, in degrees
											  100, // small error range timeout, in milliseconds
											  3,   // large error range, in degrees
											  500, // large error range timeout, in milliseconds
											  0	   // maximum acceleration (slew)
);
lemlib::Chassis chassis(drivetrain,			// drivetrain settings
						lateral_controller, // lateral PID settings
						angular_controller, // angular PID settings
						sensors				// odometry sensors
);
void initialize()
{
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
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
void autonomous()
{
	using namespace util;
	using namespace lemlib;
	chassis.setPose(0, 0, 0); // TODO: Set the proper starting position for each path.
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
void opcontrol()
{
	while (true)
	{
		// get left y and right y positions
		int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

		// move the robot
		chassis.tank(leftY, rightY);

		// delay to save resources
		pros::delay(25);
	}
}