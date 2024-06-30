#include "main.h"

/*
 ___     ___    ______   ___        ___
|__ \   / _ \  |____  | |__ \      /   \
   ) | | | | |     / /     ) |    /  ^  \
  / /  | | | |    / /     / /    /  /_\  \
 / /_  | |_| |   / /     / /_   /  _____  \
|____|  \___/   /_/     |____| /__/     \__\
From: 2072A / Enginotic 6 Robotics.
*/

using namespace std;	// This is definitely bad practice but it saves time.
using namespace pros;	// This is definitely bad practice but it saves time.
using namespace lemlib; // This is definitely bad practice but it saves time.
using namespace rd;		// This is definitely bad practice but it saves time.
// using the namespaces makes the code more readable for the notebook...

// Declare functions for selector...
void red_left();
void red_right();
void blue_left();
void blue_right();
void skills();


/*
    DRIVETRAIN:
        Left Motor Group: 1, 2, 3
        Right Motor Group: 4, 5, 6
    INTAKE: 7
    MOGO MECH: A
    INERTIAL SENSOR: 10
    TRACKING WHEELS:
        Vertical: 11
        Horizontal: 12
*/
Controller master(E_CONTROLLER_MASTER); // Main controller

// Robodash selector initialization...
Selector selector({{"Red Left", &red_left},
				   {"Red Right", &red_right},
				   {"Blue Left", &blue_left},
				   {"Blue Right", &blue_right},
				   {"Skills", &skills}});

// Robodash console initialization...
Console console;

// Creates the custom pages
rd_view_t *view = rd_view_create("Info");
rd_view_t *auto_override = rd_view_create("Override");

// Initializes the paths for LemLib to follow
ASSET(example_txt); // TODO: Zach, add the path name here. Add the path to static folder.

// left motor group
MotorGroup left_motor_group({1, 2, 3});
// right motor group
MotorGroup right_motor_group({4, 5, 6});
// Intake
Motor intake(7);
// Pneumatics
adi::Pneumatics mogo('a', false);

Drivetrain drivetrain(&left_motor_group,  // left motor group
					  &right_motor_group, // right motor group
					  10,				  // TRACK WIDTH, NEEDS MEASUREMENT LATER...
					  Omniwheel::NEW_325, // using new 3.25" omnis
					  450,				  // 450 rpm
					  2					  // horizontal drift is 2 (for now)
);
Imu imu(10);					 // IMU on port 10
Rotation vertical_encoder(11);	 // tracking wheel on port 11
Rotation horizontal_encoder(12); // tracking wheel on port 12
TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, Omniwheel::NEW_275, -5.75);
TrackingWheel vertical_tracking_wheel(&vertical_encoder, Omniwheel::NEW_275, -5.75);

OdomSensors sensors(&vertical_tracking_wheel,	// vertical tracking wheel 1, set to null
					nullptr,					// vertical tracking wheel 2, set to nullptr as we are using IMEs
					&horizontal_tracking_wheel, // horizontal tracking wheel 1
					nullptr,					// horizontal tracking wheel 2, set to nullptr as we don't have a second one
					&imu						// inertial sensor
);

// lateral (driving) PID controller
ControllerSettings lateral_controller(10,  // proportional gain (kP)
									  0,   // integral gain (kI)
									  3,   // derivative gain (kD)
									  3,   // anti windup
									  1,   // small error range, in inches
									  100, // small error range timeout, in milliseconds
									  3,   // large error range, in inches
									  500, // large error range timeout, in milliseconds
									  20   // maximum acceleration (slew)
);

// angular (turing) PID controller
ControllerSettings angular_controller(2,   // proportional gain (kP)
									  0,   // integral gain (kI)
									  10,  // derivative gain (kD)
									  3,   // anti windup
									  1,   // small error range, in degrees
									  100, // small error range timeout, in milliseconds
									  3,   // large error range, in degrees
									  500, // large error range timeout, in milliseconds
									  0	   // maximum acceleration (slew)
);

// Creates the chassis using LemLib, and applies the settings and sensors to it.
Chassis chassis(drivetrain,			// drivetrain settings
				lateral_controller, // lateral PID settings
				angular_controller, // angular PID settings
				sensors				// odometry sensors
);

/**
 * A function to create/update the user interface elements with buttons, labels, and event callbacks.
 *
 * @return void
 *
 * @throws None
 */
int update_ui()
{
	// Creates the UI elements
	lv_obj_t *override_btn = lv_btn_create(rd_view_obj(auto_override));
	lv_obj_center(override_btn);
	lv_obj_add_event_cb(override_btn, [](lv_event_t *e)
						{
		if (lv_event_get_code(e) == LV_EVENT_CLICKED){
			rd_view_alert(view, "Running autonomous...");
			selector.run_auton();
		} }, LV_EVENT_ALL, NULL);
	lv_obj_t *override_label = lv_label_create(override_btn);
	lv_label_set_text(override_label, "Run Selected Autonomous");
	lv_obj_center(override_label);

	lv_obj_t *calibrate_btn = lv_btn_create(rd_view_obj(view));	  // Creates Button
	lv_obj_align(calibrate_btn, LV_ALIGN_BOTTOM_RIGHT, -20, -20); // Moves Button

	lv_obj_t *calibrate_label = lv_label_create(calibrate_btn); // Creates Label
	lv_label_set_text(calibrate_label, "Calibrate Inertial");	// Sets the text
	lv_obj_center(calibrate_label);								// Centers it on the parent button
	lv_obj_add_event_cb(calibrate_btn, [](lv_event_t *e) {		// Adds event listener
		if (lv_event_get_code(e) == LV_EVENT_CLICKED)			// If the button is clicked
		{
			rd_view_alert(view, "Calibrating IMU..."); // Alert the user.
			imu.reset();							   // Calibrate IMU.
		}
	},
						LV_EVENT_ALL, NULL);

	lv_obj_t *reset_pose_btn = lv_btn_create(rd_view_obj(view));
	lv_obj_align(reset_pose_btn, LV_ALIGN_TOP_RIGHT, -20, 60);

	lv_obj_t *reset_pose_label = lv_label_create(reset_pose_btn);
	lv_label_set_text(reset_pose_label, "Reset Pose");
	lv_obj_center(reset_pose_label);
	lv_obj_add_event_cb(reset_pose_btn, [](lv_event_t *e)
						{
		if (lv_event_get_code(e) == LV_EVENT_CLICKED)
		{
			rd_view_alert(view, "Calibrating and Reseting...");
			chassis.calibrate();
			chassis.resetLocalPosition(); 
		} }, LV_EVENT_ALL, NULL);

	lv_obj_t *imu_heading_label = lv_label_create(rd_view_obj(view));
	lv_obj_align(imu_heading_label, LV_ALIGN_TOP_LEFT, 5, 5);

	lv_obj_t *drive_temp_label = lv_label_create(rd_view_obj(view));
	lv_obj_align(drive_temp_label, LV_ALIGN_TOP_LEFT, 5, 25);

	lv_obj_t *lem_label = lv_label_create(rd_view_obj(view));
	lv_obj_align(lem_label, LV_ALIGN_TOP_LEFT, 5, 45);

	lv_obj_t *vert_one_label = lv_label_create(rd_view_obj(view));
	lv_obj_align(vert_one_label, LV_ALIGN_TOP_LEFT, 5, 150);

	lv_obj_t *horizontal_one_label = lv_label_create(rd_view_obj(view));
	lv_obj_align(horizontal_one_label, LV_ALIGN_TOP_LEFT, 5, 175);
	// Creates the UI elements

	// Update Loop
	while (true)
	{
		// Updates the labels to show the most recent data.
		lv_label_set_text(imu_heading_label, ("IMU Heading: " + to_string(round(imu.get_heading()))).c_str());
		lv_label_set_text(drive_temp_label, ("Average drive temp: " + to_string((left_motor_group.get_temperature() + right_motor_group.get_temperature()) / 2)).c_str());
		lv_label_set_text(lem_label, ("X: " + to_string(chassis.getPose().x) + "\nY: " + to_string(chassis.getPose().y) + "\nTheta: " + to_string(chassis.getPose().theta)).c_str());
		lv_label_set_text(vert_one_label, ("Vertical tracking 1: " + to_string(vertical_tracking_wheel.getDistanceTraveled())).c_str());
		lv_label_set_text(horizontal_one_label, ("Horizontal tracking 1: " + to_string(horizontal_tracking_wheel.getDistanceTraveled())).c_str());
		delay(100); // Waits 100ms before updating again to make it readable.
	}
	// Update Loop
}

/**
 * Checks if a device is plugged in at the given port.
 *
 * @param port The port number of the device.
 * @param deviceName The name of the device.
 *
 * @throws None
 */
void check_device_plugged_in(int port, std::string deviceName)
{
	if (!isPluggedIn(port))
	{
		master.rumble("---");
		master.print(0, 0, "check console...");
		console.println((deviceName + " not plugged in!").c_str());
	}
}

/**
 * Initializes the robot by starting the update_ui task and checking if the devices are plugged in.
 *
 * @throws None
 */
void initialize()
{
	master.clear();
	mogo.retract(); // Make sure mogo mech is retracted on initialization.
	// Check if all devices are plugged in
	console.println("Checking device status...");
	check_device_plugged_in(imu.get_port(), "IMU");
	check_device_plugged_in(left_motor_group.get_port(0), "Left Motor Group 1");
	check_device_plugged_in(left_motor_group.get_port(1), "Left Motor Group 2");
	check_device_plugged_in(left_motor_group.get_port(2), "Left Motor Group 3");
	check_device_plugged_in(right_motor_group.get_port(0), "Right Motor Group 1");
	check_device_plugged_in(right_motor_group.get_port(1), "Right Motor Group 2");
	check_device_plugged_in(right_motor_group.get_port(2), "Right Motor Group 3");
	check_device_plugged_in(vertical_encoder.get_port(), "Vertical Encoder");
	check_device_plugged_in(horizontal_encoder.get_port(), "Horizontal Encoder");
	
	// Start the update_ui task
	Task t(update_ui);
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
	console.println("Running auton...");
	chassis.setPose(0, 0, 0); // Resets the position before running the auton
	selector.run_auton();
}

void red_left()
{
	chassis.setPose(0, 0, 0); // TODO: Zach change to the start of this path...
	console.println("red left is running...");

	// TODO: Zach, run the correct path

	// path, lookahead, timeout, forwards, async
	chassis.follow(example_txt, 9999999, 7, true, true);

	chassis.waitUntil(999); // TODO: Add the clamp and intake stuff after the waitUntils...

}
void red_right()
{
}

void blue_left()
{
}
void blue_right()
{
}
void skills()
{
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
	// Loop indefinitely
	while (true)
	{
		// Get the left and right y-axis positions on the controller
		int leftY = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
		int rightY = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y);

		// Move the robot based on the controller inputs
		chassis.tank(leftY, rightY);

		// Toggle mogo mech
		if(master.get_digital(E_CONTROLLER_DIGITAL_R1))
			mogo.toggle();
		

		// Delay to save resources
		delay(25);
	}
}
