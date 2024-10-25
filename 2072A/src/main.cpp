#include "main.h"
#include <string>

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
void red_pos();
void red_neg();
void red_AWP();
void blue_pos();
void blue_neg();
void blue_AWP();
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
void blue_pos_AWP();
void red_pos_AWP();
// Robodash selector initialization...
Selector selector({{"Red Positive", &red_pos},
				   {"Red Negtative", &red_neg},
				   {"Red AWP", &red_AWP},
					{"Red positive AWP", &red_pos_AWP},
				   {"Blue Positive", &blue_pos},
				   {"Blue Negative", &blue_neg},
				   {"Blue AWP", &blue_AWP},
				   	{"Blue positive AWP", &blue_pos_AWP},
				   {"Skills", &skills}});
bool racism_against_blue = false;
// Robodash console initialization...
Console console;

// Creates the custom pages
rd_view_t *view = rd_view_create("Info");
rd_view_t *auto_override = rd_view_create("Override");

ASSET(RedAWPRush_1_txt);
ASSET(RedAWPRush_2_txt);
ASSET(RedAWPRush_3_txt);
ASSET(RedFullAWP_1_txt);
ASSET(RedFullAWP_2_txt);

ASSET(BlueAWPRush_1_txt);
ASSET(BlueAWPRush_2_txt);
ASSET(BlueAWPRush_3_txt);
ASSET(BlueFullAWP_1_txt);
ASSET(BlueFullAWP_2_txt);

ASSET(SkillsRunAuto_1_txt);
ASSET(SkillsRunAuto_1_5_txt);
ASSET(SkillsRunAuto_2_txt);
ASSET(SkillsRunAuto_3_txt);
ASSET(SkillsRunAuto_4_txt);
ASSET(SkillsRunAuto_5_txt);
ASSET(SkillsRunAuto_6_txt);
ASSET(SkillsRunAuto_7_txt);
ASSET(SkillsRunAuto_8_txt);
ASSET(SkillsRunAuto_9_txt);
ASSET(SkillsRunAuto_10_txt);

// left motor group
MotorGroup left_motor_group({-16, -12, 11});
// right motor group
MotorGroup right_motor_group({-13, 14, 17});
// Intake
Motor hooks(-4);
Motor front_floating(20);
void intake_move(int speed){
	hooks.move(speed);
	front_floating.move(speed);
}
// Pneumatics
adi::Pneumatics mogo('h', true);
adi::Pneumatics floating('e', false);
adi::Pneumatics lift('d', false);
adi::Pneumatics arm('g', false);
adi::Pneumatics descore_arm('f', false);

Optical color_sensor(3);

Drivetrain drivetrain(&left_motor_group,  // left motor group
					  &right_motor_group, // right motor group
					  13,				  // TRACK WIDTH
					  Omniwheel::NEW_275, // using new 3.25" omnis
					  480,				  // 480 rpm
					  2					  // horizontal drift is 2 (for now)
);
Imu imu(2);					  // IMU on port 10
Rotation horizontal_encoder(19);  // tracking wheel on port 11
Rotation vertical_encoder(18); // tracking wheel on port 12

TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, Omniwheel::NEW_2, 4.5);
TrackingWheel vertical_tracking_wheel(&vertical_encoder, Omniwheel::NEW_2, -0.5);

OdomSensors sensors(&vertical_tracking_wheel,					 // vertical tracking wheel 1, set to null
					nullptr,					 // vertical tracking wheel 2, set to nullptr as we are using IMEs
					&horizontal_tracking_wheel,	 // horizontal tracking wheel 1
					nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
					&imu						 // inertial sensor
);

// lateral (driving) PID controller
lemlib::ControllerSettings lateral_controller(7,  // proportional gain (kP)
											  0,   // integral gain (kI)
											  0.5,   // derivative gain (kD)
											  3,   // anti windup
											  0.5,   // small error range, in inches
											  100, // small error range timeout, in milliseconds
											  1.5,   // large error range, in inches
											  500, // large error range timeout, in milliseconds
											  20   // maximum acceleration (slew)
);

// angular (turning) PID controller
ControllerSettings angular_controller(3,   // proportional gain (kP)
									  0, // integral gain (kI)
									  1,   // derivative gain (kD)
									  3,   // anti windup
									  0.5, // small error range, in degrees
									  100, // small error range timeout, in milliseconds
									  2,   // large error range, in degrees
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
	master.clear();
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
			chassis.setPose(0, 0, 0, false);
		} }, LV_EVENT_ALL, NULL);

	lv_obj_t *imu_heading_label = lv_label_create(rd_view_obj(view));
	lv_obj_align(imu_heading_label, LV_ALIGN_TOP_LEFT, 5, 5);

	lv_obj_t *drive_temp_label = lv_label_create(rd_view_obj(view));
	lv_obj_align(drive_temp_label, LV_ALIGN_TOP_LEFT, 5, 25);

	lv_obj_t *intake_temp_label = lv_label_create(rd_view_obj(view));
	lv_obj_align(intake_temp_label, LV_ALIGN_TOP_LEFT, 5, 40);

	lv_obj_t *lem_label = lv_label_create(rd_view_obj(view));
	lv_obj_align(lem_label, LV_ALIGN_TOP_LEFT, 5, 65);

	lv_obj_t *horizontal_one_label = lv_label_create(rd_view_obj(view));
	lv_obj_align(horizontal_one_label, LV_ALIGN_TOP_LEFT, 5, 175);

	// Update Loop
	while (true)
	{
		// Updates the labels to show the most recent data.
		lv_label_set_text(imu_heading_label, ("IMU Heading: " + to_string(round(imu.get_heading()))).c_str());
		lv_label_set_text(drive_temp_label, ("Average drive temp: " + to_string((left_motor_group.get_temperature() + right_motor_group.get_temperature()) / 2)).c_str());

		lv_label_set_text(lem_label, ("X: " + to_string(chassis.getPose().x) + "\nY: " + to_string(chassis.getPose().y) + "\nTheta: " + to_string(chassis.getPose().theta)).c_str());
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
//#define waitUntil(CONDITION) while (!(CONDITION))

int color_sensor_task()
{
	bool lastReverse = false;
	while(true){
		if (
			(racism_against_blue && (color_sensor.get_hue() >= 180 && color_sensor.get_hue() <= 290)) ||
			(!racism_against_blue && (color_sensor.get_hue() >= 0 && color_sensor.get_hue() <= 20)))
		{

		delay(200);
		intake_move(-50);
		delay(300);
		intake_move(-127);
		}
	}
}
/**
 * Initializes the robot by starting the update_ui task and checking if the devices are plugged in.
 *
 * @throws None
 */
void initialize()
{
	racism_against_blue = true;
	master.clear();
	mogo.extend();
	chassis.calibrate();

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
void competition_initialize() {
	racism_against_blue = true;
	master.clear();
	mogo.extend();
	chassis.calibrate();

	// Start the update_ui task
	Task t(update_ui);
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
void autonomous()
{
	console.println("Running auton...");
	chassis.setPose(0, 0, 0,false); // Resets the position before running the auton
	//Task t(color_sensor_task);
	racism_against_blue = true;
	//chassis.calibrate();
	delay(500);
	selector.run_auton();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void red_pos()
{
	racism_against_blue = true;
	chassis.setPose(-52, -24, 270); // TODO: Zach change to the start of this path...
	console.println("Red Positive is running...");

	chassis.moveToPoint(-24, -24, 1000, {.forwards = false}, false);
	mogo.extend();
	floating.extend();
	intake_move(-110);
	chassis.moveToPose(-46, -6, 0, 1000, {.lead = 0.6}, false); // default is 0.6
	floating.retract();
	chassis.moveToPoint(-30, -24, 1000, {.minSpeed = 70}, false);
	chassis.moveToPose(-24, -42, 180, 1000, {.lead = 0.6}, false);
	chassis.moveToPose(-58, -58, 225, 1000, {.lead = 0.8}, false);
	int i = 0;
	while (i < 2)
	{
		chassis.moveToPoint(-55, -55, 200, {.forwards = false}, false);
		chassis.moveToPoint(-55, -55, 200, {.forwards = false}, false);
	}
}
void red_neg()
{
	racism_against_blue = false;
	chassis.setPose(-54, 36, 270); // TODO: Zach change to the start of this path...
	console.println("Blue Negative is running...");
	chassis.moveToPoint(-24, 24, 2000, {.forwards = false, .maxSpeed = 90}, false);
	chassis.waitUntil(32);
	mogo.retract();
	delay(200);
	chassis.moveToPose(-7, 40, 0, 2000, {.forwards = true}, false);
	intake_move(-127);
	chassis.moveToPoint(-7, 52, 2000, {.forwards = true, .maxSpeed = 70}, false);
	chassis.moveToPoint(-24, 48, 2000, {.forwards = true}, false);

	chassis.moveToPoint(-24, 0, 2000, {.forwards = true}, true);
	for (int i = 0; i < 500; i++) {
		if (color_sensor.get_hue() >= 0 && color_sensor.get_hue() <= 20)
		intake_move(40);
		delay(10);
	}
}
void red_AWP()
{
	racism_against_blue = true;
	chassis.setPose(-52, 24, 270); // TODO: Zach change to the start of this path...
	console.println("Red AWP is running...");

	chassis.moveToPoint(-24, -24, 1000, {.forwards = false}, false);
	mogo.extend();
	floating.extend();
	intake_move(-110);
	chassis.moveToPose(-46, -6, 0, 1000, {.lead = 0.6}, false); // default is 0.6
	floating.retract();
}
void red_pos_AWP()
{
	racism_against_blue = false;
	console.println("Blue Positive AWP is running...");
	chassis.setPose(50, 63, 90, false);
	
	chassis.moveToPoint(27, 63, 1500, {.forwards = false}, false);
	chassis.moveToPoint(5, 50, 1500, {.forwards = false, .maxSpeed = 80}, true);
	chassis.waitUntil(24);
	mogo.retract();
	delay(200);
	chassis.moveToPoint(24, 48, 1500, {.forwards = true}, true);
	delay(500);
	intake_move(-127);
	chassis.waitUntilDone();
	//chassis.moveToPoint(30, 50, 1500, {.forwards = false}, false);
	delay(1000);
	intake_move(0);
	mogo.extend();
	//chassis.moveToPoint(24, 45, 1500, {.forwards = true}, false);
	chassis.turnToPoint(24, 24, 1500, {.forwards = false}, false);
	chassis.moveToPoint(24, 24, 1500, {.forwards = false}, false);
	chassis.waitUntil(20);
	mogo.retract();
	delay(200);
	intake_move(-127);
	chassis.moveToPoint(36, 12, 1500, {.forwards = true}, false);
	chassis.turnToHeading(135, 500, {.maxSpeed = 100}, 500);
	lift.extend();
	floating.extend();
	delay(200);
	chassis.moveToPoint(48, 4, 1500, {.forwards = true, .maxSpeed = 40}, false);
	floating.retract();
	chassis.moveToPoint(40, 8, 1500, {.forwards = false, .maxSpeed = 40}, true);
	while (chassis.isInMotion()) {
		if (color_sensor.get_hue() >= 0 && color_sensor.get_hue() <= 20)
		intake_move(0);
	}
	chassis.moveToPoint(12, 12, 1500, {.forwards = true}, true);
	while (chassis.isInMotion()) {
		if (color_sensor.get_hue() >= 0 && color_sensor.get_hue() <= 20)
		intake_move(0);
	}
	lift.retract();


}
void red_neg_AWP()
{
	racism_against_blue = true;
	chassis.setPose(-173.618, -24, 180); // TODO: Zach change to the start of this path...
	console.println("Red Negative AWP is running...");

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void blue_pos()
{
	racism_against_blue = false;
	chassis.setPose(54, 36, 90); // TODO: Zach change to the start of this path...
	console.println("Blue Positive is running...");

}
void blue_neg()
{
	racism_against_blue = false;
	chassis.setPose(54, 36, 90); // TODO: Zach change to the start of this path...
	console.println("Blue Negative is running...");
	chassis.moveToPoint(24, 24, 2000, {.forwards = false, .maxSpeed = 90}, false);
	chassis.waitUntil(32);
	mogo.retract();
	delay(200);
	chassis.moveToPose(7, 40, 0, 2000, {.forwards = true}, false);
	intake_move(-127);
	chassis.moveToPoint(7, 52, 2000, {.forwards = true, .maxSpeed = 70}, false);
	chassis.moveToPoint(24, 48, 2000, {.forwards = true}, false);



	
	chassis.moveToPoint(24, 0, 2000, {.forwards = true}, true);
	for (int i = 0; i < 500; i++) {
		if (color_sensor.get_hue() >= 0 && color_sensor.get_hue() <= 20)
		intake_move(40);
		delay(10);
	}


}
void blue_AWP()
{
	racism_against_blue = false;
	chassis.setPose(52, 24, 90); // TODO: Zach change to the start of this path...
	console.println("Blue AWP is running...");
}
void blue_pos_AWP()
{
	racism_against_blue = false;
	console.println("Blue Positive AWP is running...");
	chassis.setPose(50, -63, 90, false);
	
	chassis.moveToPoint(27, -63, 1500, {.forwards = false}, false);
	chassis.moveToPoint(5, -50, 1500, {.forwards = false, .maxSpeed = 80}, true);
	chassis.waitUntil(24);
	mogo.retract();
	delay(200);
	chassis.moveToPoint(24, -48, 1500, {.forwards = true}, true);
	delay(500);
	intake_move(-110);
	chassis.waitUntilDone();
	//chassis.moveToPoint(30, 50, 1500, {.forwards = false}, false);
	delay(1000);
	intake_move(0);
	mogo.extend();
	//chassis.moveToPoint(24, 45, 1500, {.forwards = true}, false);
	chassis.turnToPoint(24, -24, 1500, {.forwards = false}, false);
	chassis.moveToPoint(24, -24, 1500, {.forwards = false}, false);
	chassis.waitUntil(20);
	mogo.retract();
	delay(200);
	intake_move(-110);
	chassis.moveToPoint(36, -12, 1500, {.forwards = true}, false);
	chassis.turnToHeading(45, 500, {.maxSpeed = 100}, 500);
	lift.extend();
	floating.extend();
	delay(200);
	chassis.moveToPoint(48, -4, 1500, {.forwards = true, .maxSpeed = 40}, false);
	floating.retract();
	chassis.moveToPoint(40, -8, 1500, {.forwards = false, .maxSpeed = 40}, true);
	while (chassis.isInMotion()) {
		if (color_sensor.get_hue() >= 0 && color_sensor.get_hue() <= 20)
		intake_move(0);
	}
	chassis.moveToPoint(12, -12, 1500, {.forwards = true}, true);
	while (chassis.isInMotion()) {
		if (color_sensor.get_hue() >= 0 && color_sensor.get_hue() <= 20)
		intake_move(0);
	}
	intake_move(0);
	lift.retract();
}
void blue_neg_AWP()
{
	racism_against_blue = false;
	console.println("Blue Negative AWP is running...");
	chassis.setPose(63, 10, 0, false);
	
	chassis.moveToPoint(63, -10, 200, {.forwards = false, .maxSpeed = 110, .minSpeed=60}, false);
	chassis.moveToPoint(47, 20, 4000, {.forwards = true, .maxSpeed = 110, .minSpeed=60, .earlyExitRange = 2}, false);
	chassis.moveToPoint(18, 25, 4000, {.forwards = false, .maxSpeed = 110, .minSpeed=60, .earlyExitRange = 2}, false);
	mogo.retract();
	delay(250);
	intake_move(-127);
	chassis.moveToPoint(20, 50, 3000, {.forwards = true, .maxSpeed = 110, .minSpeed=60}, false);
	delay(750);

	//chassis.moveToPoint(50, 60, 5000, {.forwards = true, .maxSpeed = 70}, false);
	//descore_arm.extend();
	//delay(250);
	//chassis.moveToPoint(55, 60, 2000, {.forwards = true, .maxSpeed = 50}, false);
	//chassis.turnToHeading(180, 3000, {.maxSpeed = 100}, 500);
	//descore_arm.retract();
	//chassis.turnToHeading(90, 3000, {}, 500);
	

	chassis.moveToPose(33, 11, 150, 5000, {.forwards = true, .maxSpeed = 110}, false);
	descore_arm.extend();
	delay(300);
	chassis.turnToHeading(180, 1000, {}, true);
	delay(250);
	descore_arm.retract();
	delay(950);
	chassis.moveToPoint(35, 0, 1000, {.forwards = true, .maxSpeed = 70}, false);
	intake_move(-127);
	chassis.moveToPose(34, -6, 170, 750, {.forwards = true, .maxSpeed = 110, .minSpeed=60}, false);
	chassis.moveToPose(9, -11, 300, 3000, {.forwards = true, .maxSpeed = 110, .minSpeed=60}, false);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void skills()
{
	//chassis.calibrate();
	chassis.setPose(-49, -37, 90); // TODO: Zach change to the start of this path...
	console.println("Skills is running...");
	chassis.setPose(-62, 0, 90, false);
	intake_move(-127);
	delay(500);
	intake_move(0);
	chassis.moveToPoint(-42, 0, 1500, {.forwards = true}, false);
	chassis.moveToPoint(-46, -20, 1500, {.forwards = false}, false);
	chassis.moveToPoint(-46, -26, 1500, {.forwards = false, .maxSpeed = 50}, true);
	chassis.waitUntil(5);
	mogo.retract();
	delay(200);
	intake_move(-127);
	chassis.moveToPoint(-20, -24, 1500, {.forwards = true}, false);
	chassis.moveToPoint(-24, -52, 1500, {.forwards = true}, false);
	chassis.moveToPoint(-48, -48, 1500, {.forwards = true}, false);
	chassis.moveToPoint(-56, -48, 1500, {.forwards = true}, false);
	chassis.moveToPoint(-48, -48, 1500, {.forwards = false}, false);
	chassis.moveToPoint(-48, -56, 1500, {.forwards = true}, false);
	chassis.turnToHeading(30, 1500, {.maxSpeed = 100}, 500);
	mogo.extend();
	chassis.moveToPoint(-64, -60, 1500, {.forwards = false}, false);
	intake_move(127);
	chassis.turnToHeading(45, 1500, {.maxSpeed = 100}, 500);
	intake_move(0);
	


	chassis.moveToPoint(-46, 20, 1500, {.forwards = false}, false);
	chassis.moveToPoint(-46, 26, 1500, {.forwards = false, .maxSpeed = 50}, true);
	chassis.waitUntil(5);
	mogo.retract();
	delay(200);
	intake_move(-127);
	chassis.moveToPoint(-20, 24, 1500, {.forwards = true}, false);
	chassis.moveToPoint(-24, 52, 1500, {.forwards = true}, false);
	chassis.moveToPoint(-48, 48, 1500, {.forwards = true}, false);
	chassis.moveToPoint(-56, 48, 1500, {.forwards = true}, false);
	chassis.moveToPoint(-48, 48, 1500, {.forwards = false}, false);
	chassis.moveToPoint(-48, 56, 1500, {.forwards = true}, false);
	chassis.turnToHeading(330, 1500, {.maxSpeed = 100}, 500);
	mogo.extend();
	chassis.moveToPoint(-64, 60, 1500, {.forwards = false}, false);
	intake_move(127);
	chassis.turnToHeading(315, 1500, {.maxSpeed = 100}, 500);
	intake_move(0);
	



	chassis.moveToPoint(-42, -36, 1500, {.forwards = true}, false);
	chassis.moveToPoint(-42, 24, 1500, {.forwards = false}, false);
	chassis.moveToPoint(-60, 60, 1500, {.forwards = false}, false);
	intake_move(-127);
	chassis.moveToPoint(0, 40, 1500, {.forwards = true}, false);
	chassis.moveToPoint(55, 12, 1500, {.forwards = true}, false);
	chassis.turnToHeading(270, 1000, {.maxSpeed = 100}, 500);
	chassis.moveToPoint(75, 12, 3000, {.forwards = false, .maxSpeed = 70}, false);
	chassis.setPose(63.5, 12, 270, false);

	chassis.moveToPoint(48, 18, 3000, {.forwards = true}, false);
	chassis.moveToPoint(48, 0, 3000, {.forwards = false, .maxSpeed = 70}, false);
	chassis.waitUntil(16);
	mogo.retract();
	delay(200);
	chassis.moveToPoint(24, 18, 1500, {.forwards = true}, false);
	chassis.moveToPoint(0, 0, 1500, {.forwards = true}, false);
	chassis.moveToPoint(24, -24, 1500, {.forwards = true}, false);
	chassis.moveToPoint(24, -48, 1500, {.forwards = true}, false);
	chassis.moveToPoint(48, -48, 1500, {.forwards = true}, false);
	chassis.moveToPoint(54, -48, 1500, {.forwards = true}, false);
	chassis.moveToPoint(48, -48, 1500, {.forwards = false}, false);
	chassis.moveToPoint(48, -56, 1500, {.forwards = true}, false);
	chassis.turnToHeading(30, 1500, {.maxSpeed = 100}, 500);
	mogo.extend();
	chassis.moveToPoint(60, -60, 1500, {.forwards = false}, false);
	intake_move(127);
	chassis.turnToHeading(45, 1500, {.maxSpeed = 100}, 500);
	intake_move(0);

	//chassis.moveToPoint(55, 0, 2000, {.forwards = true}, false);
	chassis.moveToPoint(60, -60, 3000, {.forwards = false}, false);
	chassis.moveToPoint(60, 60, 3000, {.forwards = false}, false);
	

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
	master.clear();
	//Task t(color_sensor_task);
	// Loop indefinitely
	while (true)
	{
		color_sensor.set_led_pwm(50);

		// Get the left and right y-axis positions on the controller
		int leftY = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
		int rightY = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y);

		// Move the robot based on the controller inputs
		chassis.tank(leftY, rightY);

		// Toggle mogo mech
		if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_R1))
			mogo.toggle();
		if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_L2))
			lift.toggle();
		if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_L1))
			floating.toggle();

		// if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_LEFT))
		//	floating.toggle();
		/*bool lastReverse = false;
		if (master.get_digital(E_CONTROLLER_DIGITAL_R2))
		{
			if (
				(racism_against_blue && (color_sensor.get_hue() >= 180 && color_sensor.get_hue() <= 290)) ||
				(!racism_against_blue && (color_sensor.get_hue() >= 0 && color_sensor.get_hue() <= 20)))
			{
				if (lastReverse)
				{
					bool i = false;
					int j = ((int)hooks.get_position()%1005)+5;
					while (!i) {
						j = ((int)hooks.get_position()%1005)+5;
						
						if ((318 < j && j < 322)||(631 < j && j < 635)||(1003 < j && j < 1007)){i = true;}
						delay(10);
						
					}
					intake_move(127);
					delay(200);
					lastReverse = false;
				}

				int i = 0;
				int j = (abs((int)hooks.get_position())%1005)+5;
				while (i < 2) {
					j = (abs((int)hooks.get_position())%1005)+5;
					if ((285 < j && j < 295)||(595 < j && j < 610)||(970 < j && j < 980)){
						i++;
						console.printf("%d", (j));
					}
					delay(5);
				}
				intake_move(127);
				delay(200);
				lastReverse = true;
			}
			else
			{
				intake_move(-127);
				lastReverse = false;
			}
			if (master.get_digital(E_CONTROLLER_DIGITAL_L1))
			{
				hooks.move(-90);
				front_floating.move(-127);

				// REVERSE INTO LIFT
				if (
					(!racism_against_blue && (color_sensor.get_hue() >= 180 && color_sensor.get_hue() <= 290)) ||
					(racism_against_blue && (color_sensor.get_hue() >= 0 && color_sensor.get_hue() <= 20)))
				{
					hooks.move(110);
					delay(500);
				}
				
			}
		}*/
		if (master.get_digital(E_CONTROLLER_DIGITAL_R2))
			intake_move(-127);
		else if (master.get_digital(E_CONTROLLER_DIGITAL_DOWN))
			intake_move(127);
		else
			intake_move(0);

		if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_Y))
		{
			descore_arm.toggle();
		}
		if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_LEFT))
		{
			blue_neg();
		}
		
		// FOR REFERENCE, LIFT 50 deg is parrellel to the ground, 90-100 is fully up.
		// if ((lift_encoder.get_position() / 100 * 0.2) > 15)
		//	master.rumble("-");

		// Delay to save resources
		delay(25);
	}
}
