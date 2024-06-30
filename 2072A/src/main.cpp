#include "main.h"

using namespace pros;

void red_left();
void red_right();
void skills();

rd::Selector selector({{"Red Left", &red_left},
					   {"Red Right", &red_right},
					   {"Skills", &skills}});

rd::Console console;
rd_view_t *view = rd_view_create("Info");
rd_view_t *auto_override = rd_view_create("Override");
ASSET(example_txt); // TODO: Zach, add the path name here. Add the path to static folder.

// left motor group
pros::MotorGroup left_motor_group({1, 2, 3});
// right motor group
pros::MotorGroup right_motor_group({1, 2, 3});

lemlib::Drivetrain drivetrain(&left_motor_group,		  // left motor group
							  &right_motor_group,		  // right motor group
							  10,						  // TRACK WIDTH, NEEDS MEASUREMENT LATER...
							  lemlib::Omniwheel::NEW_325, // using new 4" omnis
							  450,						  // 450 rpm
							  2							  // horizontal drift is 2 (for now)
);
Imu imu(10);					 // IMU on port 10
Rotation vertical_encoder(11);	 // tracking wheel on port 11
Rotation horizontal_encoder(12); // tracking wheel on port 12
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -5.75);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -5.75);

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

int update_ui()
{
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

	lv_obj_t *calibrate_btn = lv_btn_create(rd_view_obj(view));
	lv_obj_align(calibrate_btn, LV_ALIGN_BOTTOM_RIGHT, -20, -20);

	lv_obj_t *calibrate_label = lv_label_create(calibrate_btn);
	lv_label_set_text(calibrate_label, "Calibrate Inertial");
	lv_obj_center(calibrate_label);
	lv_obj_add_event_cb(calibrate_btn, [](lv_event_t *e) {
		if (lv_event_get_code(e) == LV_EVENT_CLICKED)
		{
			rd_view_alert(view, "Calibrating IMU...");
			imu.reset();
		} }, LV_EVENT_ALL, NULL);
	lv_obj_t *reset_pose_btn = lv_btn_create(rd_view_obj(view));
	lv_obj_align(reset_pose_btn, LV_ALIGN_TOP_RIGHT, -20, 60);

	lv_obj_t *reset_pose_label = lv_label_create(reset_pose_btn);
	lv_label_set_text(reset_pose_label, "Reset Pose");
	lv_obj_center(reset_pose_label);
	lv_obj_add_event_cb(reset_pose_btn, [](lv_event_t *e) {
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

	lv_obj_t *battery_label = lv_label_create(rd_view_obj(view));
	lv_obj_align(battery_label, LV_ALIGN_TOP_LEFT, 5, 100);
	while (true)
	{
		lv_label_set_text(imu_heading_label, ("IMU Heading: " + std::to_string(round(imu.get_heading()))).c_str());
		lv_label_set_text(drive_temp_label, ("Average drive temp: " + std::to_string((left_motor_group.get_temperature() + right_motor_group.get_temperature()) / 2)).c_str());
		lv_label_set_text(lem_label, ("X: " + std::to_string(chassis.getPose().x) + "\nY: " + std::to_string(chassis.getPose().y) + "\nTheta: " + std::to_string(chassis.getPose().theta)).c_str());
		lv_label_set_recolor(battery_label, true);
		((pros::battery::get_voltage() / 12000) * 100) > 50 ? lv_obj_set_style_text_color(battery_label, lv_color_hex(0x00FF00), NULL) : lv_obj_set_style_text_color(battery_label, lv_color_hex(0xFF0000), NULL);
		lv_label_set_text(battery_label, ("Battery: " + std::to_string((pros::battery::get_voltage() / 12000) * 100) + "%").c_str());
		pros::delay(100);
	}
}
void initialize()
{
	// pros::lcd::initialize();
	// pros::lcd::set_text(1, "Hello PROS User!");
	console.println("Initializing robot...");
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
	using namespace lemlib;
	console.println("Running auton...");
	chassis.setPose(0, 0, 0); // TODO: Set the proper starting position for each path.
	selector.run_auton();
}
void red_left()
{
	console.println("red left is running...");
}
void red_right()
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
	pros::Controller master(E_CONTROLLER_MASTER);

	while (true)
	{
		// get left y and right y positions
		int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
		// move the robot
		chassis.tank(leftY, rightY);

		// delay to save resources
		pros::delay(25);
	}
}