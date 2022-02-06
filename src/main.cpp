#include "main.h"
#define clampPiston 'B'
#define anglerPiston 'A'

std::shared_ptr<ChassisController> drive =
	ChassisControllerBuilder()
		.withMotors( {-11,-12},{1,2})
		// Green gearset, 4 in wheel diam, 11.5 im wheel track
		// 36 to 60 gear ratio
		.withDimensions({AbstractMotor::gearset::blue, (60.0/36.0)},{{3.25_in, 11_in}, imev5GreenTPR})
		.build();
		std::shared_ptr<AsyncMotionProfileController> profileController =
				  AsyncMotionProfileControllerBuilder()
				    .withLimits({
				      1.0, // Maximum linear velocity of the Chassis in m/s
				      2.0, // Maximum linear acceleration of the Chassis in m/s/s
				      10.0 // Maximum linear jerk of the Chassis in m/s/s/s
				    })
				    .withOutput(drive)
						.buildMotionProfileController();
		Controller controller;
		pros::ADIDigitalOut clamp (clampPiston);
		pros::ADIDigitalOut angler (anglerPiston);
		ControllerButton clampButton (ControllerDigital::R1);
		ControllerButton anglerButton (ControllerDigital::R2);
		ControllerButton liftUpButton (ControllerDigital::L1);
		ControllerButton liftDownButton (ControllerDigital::L2);
		ControllerButton ringIntakeButton (ControllerDigital::Y);
		ControllerButton ringNonIntakeButton (ControllerDigital::B);
		bool isClampClosed = false;
		bool isAnglerLifted =false;
		bool isRingOn = false;
		MotorGroup lift {-3,13};
		Motor ringMotor {14};
		std::shared_ptr<AsyncPositionController<double, double>> liftControl =
	AsyncPosControllerBuilder()
		.withMotor(lift)
		.build();



int height = 0;

// Functions
void checkClamp();
void ring();
void armPID();
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	/*
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
	*/
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	clamp.set_value(true);
	lift.setBrakeMode(AbstractMotor::brakeMode(2));
	/*
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
	pros::lcd::register_btn1_cb(on_center_button);
	*/
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
void autonomous() {
	std::shared_ptr<AsyncMotionProfileController> slowDrive =
  AsyncMotionProfileControllerBuilder()
    .withLimits({
      0.5, // Maximum linear velocity of the Chassis in m/s
      1.0, // Maximum linear acceleration of the Chassis in m/s/s
      10.0 // Maximum linear jerk of the Chassis in m/s/s/s
    })
    .withOutput(drive)
    .buildMotionProfileController();

	std::shared_ptr<AsyncMotionProfileController> fastDrive =
  AsyncMotionProfileControllerBuilder()
    .withLimits({
      5.0, // Maximum linear velocity of the Chassis in m/s
      6.0, // Maximum linear acceleration of the Chassis in m/s/s
      10.0 // Maximum linear jerk of the Chassis in m/s/s/s
    })
    .withOutput(drive)
    .buildMotionProfileController();

		fastDrive->generatePath(
			{{0_ft, 0_ft, 0_deg}, {5.0_ft, 0.0_ft, 0.0_deg}}, "A");

		fastDrive->generatePath(
			{{0_ft, 0_ft, 0_deg}, {2.5_ft, 0_ft, 0_deg}}, "B");

		fastDrive->generatePath(
			{{0_ft, 0_ft, 0_deg}, {2.8_ft, 0_ft, 0_deg}}, "C");

		slowDrive->generatePath(
			{{0_ft, 0_ft, 0_deg}, {1.0_ft, 0_ft, 0_deg}}, "D");

		slowDrive->generatePath(
			{{0_ft, 0_ft, 0_deg}, {1.75_ft, 0_ft, 0_deg}}, "F");

		slowDrive->generatePath(
			{{0_ft, 0_ft, 0_deg}, {1.5_ft, 0_ft, 0_deg}}, "G");

		liftControl->setTarget(-100);
		clamp.set_value(false);
		fastDrive->setTarget("A");
		fastDrive->waitUntilSettled();
		clamp.set_value(true);
		liftControl->setTarget(-700);
		fastDrive->setTarget("B", true);
		fastDrive->waitUntilSettled();
		drive->turnAngle(-90_deg);

		fastDrive->setTarget("B");
		fastDrive->waitUntilSettled();
		liftControl->setTarget(-2000);
		pros::delay(20);
		drive->turnAngle(-90_deg);
		pros::delay(500);
		slowDrive->setTarget("G");
		slowDrive->waitUntilSettled();
		clamp.set_value(false);
		drive->turnAngle(-25_deg);
		slowDrive->setTarget("D", true);
		slowDrive->waitUntilSettled();
		drive->turnAngle(-90_deg);
		liftControl->setTarget(-100);

		fastDrive->setTarget("C");
		fastDrive->waitUntilSettled();
		clamp.set_value(true);
		ringMotor.moveVelocity(200);
		slowDrive->setTarget("F", true);
		slowDrive->waitUntilSettled();
		clamp.set_value(false);
		slowDrive->setTarget("D", true);
		slowDrive->waitUntilSettled();
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
	while (true) {
		drive->getModel() -> arcade(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightX));
		pros::delay(10);
		if (clampButton.isPressed())
		{
			if(isClampClosed){
			clamp.set_value(false);
			isClampClosed=false;
			pros::delay(200);
		}else{
			clamp.set_value(true);
			isClampClosed=true;
			pros::delay(200);
		}
		}

		if (anglerButton.isPressed())
		{
			if(isAnglerLifted){
			angler.set_value(false);
			isAnglerLifted=false;
			pros::delay(200);
		}else{
			angler.set_value(true);
			isAnglerLifted=true;
			pros::delay(200);
		}
		}

		if (liftUpButton.changedToPressed())
		{
			lift.moveVelocity(-100);
			if (liftUpButton.isPressed()&&liftDownButton.isPressed())
			{
				lift.moveVoltage(-500);
			}
		}
		else if(liftUpButton.changedToReleased())
		{
			lift.moveVoltage(0);
			if (liftUpButton.isPressed()&&liftDownButton.isPressed())
			{
				lift.moveVoltage(-500);
			}
		}
		else if(liftDownButton.changedToPressed())
		{
			lift.moveVelocity(900);
			if (liftUpButton.isPressed()&&liftDownButton.isPressed())
			{
				lift.moveVoltage(-500);
			}
		}
		else if (liftDownButton.changedToReleased())
		{
			lift.moveVoltage(0);
			if (liftUpButton.isPressed()&&liftDownButton.isPressed())
			{
				lift.moveVoltage(-500);
			}
		}

		if (ringIntakeButton.isPressed())
	{
		if (isRingOn == false) {
			ringMotor.moveVelocity(300);
			isRingOn = true;
		} else {
			ringMotor.moveVelocity(0);
			isRingOn = false;
		}
		pros::delay(200);
	}

	if (ringNonIntakeButton.isPressed())
	{
		if (isRingOn == false) {
			ringMotor.moveVelocity(-300);
			isRingOn = true;
		} else {
			ringMotor.moveVelocity(0);
			isRingOn = false;
		}
		pros::delay(200);
	}

		//pros::delay(20);
	}

}


/*

old auto
autoDrive->generatePath(
	{{0_ft, 0_ft, 0_deg}, {5.2_ft, 0_ft, 0_deg}}, "F");
autoDrive->generatePath(
	{{0_ft, 0_ft, 10_deg}, {1.5_ft, 3.0_ft, 90_deg}}, "B");

liftControl->setTarget(-100);
clamp.set_value(false);
autoDrive->setTarget("F");
autoDrive->waitUntilSettled();

clamp.set_value(true);

pros::delay(20);
autoDrive->setTarget("B", true);
liftControl->setTarget(-700);
autoDrive->waitUntilSettled();
liftControl->setTarget(-1000);
pros::delay(50);
drive->turnAngle(-90_deg);
pros::delay(50);
liftControl->setTarget(-MAX_HEIGHT);
drive->turnAngle(-80_deg);
pros::delay(50);
drive->moveDistance(0.75_ft);
clamp.set_value(false);
pros::delay(50);
drive->moveDistance(-0.75_ft);

*/
