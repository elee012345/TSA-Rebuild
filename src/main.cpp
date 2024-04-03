#include "main.h"
#include "lemlib/api.hpp"
#include <cstddef>


pros::Controller	master(pros::E_CONTROLLER_MASTER);

pros::Motor			right_front(2, MOTOR_GEAR_BLUE, false);
pros::Motor			right_mid(3, MOTOR_GEAR_BLUE, false);
pros::Motor			right_back(4,  MOTOR_GEAR_BLUE, false);
pros::Motor			left_front(5, MOTOR_GEAR_BLUE, true);
pros::Motor			left_mid(6, MOTOR_GEAR_BLUE, true);
pros::Motor			left_back(7, MOTOR_GEAR_BLUE, true);

pros::Motor_Group	left_drive({left_front, left_mid, left_back});
pros::Motor_Group	right_drive({right_front, right_mid, right_back});

pros::Motor 		left_intake(12, MOTOR_GEAR_BLUE, true);
pros::Motor 		right_intake(13, MOTOR_GEAR_BLUE, false);
pros::Motor_Group	intake({left_intake, right_intake});

pros::ADIDigitalOut	back_right_wing('B', false);
pros::ADIDigitalOut	back_left_wing('C', false);

pros::ADIDigitalOut	front_left_wing('D', false);
pros::ADIDigitalOut	front_right_wing('E', false);

pros::Imu			imu(14);
pros::Rotation		yTracking(16, false);
pros::Rotation		xTracking(15, false);




lemlib::Drivetrain_t drivetrain {
	&left_drive, // left motor group
	&right_drive, // right motor group
	12.6, // " track width
	3.25, // using new 3.25" omnis
	360 // drivetrain rpm is 360
};

lemlib::TrackingWheel x_tracking_wheel(
	&xTracking, // encoder
	3.25,
	-6.473 // " offset from tracking center
);
// -6.5
// -6.88976378

lemlib::TrackingWheel y_tracking_wheel(
	&yTracking, // encoder
	3.25,
	-1.506 // " offset from tracking center
);
// -1.25984252

lemlib::OdomSensors_t sensors {
	&y_tracking_wheel, // vertical tracking wheel 1
	nullptr, // vertical tracking wheel 2
	&x_tracking_wheel, // horizontal tracking wheel 1
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    &imu // inertial sensor
};


// forward/backward PID (untuned)
lemlib::ChassisController_t lateralController {
    6, // kP
    1, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    10 // slew rate
};
 
// turning PID (untuned)
lemlib::ChassisController_t angularController {
    1, // kP
    0, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    1 // slew rate
};

lemlib::Chassis chassis(
	drivetrain,
	lateralController,
	angularController,
	sensors
);




/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.	
 */
void initialize() {
	// if(pros::competition::is_autonomous()) {
	// 	pros::lcd::initialize();
	// 	chassis.calibrate();
	// 	while(imu.is_calibrating()) {
	// 		pros::delay(20);
	// 	}
	// }
	chassis.calibrate();
	chassis.setPose(0, 0, 0);
	// while(imu.is_calibrating()) {
	// 	pros::delay(20);
	// } 

	left_drive.set_brake_modes(MOTOR_BRAKE_BRAKE);
	right_drive.set_brake_modes(MOTOR_BRAKE_BRAKE);
	pros::lcd::initialize(); // initialize brain screen
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

void close_side_auton() {
	
	chassis.setPose(-33, -58, 0);
	// fling triball with wing
	front_right_wing.set_value(true);
	pros::delay(200);
	front_right_wing.set_value(false);
	// grab first triball
	intake.move_voltage(12000);
	chassis.moveTo(-28, -27, 700, 360);
	chassis.turnTo(-28, -8.5, 400);
	chassis.moveTo(-28, -8.5, 800, 360);
	intake.move_voltage(0);

	// go back a little bit so we don't cross the center line
	chassis.moveTo(-28, -12, 1000);

	// turn to second triball and push over
	chassis.turnTo(0, -12, 1000);
	intake.move_voltage(-12000);
	pros::delay(300);
	front_left_wing.set_value(true);
	front_right_wing.set_value(true);
	// push
	chassis.moveTo(-6, -12, 1000);
	intake.move_voltage(0);
	front_left_wing.set_value(false);
	front_right_wing.set_value(false);

	chassis.moveTo(-23, -30, 1000);
	chassis.turnTo(-53, -44, 1000, true);
	chassis.moveTo(-53, -44, 3000);
	back_left_wing.set_value(true);
	chassis.turnTo(-40, -61, 1000, true);
	chassis.moveTo(-40, -61, 1000);
	back_left_wing.set_value(false);
	chassis.turnTo(1, -58, 1000, true);
	chassis.moveTo(1, -58, 2000);
	
	
}

void ballz(){
	chassis.setPose(36.772, -59.647, 0);

	//Fling matchload
	// front_right_wing.set_value(true);
	// pros::delay(200);
	// front_right_wing.set_value(false);

	//Intake under crossbar
	chassis.turnTo(0, -67, 1000, false);
	intake.move_voltage(12000);
	chassis.moveTo(6, -59, 2000);
	pros::delay(600);
	intake.move_voltage(0);

	//Descore
	chassis.moveTo(47, -54, 2000);

	


}

void newSixBall() {
	chassis.setPose(14, -58, 270);
	intake.move_voltage(12000);
	// grab first triball
	chassis.moveTo(5, -59, 1000);
	intake.move_voltage(0);
	// go back to descore
	chassis.moveTo(48, -46, 1000, 150);
	// // go to right place to descore
	// chassis.turnTo(50, -44, 1000, true);
	// chassis.moveTo(50, -44, 1000);
	// descore
	back_left_wing.set_value(true);
	// go to right place to descore
	chassis.turnTo(53, -41, 300, true);
	chassis.moveTo(53, -41, 300);
	// fling triball to descore
	chassis.turnTo(30, -26, 300, true, 500);
	chassis.turnTo(63, -26, 400, true);
	back_left_wing.set_value(false);
	pros::delay(100);
	// push into goal
	chassis.moveTo(64, -26, 1000, 360);
	// go forward and spin
	chassis.moveTo(60, -34, 500, 360);
	chassis.turnTo(60, -23, 700);
	// score third triball
	intake.move_voltage(-12000);
	pros::delay(500);
	chassis.moveTo(60, -19, 600, 360);
	// chassis.moveTo(60, -33, 700, 360);
	// chassis.moveTo(60, -23, 1000, 360);
	intake.move_voltage(0);
	chassis.moveTo(60, -48, 600);
	// go to mid
	chassis.turnTo(5, -33, 400);
	intake.move_voltage(12000);
	chassis.moveTo(5, -33, 1000, 360);
	// outtake towards goal
	chassis.turnTo(45, 0, 700);
	intake.move_voltage(-8000);
	pros::delay(600);
	intake.move_voltage(0);
	// go to other triball
	chassis.turnTo(2, -1, 500);
	intake.move_voltage(12000);
	chassis.moveTo(2, -1, 800, 150);
	// move back a bit
	chassis.moveTo(7, -8, 600, 360);
	// turn towards goal
	chassis.turnTo(14, 17, 100, false, 200);
	chassis.turnTo(42, -8, 500, false, 200);
	// push into goal
	intake.move_voltage(-12000);
	front_left_wing.set_value(true);
	front_right_wing.set_value(true);
	chassis.moveTo(42, -8, 900, 360);
	intake.move_voltage(0);
	front_left_wing.set_value(false);
	front_right_wing.set_value(false);
	chassis.turnTo(15, -15, 700, true, 200);
	chassis.moveTo(15, -15, 3000, 360);
	





}

void testing() {
	
	chassis.setPose(0, 0, 0);
	chassis.moveTo(0, 24, 10000, 360);
	//chassis.turnTo(30, 30, 10000);
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
	newSixBall();
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
	double drive, turn;
	bool backWingToggle = false;
	bool frontWingToggle = false;
	
	
	while(true) {
		drive = master.get_analog(ANALOG_LEFT_Y) / 127.0;
		turn = master.get_analog(ANALOG_RIGHT_X) / 127.0;
		if ( drive < 0.5 ) {
			turn /= 1.4;
		} else {
			turn /= 1.2;
		}
		



		left_drive.move_voltage((drive + turn)*12000);
		right_drive.move_voltage((drive - turn)*12000);

		// set button bindings and velocity of the intake
		if(master.get_digital(DIGITAL_R2)) {
			// intake
			intake.move_voltage(12000);
		} else if (master.get_digital(DIGITAL_R1)) {
			// outtake
			intake.move_voltage(-12000);
		} else {
			intake.move_velocity(0);
		}

		// // toggles pneumatics for back wings
		// // wont work if its anything else idk why
		// // tried wingToggle != wingToggle and ^=, neither worked so we do this
		// if(master.get_digital_new_press(DIGITAL_L1)) {
		// 	if(backWingToggle == false) {
		// 		back_left_wing.set_value(true);
		// 		back_right_wing.set_value(true);

		// 		backWingToggle = true;
		// 	} else {
		// 		back_left_wing.set_value(false);
		// 		back_right_wing.set_value(false);
		// 		backWingToggle = false;
		// 	}
		// }

		// // toggles pneumatics for front wings
		// // wont work if its anything else idk why
		// // tried wingToggle != wingToggle and ^=, neither worked so we do this
		// if(master.get_digital_new_press(DIGITAL_L2)) {
		// 	if(frontWingToggle == false) {
		// 		front_right_wing.set_value(true);
		// 		front_left_wing.set_value(true);
		// 		frontWingToggle = true;
		// 	} else {
		// 		front_right_wing.set_value(false);
		// 		front_left_wing.set_value(false);
		// 		frontWingToggle = false;
		// 	}
		// }

		// toggles pneumatics for back wings
		// wont work if its anything else idk why
		// tried wingToggle != wingToggle and ^=, neither worked so we do this
		if( !master.get_digital(DIGITAL_L2) && master.get_digital(DIGITAL_L1) ) {
			back_left_wing.set_value(true);
			back_right_wing.set_value(true);
		} else {
			back_left_wing.set_value(false);
			back_right_wing.set_value(false);
		}
		

		// toggles pneumatics for front wings
		// wont work if its anything else idk why
		// tried wingToggle != wingToggle and ^=, neither worked so we do this
		if( !master.get_digital(DIGITAL_L1) && master.get_digital(DIGITAL_L2) ) {
			front_right_wing.set_value(true);
			front_left_wing.set_value(true);
		} else {
			front_right_wing.set_value(false);
			front_left_wing.set_value(false);
		}

		
        //pose = chassis.getPose(); // get the current position of the robot
        // master::print(0, "x: %f", pose.x); // print the x position
        // pros::lcd::print(1, "y: %f", pose.y); // print the y position
        // pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
		master.print(0, 1, "x: %f", chassis.getPose().x);
		master.print(1, 1, "y: %f", chassis.getPose().y);
		master.print(2, 1, "heading: %f", chassis.getPose().theta);
		master.clear();
		
	
		
		
	}
}
