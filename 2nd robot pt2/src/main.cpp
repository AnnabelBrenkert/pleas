#include "main.h"
#include "EZ-Template/util.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

extern pros::Motor Intake1;
extern pros::Motor Intake2;
extern pros::Motor LB;
extern pros::Controller master;
extern pros::ADIDigitalOut MOGO;
extern pros::ADIDigitalOut arm;

pros::Motor Intake1(2);
pros::Motor Intake2(9);
pros::Motor LB (20);
pros::Controller master (pros::E_CONTROLLER_MASTER);
    #define MOGO_PORT 'g'
     #define ARM_PORT 'h'
pros::ADIDigitalOut MOGO(MOGO_PORT);
pros::ADIDigitalOut arm(ARM_PORT);

const int DRIVE_SPEED = 550;
const int TURN_SPEED = 300;
const int SWING_SPEED = 300;

/*Flywheel Variables*/
	bool lastKnownButtonL1State;
	int MOGOState = 0; /*0 = off, 1 = Grab*/

/*Arm Variables*/
	bool lastKnownButtonXState;
	int ARMState = 0; /*0 = off, 1 = Grab*/

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-10, -7, -8},     // Left Chassis Ports (negative port will reverse it!)
    {3, 4, 1},  // Right Chassis Ports (negative port will reverse it!)

    5,      // IMU Port
    3.25,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450);   // Wheel RPM

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);  // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0);    // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0, 0);     // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  arm.set_value(0);
MOGO.set_value(0);

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

    // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(".");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
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
  // . . .
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
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

 
//red 

chassis.pid_swing_set(ez::LEFT_SWING, -90_deg, 120, 45);
  pros::delay (700);
  
chassis.pid_drive_set(-6_in, 50);
pros::delay (1000);

Intake2.move(-127);
Intake1.move(-127);
pros::delay (400);

Intake1.move(127);
Intake2.move(127);
pros::delay (200); 

// chassis.pid_drive_set(10_in, 50);
// Intake.move(-127);
// pros::delay (1000); 

// chassis.pid_drive_set(10_in, 100);
// pros::delay (1000); 

// chassis.pid_turn_set(-223_deg, 150);
// pros::delay (500); 

 chassis.pid_drive_set(6_in, 100);
 pros::delay (1000); 

chassis.pid_turn_set(-230_deg, 150);
pros::delay (500); 

chassis.pid_drive_set(-36_in, 50);
pros::delay (1500); 

MOGO.set_value(1);
pros::delay (100); 

Intake1.move(-127);
Intake2.move(-127);
pros::delay (1000); 
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
  // This is preference to what you like to drive on
  pros::motor_brake_mode_e_t driver_preference_brake = MOTOR_BRAKE_COAST;

  chassis.drive_brake_set(driver_preference_brake);

  while (true) {
    chassis.opcontrol_tank();  // Tank control
    chassis.opcontrol_arcade_standard(ez::SPLIT);  // Standard split arcade

    /*Intake Control*/
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		{
        Intake1.move(127);
        Intake2.move(-127);
    
		}
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
       Intake1.move(-127);
       Intake2.move(-127);
		}
		else 
		{
			  Intake2.move(0);
        Intake1.move(-127);
		}

/*LB Control*/
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP))
		{
        LB.move(100);
    
		}
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
		{
       LB.move(-100);
		}
		else 
		{
			  LB.move(0);
		}

//MOGO code
   	if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) != lastKnownButtonL1State)
		{
			lastKnownButtonL1State = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
			if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && MOGOState == 0 || MOGOState == 2)
			{
				MOGOState = 1;
			} 
			else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && MOGOState == 1)
			{
				MOGOState = 0;
			}
		}  

    switch (MOGOState)
    {
      case 0:
			 MOGO.set_value(0);
      
      pros::delay(10);
      break;
			
      case 1:
        MOGO.set_value(1);
				
      pros::delay(10);
				break;
    }

 //arm code
   	if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X) != lastKnownButtonXState)
		{
			lastKnownButtonXState = master.get_digital(pros::E_CONTROLLER_DIGITAL_X);
			if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X) && ARMState == 0 || ARMState == 2)
			{
				ARMState = 1;
			} 
			else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X) && ARMState == 1)
			{
				ARMState = 0;
			}
		}  

    switch (ARMState)
    {
      case 0:
       arm.set_value(1);
      pros::delay(10);
      break;
			
      case 1:
				arm.set_value(0);
      pros::delay(10);
				break;
    }
    

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}