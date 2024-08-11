#include "main.h"
#include <string>

/* Documentation */
// https://ez-robotics.github.io/EZ-Template/

// Chassis constructor, edit accordingly
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {13, -14, -15},  // Left Chassis Ports, (use negative numbers for reversed motors!)
    {-3, 4, 5},  // Right Chassis Ports (use negative numbers for reversed motors!)

    10,         // IMU (inertial) port
    4,          // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    257         // Wheel RPM
);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Configure your chassis controls

  // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_curve_buttons_toggle(true);

  // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_drive_activebrake_set(2);

  // Defaults for curve. If using tank, only the first parameter is used.
  // (Comment this line out if you have an SD card!) 
  chassis.opcontrol_curve_default_set(0, 0);

  // Set the constants using the function defined in autons.cpp
  default_constants();

  // Autonomous Selector
  ez::as::auton_selector.autons_add(
    {
      // Add autons selections here
    }
  );

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
void disabled() 
{
  // Add your code here
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
void competition_initialize() 
{
  // Add your code here
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
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold. This helps autonomous consistency

  // Calls selected auton from autonomous selector
  ez::as::auton_selector.selected_auton_call();
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
  // This is preference to what you like to drive on
  // MOTOR_BRAKE_HOLD (recommended), MOTOR_BRAKE_BRAKE, MOTOR_BRAKE_COAST
  // pros::motor_brake_mode_e_t driver_preference_brake = MOTOR_BRAKE_HOLD;
  // chassis.drive_brake_set(driver_preference_brake);


  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold. This helps autonomous consistency
  chassis.pid_print_toggle(true);


  // Lesson 1 Distance / Sound Sensor Showcase

  // master.clear();
  // pros::lcd::initialize();
  // pros::delay(300);

  // while (true) {
  //   // std::cout << "Distance 1 (nm): " << distance_back.get() << endl; 
  //   // std::cout << "Distance 2 (nm): " << distance_front.get() << endl;  

  //   // master.print(0, 0, "Distance1 (nm): %d", distance_back.get());
  //   // pros::delay(100);
  //   // master.print(1, 0, "Distance2 (nm): %d", distance_front.get());
  //   // pros::delay(100);

  //   // dividing by 25.4 and 2.54 because distance sensor values are in millimeters and sonic sensors are in centimeters.

  //   double distanceback = distance_back.get_distance() / 25.4;
  //   std::string distancebackstring = std::to_string(distanceback);

  //   double distancefront = distance_front.get_distance() / 25.4;
  //   std::string distancefrontstring = std::to_string(distancefront);

  //   double distancesound = sound_sensor.get_value() / 25.4;
  //   std::string distancesoundstring = std::to_string(distancesound);

  //   pros::lcd::set_text(1, "Front Distance: (in) " + distancefrontstring);
  //   pros::lcd::set_text(2, "Back Distance: (in) " + distancebackstring);
  //   pros::lcd::set_text(3, "Sound Distance: (in) " + distancesoundstring);

  //   pros::delay(500);
  // }



  chassis.drive_angle_set(180);

  // chassis.pid_drive_set(-37, 70);
  // chassis.pid_wait();
  

  chassis.pid_drive_set(-20, 70);
  chassis.pid_wait();

  while ((distance_front.get() / 25.4) > 5) {
    chassis.pid_drive_set(-3, 100);
    chassis.pid_wait();
    pros::delay(300);
  }

  // if ((distance_front.get() / 25.4) < 25) {
  //   chassis.pid_turn_set(135, 70);
  //   chassis.pid_wait();
  // }



  chassis.pid_turn_set(135, 70);
  chassis.pid_wait();

  chassis.pid_drive_set(-8, 70);
  chassis.pid_wait();
  
  chassis.pid_turn_set(180, 70);
  chassis.pid_wait();

  chassis.pid_drive_set(-30, 70);
  chassis.pid_wait();

  chassis.pid_turn_set(225, 70);
  chassis.pid_wait();

  chassis.pid_drive_set(-8, 70);
  chassis.pid_wait();

  chassis.pid_turn_set(180, 70);

  while ((sound_sensor.get_value() / 25.4) > 15) {
    chassis.pid_drive_set(-3, 70);
    pros::delay(300);
  }

  chassis.pid_turn_set(270, 70);
  chassis.pid_wait();

  chassis.pid_drive_set(60, 100);
  chassis.pid_wait();

  // const double tolerance = 1; // INCHES
  // while ((std::abs(frontdistance - (distance_back.get() / 25.4))) > tolerance) {
  //   chassis.pid_turn_relative_set(10, 100);
  //   chassis.pid_wait();
  //   pros::delay(100);
  // }






  /** 
  * Driving is disabled for auton purposes.
  * If user control is necessary, uncomment the
  * while loop and comment out the auton above.
  */

  // while (true) 
  // {
  //      chassis.opcontrol_arcade_standard(ez::SPLIT);      // Standard split arcade
  //   // chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
  //   // chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
  //   // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade
  //   // chassis.opcontrol_tank();  // Tank control

  //   // This is used for timer calculations! Keep this ez::util::DELAY_TIME
  //   pros::delay(ez::util::DELAY_TIME);
  // }
}