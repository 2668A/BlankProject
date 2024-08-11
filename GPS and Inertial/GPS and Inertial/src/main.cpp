#include "main.h"
#include <string>
#include <cmath>
#include <vector>


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
  
  GPS.set_offset (-0.0899, 0.1778 );//units are meters
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

// Function to drive to a specific point
void drive_to_point(double target_x, double target_y, double current_x, double current_y, double current_theta, bool end = false) {
  // Calculate the distance and angle to the target point
  double delta_x = std::abs(target_x - current_x);
  double delta_y = std::abs(target_y - current_y);
  double distance = sqrt(delta_x * delta_x + delta_y * delta_y); // Distance in inches
  double target_angle = atan2(delta_y, delta_x) * 180 / M_PI; // Angle in degrees

  // Rotate to face the target point
  double angle_diff = target_angle - current_theta;

  // // Normalize the angle difference to be within -180 to 180 degrees
  // while (angle_diff > 180) angle_diff -= 360;
  // while (angle_diff < -180) angle_diff += 360;

  if (end) {
    angle_diff = std::abs(current_theta - 180);
    chassis.pid_turn_set(angle_diff, 50);
    chassis.pid_wait();
  } else {
    angle_diff = target_angle - current_theta;
    chassis.pid_turn_relative_set(angle_diff, 70);
    chassis.pid_wait();
  };

  // Drive forward to the target point
  chassis.pid_drive_set(distance - 10, 70);
  chassis.pid_wait();
}

void opcontrol() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold. This helps autonomous consistency

  // Get current positions of the chassis
  double x = GPS.get_position_x() * 39.3700787402; // Convert meters to inches
  pros::delay(2000);
  double y = GPS.get_position_y() * 39.3700787402; // Convert meters to inches
  pros::delay(2000);
  double theta = std::abs(GPS.get_heading() - 180); // Get the heading of the chassis (degrees)
  pros::delay(2000);

  // // Determine the nearest corner
  // double corner_x = (x < 72) ? 0 : 144;
  // double corner_y = (y < 72) ? 0 : 144;

  // // Drive to the nearest corner
  // drive_to_point(corner_x, corner_y, x, y, theta);
  // pros::delay(2000);

  // Drive to the nearest corner
  chassis.pid_turn_set(0, 70);
  chassis.pid_wait();

  chassis.pid_drive_set(40, 70);
  chassis.pid_wait();

  chassis.pid_turn_set(315, 80);
  chassis.pid_wait();




  // Update current positions
  x = GPS.get_position_x() * 39.3700787402;
  pros::delay(2000);
  y = GPS.get_position_y() * 39.3700787402;
  pros::delay(2000);
  theta = std::abs(GPS.get_heading() - 180);
  pros::delay(2000);

  // Drive to (0,0)
  drive_to_point(0, 0, x, y, theta, true);
}


