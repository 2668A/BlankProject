#include "main.h"
#include <string>

/* Documentation */
// https://ez-robotics.github.io/EZ-Template/

// Chassis constructor, edit accordingly
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {13, -14, -15},  // Left Chassis Ports, (use negative numbers for reversed motors!)
    {-3, 4, 5},  // Right Chassis Ports (use negative numbers for reversed motors!)

    20,         // IMU (inertial) port
    4,          // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    343         // Wheel RPM
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


  // Lesson 1 PID Showcase




  /**
   * By default, EZ-Template comes with some PID constants should work
   * well on most machines. These constants can be adjusted in the 
   * default_constants() function in autons.cpp. There are 4 main
   * things you should change: heading PID, drive PID, turn PID, and swing PID.
   * The first value is kP, proportional gain. Next is kI, integral gain.
   * Then, kD is derivative gain. Lastly, there is a 4th argument
   * start_I, which you don't need to change. You shouldn't touch kI
   * unless nothing else works, as a tiny change can easily mess up the rest
   * of your constants. In simple terms, kP is the speed at how it goes, and kD
   * allows for more time at full speed before reaching the target by stopping more
   * abruptly (slowing down at a later point, but this can cause overshoot). 
   * 
   * You may need to adjust your PID constants if your robot is:
   *  
   *  * Trembling while driving 
   *  * Not able to drive straight lines at short distances
   *  * Consistently falling short or overshooting your target
   *  * Shaking after reaching your target (occurs most often during turning)
   *  * Too slow or starts slowing down way too quickly
   * 
   * 
   */



  /**
   * This demonstration uses two of the most basic PID functions,
   * driving, and turning. Driving uses the IMU to sense the distance
   * it has moved to determine how close it is from the target.
   * Turning uses the IMU to sense heading, the robot's angle relative
   * to the coordinates of the field. It uses your current heading / angle
   * to determine how much you need to turn to reach you target.
   * 
   * To drive, the best function to use is pid_drive_set(). It takes
   * 4 arguments. First is your target in inches.
   * This is how much you will move, negative values will drive in reverse. 
   * move. Next, is a speed from 0-127. The last two are slew and heading
   * correction, which are disabled and enabled by default, respectively. This
   * means that you do not need to explicitly pass that argument, but you will
   * if want to change those values. 
   * 
   * The turn function is very similar, but can be split into two separate functions:
   * pid_turn_set() and pid_turn_relative_set(). The non-relative function turns to an
   * absolute heading, while the relative function turns to a heading relative to your robot's
   * heading before the turn. For example, if you're facing 80 degrees, a non-relative turn
   * with an angle 15 will turn your robot 65 degrees so it faces 15 degrees on the field.
   * The relative function will turn your robot from 80 degrees to 95 degrees, since turns
   * based on your old heading. Both functions take 3 arguments. First is the target in degrees.
   * If you give the relative function a negative angle, it will turn the other way. Next is the
   * speed, from 0-127. Finally, there is slew. Just like drive, it is disabled by default. But you
   * will need to specify it in your function call if you want to use slew.
   * 
   * Lastly, there no definitions for these functions in here, as it's all compiled into
   * library files for simplicity's sake. If you would like to see the actual low-level
   * implementation, you should download the source code version of EZ-Template.
   * You can find examples and usage of the functions in the EZ-Template documentation
   * here: https://ez-robotics.github.io/EZ-Template/category/docs
   * 
   */


  chassis.pid_drive_set(158, 127);
  chassis.pid_wait();

  chassis.pid_turn_set(90, 70);
  chassis.pid_wait();
  master.print(0, 0, "IMU: %.2f", chassis.drive_imu_get());

  chassis.pid_drive_set(138, 127);
  chassis.pid_wait();

  chassis.pid_turn_set(180, 70);
  chassis.pid_wait();
  master.print(0, 0, "IMU: %.2f", chassis.drive_imu_get());

  chassis.pid_drive_set(50, 127);
  chassis.pid_wait();

  chassis.pid_turn_set(265, 70);
  chassis.pid_wait();
  master.print(0, 0, "IMU: %.2f", chassis.drive_imu_get());

  chassis.pid_drive_set(92, 127);
  chassis.pid_wait();

  chassis.pid_turn_set(177, 70);
  chassis.pid_wait();
  master.print(0, 0, "IMU: %.2f", chassis.drive_imu_get());

  chassis.pid_drive_set(85, 127);
  chassis.pid_wait();

  chassis.pid_turn_set(90, 70);
  chassis.pid_wait();
  master.print(0, 0, "IMU: %.2f", chassis.drive_imu_get());

  chassis.pid_drive_set(70, 127);
  chassis.pid_wait();

  chassis.pid_turn_set(0, 70);
  chassis.pid_wait();
  master.print(0, 0, "IMU: %.2f", chassis.drive_imu_get());

  chassis.pid_drive_set(53, 127);
  chassis.pid_wait();

  chassis.pid_turn_set(270, 70);
  chassis.pid_wait();
  master.print(0, 0, "IMU: %.2f", chassis.drive_imu_get());

  chassis.pid_drive_set(46, 127);
  chassis.pid_wait();

  chassis.pid_turn_set(320, 70);
  chassis.pid_wait();
  master.print(0, 0, "IMU: %.2f", chassis.drive_imu_get());

  chassis.pid_drive_set(43.5, 127);
  chassis.pid_wait();

  chassis.pid_turn_set(0, 70);
  chassis.pid_wait();
  master.print(0, 0, "IMU: %.2f", chassis.drive_imu_get());

  chassis.pid_drive_set(50, 127);
  chassis.pid_wait();

  chassis.pid_turn_set(90, 70);
  chassis.pid_wait();
  master.print(0, 0, "IMU: %.2f", chassis.drive_imu_get());

  chassis.pid_drive_set(-50, 70);
  chassis.pid_wait();

  chassis.pid_turn_set(0, 70);
  chassis.pid_wait();
  master.print(0, 0, "IMU: %.2f", chassis.drive_imu_get());

  chassis.pid_drive_set(-152, 70);
  chassis.pid_wait();

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