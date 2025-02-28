#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

///
// Constants
///
void smol_constants() {
  // P, I, D, and Start I
  chassis.pid_heading_constants_set(5, 0, 20);
  chassis.pid_drive_constants_forward_set(14, -20, 100);
  chassis.pid_drive_constants_backward_set(11, -20, 100);
  chassis.pid_turn_constants_set(2.5, 0, 8, 15);
  chassis.pid_swing_constants_forward_set(5, 0, 20);
  chassis.pid_swing_constants_backward_set(4,0,20);
  chassis.pid_odom_angular_constants_set(3, 0.0, 8);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(2, 0.0, 8);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(80_ms, 3_deg, 200_ms, 7_deg, 100_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(80_ms, 1_in, 200_ms, 3_in, 100_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(250_ms, 3_deg, 500_ms, 7_deg, 750_ms, 1000_ms);
  chassis.pid_odom_drive_exit_condition_set(250_ms, 1_in, 500_ms, 3_in, 750_ms, 1000_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(15_deg, 50);
  chassis.slew_drive_constants_set(5_in, 50);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.5);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

void default_constants() {
  // P, I, D, and Start I
  chassis.pid_heading_constants_set(5, 0, 20);
  chassis.pid_drive_constants_forward_set(14, -20, 100);
  chassis.pid_drive_constants_backward_set(11, -20, 100);
  chassis.pid_turn_constants_set(2.5, 0, 8, 15);
  chassis.pid_swing_constants_forward_set(5, 0, 20);
  chassis.pid_swing_constants_backward_set(4,0,20);
  chassis.pid_odom_angular_constants_set(3, 0.0, 8);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(2, 0.0, 8);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(80_ms, 3_deg, 200_ms, 7_deg, 100_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(80_ms, 1_in, 200_ms, 3_in, 100_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(15_deg, 50);
  chassis.slew_drive_constants_set(5_in, 50);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.5);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

void long_constants() {
  // P, I, D, and Start I
  chassis.pid_heading_constants_set(5, 0, 20);
  chassis.pid_drive_constants_forward_set(9, -20, 150);
  chassis.pid_drive_constants_backward_set(7, -20, 150);
  chassis.pid_turn_constants_set(2, 0, 12, 15);
  chassis.pid_swing_constants_set(6, 0, 65);    
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(80_ms, 3_deg, 200_ms, 7_deg, 100_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(80_ms, 1_in, 200_ms, 3_in, 100_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(15_deg, 50);
  chassis.slew_drive_constants_set(5_in, 50);
  chassis.slew_swing_constants_set(5_in, 50);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  long_constants();
  chassis.pid_drive_set(48_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  master.set_text(0,0,to_string(chassis.drive_sensor_right()));
  default_constants();

  pros::delay(1000);

  chassis.pid_drive_set(-24_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  master.set_text(0,0,to_string(chassis.drive_sensor_right()));

  pros::delay(1000);

  chassis.pid_drive_set(-12_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  master.set_text(0,0,to_string(chassis.drive_sensor_right()));

  pros::delay(1000);

  chassis.pid_drive_set(-12_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  master.set_text(0,0,to_string(chassis.drive_sensor_right()));

  pros::delay(1000);
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at
  long_constants();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  master.set_text(0,0,to_string(chassis.drive_imu_get()));
  default_constants();

  pros::delay(1000);

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  master.set_text(0,0,to_string(chassis.drive_imu_get()));
  

  pros::delay(1000);

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();
  master.set_text(0,0,to_string(chassis.drive_imu_get()));

  pros::delay(1000);

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  master.set_text(0,0,to_string(chassis.drive_imu_get()));

  pros::delay(1000);
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  master.set_text(0,0,to_string(chassis.drive_sensor_right()));

  pros::delay(1000);

  chassis.pid_turn_set(45_deg, TURN_SPEED,true);
  chassis.pid_wait();
  master.set_text(0,0,to_string(chassis.drive_imu_get()));

  pros::delay(1000);

  chassis.pid_turn_set(-45_deg, TURN_SPEED,true);
  chassis.pid_wait();
  master.set_text(0,0,to_string(chassis.drive_imu_get()));

  pros::delay(1000);

  chassis.pid_turn_set(0_deg, TURN_SPEED,true);
  chassis.pid_wait();
  master.set_text(0,0,to_string(chassis.drive_imu_get()));

  pros::delay(1000); 

  chassis.pid_odom_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  master.set_text(0,0,to_string(chassis.drive_sensor_right()));

  pros::delay(1000);
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();
  master.set_text(0,0,to_string(chassis.drive_imu_get()));

  pros::delay(1000);

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
  master.set_text(0,0,to_string(chassis.drive_imu_get()));

  pros::delay(1000);

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();
  master.set_text(0,0,to_string(chassis.drive_imu_get()));

  pros::delay(1000);

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
  master.set_text(0,0,to_string(chassis.drive_imu_get()));

  pros::delay(1000);
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  master.set_text(0,0,to_string(chassis.drive_sensor_right()));

  pros::delay(1000);

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
  master.set_text(0,0,to_string(chassis.drive_sensor_right()));

  pros::delay(1000);

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
  master.set_text(0,0,to_string(chassis.drive_sensor_right()));

  pros::delay(1000);
}

///
// Odom Pure Pursuit
/// 
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  
  
  chassis.pid_odom_set({{0, -24}, rev, DRIVE_SPEED});
  pros::delay(250);
  chassis.pid_wait();
  pros::delay(1000);
  chassis.pid_odom_set({{-24, -25}, fwd, DRIVE_SPEED});
  pros::delay(250);
  chassis.pid_wait();
  pros::delay(1000);
  chassis.pid_odom_set({{2, 1}, fwd, DRIVE_SPEED});
  pros::delay(250);
  chassis.pid_wait();
  pros::delay(1000);
  chassis.pid_turn_set(0,100);
  chassis.pid_wait();
}


///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 20;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .

void color_sort_blue(){
  int currentcolor=1;
  double rawcolorval=40;
  while(true){
    int allicolor=-1;
    if(Intake2.get_target_velocity()!=0){
      rawcolorval=Intakecolor.get_hue();
      if ((rawcolorval<20)||(rawcolorval>330)){
        currentcolor=1;
      }
      else if ((rawcolorval>60)&&(rawcolorval<330)){
        currentcolor=-1;
      }
      if (Intakedist.get()<50 && currentcolor!=allicolor){
        pros::delay(50);
        Intake2.set_reversed(true);
        pros::delay(75);
        Intake2.set_reversed(false);
        currentcolor=0;
      }
    }
  }
}

void color_sort_red(){
  int currentcolor=1;
  double rawcolorval=40;
  while(true){
    int allicolor=1;
    if(Intake2.get_target_velocity()!=0){
      rawcolorval=Intakecolor.get_hue();
      if ((rawcolorval<20)||(rawcolorval>330)){
        currentcolor=1;
      }
      else if ((rawcolorval>60)&&(rawcolorval<330)){
        currentcolor=-1;
      }
      if (Intakedist.get()<50 && currentcolor!=allicolor){
        pros::delay(50);
        Intake2.set_reversed(true);
        pros::delay(75);
        Intake2.set_reversed(false);
        currentcolor=0;
      }
    }
  }
}



void goal_base(){
  //BASED ON BLUE
  chassis.odom_enable(true);
  chassis.pid_odom_set( {{{0,-30}, rev, 110}} );
  chassis.pid_wait();
  Clamp.set_value(1);
  Intake2.move_velocity(120);
  Intake1.move_velocity(-200);
  chassis.pid_odom_set( {{{-28, -30}, fwd, 110}} );
  chassis.pid_wait();
  chassis.pid_odom_set( {{{0,-30}, rev, 110}} );
  chassis.pid_wait();
  chassis.pid_turn_set(-135,100);
  chassis.pid_wait();
  chassis.pid_odom_set( {{{-12,-43.5}, fwd, 110},{{-24,-48}, fwd, 110},{{-38,-48}, fwd, 110}});
  chassis.pid_wait();
  chassis.pid_odom_set( {{{-24,-24}, rev, 110}} );
  chassis.pid_wait();
  chassis.pid_turn_set(45,100);
  chassis.pid_wait();
  Lifter.set_value(1);
  chassis.pid_odom_set( {{{28, -7.5}, fwd, 110}} );
  chassis.pid_wait();
  pros::delay(500);
  Lifter.set_value(0);
  chassis.pid_odom_set( {{{12, -7.5}, rev, 110}} );
  chassis.pid_wait();
  chassis.pid_odom_set( {{{17, -24}, fwd, 110}} );
  chassis.pid_wait();
}

void goal_base_alt(){
  //BASED ON BLUE
  chassis.odom_enable(true);
  chassis.pid_odom_set( {{{0,-30}, rev, 110}} );
  chassis.pid_wait();
  Clamp.set_value(1);
  Intake2.move_velocity(120);
  Intake1.move_velocity(-200);
  chassis.pid_turn_set(-135,100);
  chassis.pid_wait();
  chassis.pid_odom_set( {{{-12,-43.5}, fwd, 110},{{-24,-48}, fwd, 110},{{-38,-48}, fwd, 110}});
  chassis.pid_wait();
  chassis.pid_turn_set(0,100);
  chassis.pid_odom_set( {{{-28, -30}, fwd, 110}} );
  chassis.pid_wait();
  chassis.pid_turn_set(90,100);
  chassis.pid_odom_set( {{{-24,-24}, fwd, 110}} );
  chassis.pid_wait();
  chassis.pid_turn_set(45,100);
  chassis.pid_wait();
  Lifter.set_value(1);
  chassis.pid_odom_set( {{{28, -7.5}, fwd, 110}} );
  chassis.pid_wait();
  pros::delay(500);
  Lifter.set_value(0);
  chassis.pid_odom_set( {{{12, -7.5}, rev, 110}} );
  chassis.pid_wait();
  chassis.pid_odom_set( {{{17, -24}, fwd, 110}} );
  chassis.pid_wait();
}

void stake_base(){
  //based on blue
  chassis.pid_swing_set(ez::RIGHT_SWING,-55,100);
  chassis.pid_wait();
  Arm.move_velocity(-150); 
  pros::delay(750);
  Arm.move_velocity(0); 
  pros::delay(500);
  chassis.pid_odom_set( {{{26+7.5,-9.75}, rev, 110}} );
  chassis.pid_wait();
  Arm.move_absolute(75,150);
  Clamp.set_value(1);
  Intake2.move_velocity(120);
  Intake1.move_velocity(-200);
  chassis.pid_odom_set( {{{24+7.5,-26-9.75}, fwd, 110}} );
  chassis.pid_wait();
  chassis.pid_odom_set( {{{26+7.5,-9.75}, rev, 110}} );
  chassis.pid_wait();
  chassis.pid_odom_set( {{{36+7.5,-12-9.75}, fwd, 110},{{48,-24-9.75}, fwd, 110},{{48,-42-9.75}, fwd, 110}});
  chassis.pid_wait();
  chassis.pid_odom_set( {{{26+7.5,-9.75}, rev, 110}} );
  chassis.pid_wait();
  chassis.pid_turn_set(-90, 100);
  chassis.pid_wait();
  Lifter.set_value(1);
  chassis.pid_odom_set( {{{7.5,24-9.75}, fwd, 110}} );
  chassis.pid_wait();
  Lifter.set_value(0);
  chassis.pid_odom_set( {{{12+7.5,0}, rev, 110}} );
  chassis.pid_wait();
  chassis.pid_turn_set(80, 100);
  Arm.move_velocity(-100);
  pros::delay(750);
  Arm.move_velocity(0);
}

void rush_base(){}





void red_left_goal() {
  pros::Task colorsort(color_sort_red);
  //chassis.odom_y_flip();
  chassis.odom_x_flip();
  chassis.odom_theta_flip();
  goal_base();
  //goal_base_alt();
  

}

void red_left_stake() {
  pros::Task colorsort(color_sort_red);
  //chassis.odom_y_flip();
  chassis.odom_x_flip();
  chassis.odom_theta_flip();
  stake_base();
  
}

void red_right_rush(){
  pros::Task colorsort(color_sort_red);
  chassis.odom_y_flip();
  chassis.odom_x_flip();
  chassis.odom_theta_flip();

  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 150_ms, 3_in, 250_ms, 500_ms);
  //start at x=+12.5,y=-8.25
  chassis.odom_look_ahead_set(40_in);
  chassis.pid_odom_set(44, 127);
  chassis.pid_wait();
  chassis.odom_look_ahead_set(7_in);
  chassis.pid_turn_set(35, 100);
  chassis.pid_wait();
  Doink.set_value(1);
  pros::delay(60);
  chassis.pid_turn_set(90, 80);
  chassis.pid_wait();
  Doink.set_value(0);
  pros::delay(250);
  chassis.pid_turn_set(-93,100);
  chassis.pid_wait();
  chassis.odom_look_ahead_set(20_in);
  chassis.pid_odom_set(-16,100);
  chassis.pid_wait();
  Clamp.set_value(1);
  Intake2.move_velocity(120);
  chassis.pid_turn_set(-168,100);
  chassis.pid_wait();
  Intake2.move_velocity(0);
  Intake1.move_velocity(-200);
  chassis.pid_odom_set(12,100);
  chassis.pid_wait();
  chassis.pid_turn_set(-258,100);
  chassis.pid_wait();
  Clamp.set_value(0);
  chassis.pid_turn_set(-437,100);
  chassis.pid_wait();
  chassis.pid_odom_set(-18,100);
  chassis.pid_wait();
  Clamp.set_value(1);
  Intake2.move_velocity(120);
  Intake1.move_velocity(-200);
  pros::delay(1000);
  chassis.odom_look_ahead_set(7_in);
  chassis.pid_turn_set(-537,100);
  chassis.pid_wait();
  Lifter.set_value(1);
  chassis.pid_odom_set( {{{54+12.5,3+8.25}, fwd, 110}} );
  chassis.pid_wait();
  Lifter.set_value(0);
  chassis.pid_odom_set( {{{54+12.5-12,3+8.25}, fwd, 110}} );
  chassis.pid_wait();
  chassis.pid_turn_set(0,100);
  chassis.pid_wait();
  

}






void blue_right_goal(){
  //starts at x=0, y=+7.5 from corner
  pros::Task colorsort(color_sort_blue);
  goal_base();
  
}

void blue_right_stake(){
  //starts at x=-7.5, y=+9.75 from corner
  pros::Task colorsort(color_sort_blue);
  stake_base();
}

void blue_left_rush(){
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 150_ms, 3_in, 250_ms, 500_ms);
  pros::Task colorsort(color_sort_blue);
  //start at x=+12.5,y=-8.25
  chassis.odom_look_ahead_set(40_in);
  chassis.pid_odom_set(44, 127);
  chassis.pid_wait();
  chassis.odom_look_ahead_set(7_in);
  chassis.pid_turn_set(35, 100);
  chassis.pid_wait();
  Doink.set_value(1);
  pros::delay(400);
  chassis.pid_turn_set(90, 80);
  chassis.pid_wait();
  Doink.set_value(0);
  pros::delay(250);
  chassis.pid_turn_relative_set(-183,100);
  chassis.pid_wait();
  chassis.odom_look_ahead_set(20_in);
  chassis.pid_odom_set(-16,100);
  chassis.pid_wait();
  Clamp.set_value(1);
  Intake2.move_velocity(120);
  chassis.pid_turn_relative_set(-75,100);
  chassis.pid_wait();
  Intake2.move_velocity(0);
  Intake1.move_velocity(-200);
  chassis.pid_odom_set(12,100);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(-90,100);
  chassis.pid_wait();
  Clamp.set_value(0);
  chassis.pid_turn_relative_set(-179,100);
  chassis.pid_wait();
  chassis.pid_odom_set(-18,100);
  chassis.pid_wait();
  Clamp.set_value(1);
  Intake2.move_velocity(120);
  Intake1.move_velocity(-200);
  pros::delay(1000);
  chassis.odom_look_ahead_set(7_in);
  chassis.pid_turn_relative_set(-100,100);
  chassis.pid_wait();
  Lifter.set_value(1);
  chassis.pid_odom_set( {{{54+12.5,3+8.25}, fwd, 110}} );
  chassis.pid_wait();
  Lifter.set_value(0);
  chassis.pid_odom_set( {{{54+12.5-12,3+8.25}, fwd, 110}} );
  chassis.pid_wait();
  chassis.pid_turn_set(0,100);
  chassis.pid_wait();



}

void movearmcustom(int angle){
  int angle_reading = ArmSensor.get_position();
  if (angle_reading<=0|| angle_reading<5000){
    angle_reading=36000+angle_reading;
  }
  int movedir=0;
  if (angle_reading<angle){
    movedir=1;
  }
  else{
    movedir=-1;
  }
  while (abs(angle_reading-angle)>500){
    master.set_text(0,0,to_string(angle_reading)+" "+to_string(ArmSensor.get_position()));
    angle_reading = ArmSensor.get_position();
    if (angle_reading<=0|| angle_reading<5000){
      angle_reading=36000+angle_reading;
    }
    Arm.move_velocity(movedir*150);
    pros::delay(5);
  }
  Arm.move_velocity(0);
}

void armcustom(int angle){
  Arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  int tickspassed=0;
  armPid.target_set(angle);
  while(tickspassed<200){
    int angle_reading = ArmSensor.get_position();
    if (angle_reading<0|| angle_reading<1000){
      angle_reading=36000+angle_reading;
    }
    Arm.move(armPid.compute(angle_reading));
    tickspassed++;
    pros::delay(10);
  }
  Arm.move(0);

}

void skillsauto(){
  ArmSensor.set_position(33200);
  Arm.set_zero_position(0);
  pros::delay(500);
  chassis.odom_look_ahead_set(7_in);
  Arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  double starty=7;
  Arm.move_velocity(-200); 
  pros::delay(750);
  Arm.move_velocity(0); 
  chassis.pid_odom_set( {{{0,-6-starty}, rev, 110},{{-24,-starty}, rev, 110}} );
  chassis.pid_wait();
  Arm.move_velocity(200); 
  Clamp.set_value(1);
  chassis.pid_turn_set(180,100);
  chassis.pid_wait();
  Arm.move_velocity(0);
  Intake1.move_velocity(-200);
  Intake2.move_velocity(150);
  chassis.pid_odom_set( {{{-24,-24-starty}, fwd, 90},{{-48,-24-starty}, fwd, 110},{{-60,-50-starty}, fwd, 90}} );
  chassis.pid_wait();
  pros::delay(750);
  movearmcustom(33200);
  chassis.pid_odom_set({{{-48,-78-starty}, fwd, 110}});
  chassis.pid_wait();
  pros::delay(1250);
  Intake2.move_velocity(0);
  Intake1.move_velocity(0);
  Intake2.move_velocity(-50);
  pros::delay(50);
  Intake2.move_velocity(0);
  pros::delay(1500);
  movearmcustom(30000);
  chassis.pid_odom_set({{{-48,-48-starty}, rev, 110}});
  chassis.pid_wait();
  chassis.pid_turn_set(-90,100);
  chassis.pid_wait();
  chassis.pid_odom_set(12,90);
  chassis.pid_wait();
  movearmcustom(20000);
  chassis.pid_odom_set(-12,90);
  chassis.pid_wait();
  Arm.move_velocity(150);
  chassis.pid_turn_set(0,100);
  chassis.pid_wait();
  Arm.move_velocity(0);





  
}