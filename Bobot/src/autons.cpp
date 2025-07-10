#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////


// SETUP SECTION


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
  chassis.pid_drive_constants_backward_set(14, -20, 100);
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
  chassis.pid_drive_constants_backward_set(14, -20, 100);
  chassis.pid_turn_constants_set(2.5, 0, 8, 15);
  chassis.pid_swing_constants_forward_set(5, 0, 20);
  chassis.pid_swing_constants_backward_set(4,0,20);
  chassis.pid_odom_angular_constants_set(3.09, 0.0, 20);    // Angular control for odom motions
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







// TESTING SECTION


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







// COLOR SORT SECTION


///*
void color_sort_blue(){
  //this variable stores the color of the ring currently in the intake
  //1 stands for red, 0 stands for none, -1 stands for blue
  int currentcolor=1;
  
  //this variable stores the raw color reading from the color sensor
  double rawcolorval=40;

  //indicates which color is being let onto the goal
  master.set_text(0,0,"running red    ");

  //loop to make sure task runs continuoously 
  while(true){
    
    //sets alliance color (allows these rings through)
    int allicolor=-1;

    //only color-sorts if intake is running
    if(Intake2.get_target_velocity()!=0){

      //get new color reading
      rawcolorval=Intakecolor.get_hue();

      //assign currentcolor based on raw reading, display to screen
      if ((rawcolorval<20)||(rawcolorval>330)){
        currentcolor=1; 
        master.set_text(0,0,"red         ");
      }
      else if ((rawcolorval>120)&&(rawcolorval<330)){
        currentcolor=-1;
        master.set_text(0,0,"blue          ");
      }

      //if ring is wrong color and detected on intake
      if (Intakedist.get()<50 && currentcolor!=allicolor){

        //small delay to allow ring to reach best position to throw off
        pros::delay(50);
        //reverses intake to throw ring off
        Intake2.move_velocity(-120);
        //delay to allow ring to clear intake
        pros::delay(75);
        //move intake the same way again
        Intake2.move_velocity(120);
        //reset color variable
        currentcolor=0;
        //indicate that ring is filtered out
        master.set_text(0,0,"exec          ");
      }
    }

    //delay to save resources
    pros::delay(5);
  }
}

void color_sort_red(){
  //this variable stores the color of the ring currently in the intake
  //1 stands for red, 0 stands for none, -1 stands for blue
  int currentcolor=1;
  
  //this variable stores the raw color reading from the color sensor
  double rawcolorval=40;

  //indicates which color is being let onto the goal
  master.set_text(0,0,"running red    ");

  //loop to make sure task runs continuoously 
  while(true){
    
    //sets alliance color (allows these rings through)
    int allicolor=1;

    //only color-sorts if intake is running
    if(Intake2.get_target_velocity()!=0){

      //get new color reading
      rawcolorval=Intakecolor.get_hue();

      //assign currentcolor based on raw reading, display to screen
      if ((rawcolorval<20)||(rawcolorval>330)){
        currentcolor=1; 
        master.set_text(0,0,"red         ");
      }
      else if ((rawcolorval>120)&&(rawcolorval<330)){
        currentcolor=-1;
        master.set_text(0,0,"blue          ");
      }

      //if ring is wrong color and detected on intake
      if (Intakedist.get()<50 && currentcolor!=allicolor){

        //small delay to allow ring to reach best position to throw off
        pros::delay(50);
        //reverses intake to throw ring off
        Intake2.move_velocity(-120);
        //delay to allow ring to clear intake
        pros::delay(75);
        //move intake the same way again
        Intake2.move_velocity(120);
        //reset color variable
        currentcolor=0;
        //indicate that ring is filtered out
        master.set_text(0,0,"exec          ");
      }
    }

    //delay to save resources
    pros::delay(5);
  }
}
//*/

void colorsorttest(){
  Intakecolor.set_led_pwm(100);
  Intake1.move_velocity(-200);
  Intake2.move_velocity(120);
  pros::delay(15000);
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
  int counter=0;
  while (abs(angle_reading-angle)>500 && counter <= 700/5){
    master.set_text(0,0,to_string(angle_reading)+" "+to_string(ArmSensor.get_position()));
    angle_reading = ArmSensor.get_position();
    if (angle_reading<=0|| angle_reading<5000){ 
      angle_reading=36000+angle_reading;
    }
    Arm.move_velocity(movedir*150);
    pros::delay(5);
    counter++;
  }
  Arm.move_velocity(0);
}



// BASE SECTION


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

  chassis.odom_look_ahead_set(10_in);
  chassis.odom_enable(true);
  chassis.pid_odom_set( {{{0,-30}, rev, 110}} );
  chassis.pid_wait();

  Clamp.set_value(1);

  Intake2.move_velocity(120);
  Intake1.move_velocity(-200);

  chassis.pid_turn_set(-135,100);
  chassis.pid_wait();

  chassis.pid_odom_set( {{{-12,-41}, fwd, 110},{{-24,-46}, fwd, 110}});
  chassis.pid_wait();

  chassis.pid_odom_set({{-38,-42}, fwd, 110});
  chassis.pid_wait();

  chassis.pid_turn_set(-95,100);
  chassis.pid_wait();

  chassis.pid_odom_set(-24,100);
  chassis.pid_wait();

  chassis.pid_odom_set( {{{-28, -30}, fwd, 110}} );
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

void goal_base_corner(){
  chassis.odom_look_ahead_set(10_in);
  chassis.odom_enable(true);
  chassis.pid_odom_set( {{{0,-30}, rev, 110}} );
  chassis.pid_wait();

  Clamp.set_value(1);

  Intake2.move_velocity(120);
  Intake1.move_velocity(-200);

  chassis.pid_turn_set(-135,100);
  chassis.pid_wait();

  chassis.pid_odom_set( {{{-12,-41}, fwd, 110},{{-24,-46}, fwd, 110}});
  chassis.pid_wait();

  chassis.pid_odom_set({{-36,-40}, fwd, 110});
  chassis.pid_wait();

  chassis.pid_turn_set(-90,100);
  chassis.pid_wait();

  chassis.pid_odom_set(-40,100);
  chassis.pid_wait();

  chassis.pid_odom_set( {{{-28, -28}, fwd, 110}} );     
  chassis.pid_wait();

  chassis.pid_turn_set(-90,100);
  chassis.pid_wait();

  Lifter.set_value(1);

  chassis.pid_odom_set(-26,100);
  chassis.pid_wait();

  chassis.pid_turn_set(-45,100);
  chassis.pid_wait();

  chassis.pid_odom_set(57,100);
  chassis.pid_wait();

  chassis.pid_odom_set(-6,100);
  chassis.pid_wait();

  Lifter.set_value(0);

  chassis.pid_odom_set(-12,100);
  chassis.pid_wait();
  
  chassis.pid_turn_set(90,100);
  chassis.pid_wait();
  
  Lifter.set_value(1);

  chassis.pid_odom_set(52,100);
  chassis.pid_wait();

  Lifter.set_value(0);

  chassis.pid_turn_set(-180,100);
  chassis.pid_wait();

  chassis.pid_odom_set(12,100);
  chassis.pid_wait();
}

void stake_base(){

  chassis.odom_look_ahead_set(10_in);
  //based on blue
  chassis.pid_swing_set(ez::RIGHT_SWING,-53,100);
  chassis.pid_wait();

  Arm.move_velocity(-150);
  pros::delay(750);
  Arm.move_velocity(0); 

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

  chassis.pid_odom_set( {{{36+7.5,-12-9.75}, fwd, 110},{{44,-24-9.75}, fwd, 110},{{44,-36-9.75}, fwd, 110}});
  chassis.pid_wait();

  chassis.pid_odom_set( {{{26+7.5,-9.75}, rev, 110}} );
  chassis.pid_wait();

  chassis.pid_turn_set(-90, 100);
  chassis.pid_wait();

  Lifter.set_value(1);

  chassis.pid_odom_set( {{{5,24-9.75}, fwd, 110}} );
  chassis.pid_wait();

  Lifter.set_value(0);

  chassis.pid_turn_set(80, 100);
  chassis.pid_wait();

  chassis.pid_odom_set(16,100);
  chassis.pid_wait(); 
}

void stake_base_alt(){  
  chassis.odom_look_ahead_set(10_in);
  //based on blue
  chassis.pid_swing_set(ez::RIGHT_SWING,-53,100);
  chassis.pid_wait();

  Arm.move_velocity(-150);
  pros::delay(750);
  Arm.move_velocity(0); 

  chassis.pid_odom_set( {{{30+7.5,-4-9.75}, rev, 110}} );
  chassis.pid_wait();

  Arm.move_absolute(75,150);

  Clamp.set_value(1);

  Intake2.move_velocity(140);
  Intake1.move_velocity(-200);

  chassis.pid_turn_set(-135,100);
  chassis.pid_wait();

  chassis.pid_odom_set( {{{38+7.5,-12-9.75}, fwd, 110},{{47,-24-9.75}, fwd, 110},{{47,-26-9.75}, fwd, 110}});
  chassis.pid_wait();

  pros::delay(250);

  chassis.pid_odom_set({{42,-39-9.75}, fwd, 110});
  chassis.pid_wait();

  chassis.pid_odom_set( {{{32+7.5,-9.75}, rev, 110}} );
  chassis.pid_wait();

  chassis.pid_odom_set( {{{26+7.5,-26-9.75}, fwd, 110}} );
  chassis.pid_wait();
  
  pros::delay(250);

  chassis.pid_turn_set(-90, 100);
  chassis.pid_wait();

  Lifter.set_value(1);

  chassis.pid_odom_set( {{{12,18-9.75}, fwd, 110}} );
  chassis.pid_wait();

  Lifter.set_value(0);

  chassis.pid_turn_set(80, 100);
  chassis.pid_wait();

  chassis.pid_odom_set(18,100);
  chassis.pid_wait(); 

  

}

void stake_base_corner(){
  chassis.odom_look_ahead_set(10_in);
  //based on blue
  chassis.pid_swing_set(ez::RIGHT_SWING,-53,100);
  chassis.pid_wait();

  Arm.move_velocity(-150);
  pros::delay(750);
  Arm.move_velocity(0); 

  chassis.pid_odom_set( {{{30+7.5,-4-9.75}, rev, 110}} );
  chassis.pid_wait();

  Arm.move_absolute(75,150);

  Clamp.set_value(1);

  Intake2.move_velocity(140);
  Intake1.move_velocity(-200);

  chassis.pid_turn_set(-135,100);
  chassis.pid_wait();

  chassis.pid_odom_set( {{{38+7.5,-12-9.75}, fwd, 110},{{47,-18-9.75}, fwd, 110},{{47,-26-9.75}, fwd, 110}});
  chassis.pid_wait();

  pros::delay(250); 

  chassis.pid_odom_set({{42,-39-9.75}, fwd, 110});
  chassis.pid_wait();

  chassis.pid_odom_set( {{{32+7.5,-9.75}, rev, 110}} );
  chassis.pid_wait();

  chassis.pid_odom_set( {{{26+7.5,-26-9.75}, fwd, 110}} );
  chassis.pid_wait();

  chassis.pid_odom_set( {{{28+7.5,-9.75}, rev, 110}} );
  chassis.pid_wait();

  chassis.pid_turn_set(-135,100);
  chassis.pid_wait();

  Lifter.set_value(1); 
  
  chassis.pid_odom_set(59,100);
  chassis.pid_wait();

  chassis.pid_odom_set(-6,100);
  chassis.pid_wait();

  Lifter.set_value(0);

  chassis.pid_odom_set(-12,100);
  chassis.pid_wait();
  
  chassis.pid_turn_set(0,100);
  chassis.pid_wait();
  
  Lifter.set_value(1);

  chassis.pid_odom_set(52,100);
  chassis.pid_wait();

  Lifter.set_value(0);

  chassis.pid_turn_set(90,100);
  chassis.pid_wait();

  chassis.pid_odom_set(16,100);
  chassis.pid_wait();
}

void stake_base_wall(){

  ArmSensor.set_position(33200);
  Arm.set_zero_position(0);

  //setting arm braking, ensures that arm holds itself up
  Arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  chassis.odom_look_ahead_set(10_in);
  //based on blue
  chassis.pid_swing_set(ez::RIGHT_SWING,-53,100);
  chassis.pid_wait();

  Arm.move_velocity(-150);
  pros::delay(750);
  Arm.move_velocity(0); 

  chassis.pid_odom_set( {{{30+7.5,-4-9.75}, rev, 110}} );
  chassis.pid_wait();

  Arm.move_absolute(75,150);

  Clamp.set_value(1);

  Intake2.move_velocity(140);
  Intake1.move_velocity(-200);

  chassis.pid_turn_set(-135,100);
  chassis.pid_wait();

  chassis.pid_odom_set( {{{38+7.5,-12-9.75}, fwd, 110},{{47,-28-9.75}, fwd, 110}});
  chassis.pid_wait();

  pros::delay(250);

  chassis.pid_odom_set({{42,-36-9.75}, fwd, 110});
  chassis.pid_wait();

  chassis.pid_odom_set( {{{32+7.5,-9.75}, rev, 110}} );
  chassis.pid_wait();

  movearmcustom(33200+400);

  chassis.pid_odom_set( {{{26+7.5,-24-9.75}, fwd, 110}} );
  chassis.pid_wait();

  pros::delay(500);

  Intake2.move_velocity(0);
  Intake1.move_velocity(0);

  chassis.pid_turn_set(135,100);
  chassis.pid_wait();

    chassis.pid_odom_set(24,100);
    chassis.pid_wait();

    Intake2.move_velocity(-120);

    Arm.move_velocity(-200);
    pros::delay(750);
    Arm.move_velocity(0);

    Intake2.move_velocity(140);
    Intake1.move_velocity(-200);
    
  chassis.pid_odom_set( {{{22+7.5,-9.75}, rev, 110}} );
  chassis.pid_wait();

  Arm.move_velocity(150);

  chassis.pid_turn_set(-135,100);
  chassis.pid_wait();

  Arm.move_velocity(0);

  Lifter.set_value(1);
  
  chassis.pid_odom_set(57,100);
  chassis.pid_wait();

  chassis.pid_odom_set(-6,100);
  chassis.pid_wait();

  Lifter.set_value(0);

  chassis.pid_odom_set(-12,100);
  chassis.pid_wait();
  
  chassis.pid_turn_set(0,100);
  chassis.pid_wait();
  
  Lifter.set_value(1);

  chassis.pid_odom_set(52,100);
  chassis.pid_wait();

  Lifter.set_value(0);

  chassis.pid_turn_set(90,100);
  chassis.pid_wait();

  chassis.pid_odom_set(16,100);
  chassis.pid_wait();
}

void rush_base_blue(){
  chassis.odom_look_ahead_set(40_in);

  chassis.pid_odom_set(44, 120);
  chassis.pid_wait();

  chassis.odom_look_ahead_set(7_in);

  chassis.pid_turn_set(35, 100);
  chassis.pid_wait();

  Doink.set_value(1);

  pros::delay(400);

  chassis.pid_turn_set(90, 80);
  chassis.pid_wait();

  Doink.set_value(0);

  pros::delay(500);

  chassis.pid_turn_set(-100,100);
  chassis.pid_wait();

  chassis.odom_look_ahead_set(20_in);

  chassis.pid_odom_set(-16,100);
  chassis.pid_wait();
  Clamp.set_value(1);

  Intake2.move_velocity(120);

  chassis.pid_turn_set(-170,100);
  chassis.pid_wait();

  Intake2.move_velocity(0);
  Intake1.move_velocity(-200);

  chassis.pid_odom_set(14,100);
  chassis.pid_wait();

  chassis.pid_turn_set(90,100);
  chassis.pid_wait();

  Clamp.set_value(0);

  chassis.pid_turn_set(-90,100);
  chassis.pid_wait();

  chassis.pid_odom_set(-24,100);
  chassis.pid_wait();

  Clamp.set_value(1);

  Intake2.move_velocity(120);
  Intake1.move_velocity(-200);

  pros::delay(1000);

  chassis.odom_look_ahead_set(7_in);

  chassis.pid_turn_set(135,100);
  chassis.pid_wait();

  Lifter.set_value(1);

  chassis.pid_odom_set( {{{60+12.5,0-8.25}, fwd, 110}} );
  chassis.pid_wait();

  Lifter.set_value(0);

  chassis.pid_odom_set( {{{60+12.5-8,0-8.25+12}, fwd, 110}} );
  chassis.pid_wait();

  chassis.pid_turn_set(0,100);
  chassis.pid_wait();

  chassis.pid_odom_set(20, 100);
  chassis.pid_wait();
}

void rush_base_red(){
  chassis.odom_look_ahead_set(40_in);

  chassis.pid_odom_set(44, 120);
  chassis.pid_wait();

  chassis.odom_look_ahead_set(7_in);

  chassis.pid_turn_set(75, 100);
  chassis.pid_wait();

  Doink.set_value(1);

  pros::delay(400);

  chassis.pid_turn_set(120, 80);
  chassis.pid_wait();

  Doink.set_value(0);

  pros::delay(500);

  chassis.pid_turn_set(-100,100);
  chassis.pid_wait();

  chassis.odom_look_ahead_set(20_in);

  chassis.pid_odom_set(-16,100);
  chassis.pid_wait();
  Clamp.set_value(1);

  Intake2.move_velocity(120);

  chassis.pid_turn_set(-170,100);
  chassis.pid_wait();

  Intake2.move_velocity(0);
  Intake1.move_velocity(-200);

  chassis.pid_odom_set(14,100);
  chassis.pid_wait();

  chassis.pid_turn_set(90,100);
  chassis.pid_wait();

  Clamp.set_value(0);

  chassis.pid_turn_set(-90,100);
  chassis.pid_wait();

  chassis.pid_odom_set(-24,100);
  chassis.pid_wait();

  Clamp.set_value(1);

  Intake2.move_velocity(120);
  Intake1.move_velocity(-200);

  pros::delay(1000);

  chassis.odom_look_ahead_set(7_in);

  chassis.pid_turn_set(135,100);
  chassis.pid_wait();

  Lifter.set_value(1);

  chassis.pid_odom_set( {{{60+12.5,0-8.25}, fwd, 110}} );
  chassis.pid_wait();

  Lifter.set_value(0);

  chassis.pid_odom_set( {{{60+12.5-8,0-8.25+12}, fwd, 110}} );
  chassis.pid_wait();

  chassis.pid_turn_set(0,100);
  chassis.pid_wait();

  chassis.pid_odom_set(20, 100);
  chassis.pid_wait();
}

void rush_alt_base_red(){
  chassis.odom_look_ahead_set(40_in);

  chassis.pid_odom_set(44, 127);
  chassis.pid_wait();

  Doink.set_value(1);

  chassis.odom_look_ahead_set(20_in);

  chassis.pid_odom_set(-45,120);
  chassis.pid_wait();

  chassis.odom_look_ahead_set(7_in);

  Doink.set_value(0);

  pros::delay(250);
  
  chassis.pid_turn_set(-170,100);
  chassis.pid_wait();

  chassis.pid_odom_set(-20,110);
  chassis.pid_wait();

  Clamp.set_value(1);

  Intake2.move_velocity(130);

  chassis.pid_odom_set(6,100);
  chassis.pid_wait();

  chassis.pid_turn_set(-75,100);
  chassis.pid_wait();

  chassis.pid_odom_set(-12,110);
  chassis.pid_wait();

  Clamp.set_value(0);

  chassis.pid_odom_set(12,110);
  chassis.pid_wait();

  chassis.pid_turn_set(30,100);
  chassis.pid_wait();
   
  Intake1.move_velocity(-200);
  Intake2.move_velocity(0);

  chassis.pid_odom_set(32,110);
  chassis.pid_wait();

  chassis.pid_turn_set(75,100);
  chassis.pid_wait();

  chassis.pid_odom_set(-30,100);
  chassis.pid_wait();

  Intake2.move_velocity(130);

  Clamp.set_value(1);

  chassis.pid_turn_set(-140,100);
  chassis.pid_wait();

  Lifter.set_value(1);

  chassis.pid_odom_set(30,100);
  chassis.pid_wait();

  Lifter.set_value(0);

  chassis.pid_turn_set(0,100);
  chassis.pid_wait();

  chassis.pid_odom_set(10,100);
  chassis.pid_wait();
  
  pros::delay(1000);



  
  
}

void rush_alt_base_blue(){
  chassis.odom_look_ahead_set(40_in);

  chassis.pid_odom_set(44, 127);
  chassis.pid_wait();

  Doink.set_value(1);

  chassis.odom_look_ahead_set(20_in);

  chassis.pid_odom_set(-40,120);
  chassis.pid_wait();

  Doink.set_value(0);

  pros::delay(250);

  chassis.pid_turn_relative_set(190,100);
  chassis.pid_wait();

  chassis.pid_odom_set(-30,110);
  chassis.pid_wait();

  Clamp.set_value(1);

  Intake2.move_velocity(130);

  chassis.pid_turn_set(65,100);
  chassis.pid_wait();

  Clamp.set_value(0);

  Intake1.move_velocity(-200);
  Intake2.move_velocity(0);

  chassis.pid_odom_set(16,110);
  chassis.pid_wait();

  chassis.pid_turn_relative_set(-130,100);
  chassis.pid_wait();
  
  chassis.pid_odom_set(-24,110);
  chassis.pid_wait();

  Clamp.set_value(1);
  
  Intake2.move_velocity(130);

  pros::delay(1000);

  chassis.pid_turn_set(125,100);
  chassis.pid_wait();

  Lifter.set_value(1);

  chassis.pid_odom_set(24,100);
  chassis.pid_wait();

  Lifter.set_value(0);

  chassis.pid_turn_set(0,100);
  chassis.pid_wait();

  chassis.pid_odom_set(10,100);
  chassis.pid_wait();


}

void awp_base(){
  chassis.odom_look_ahead_set(10_in);
  //based on blue
  chassis.pid_swing_set(ez::LEFT_SWING,53,100);
  chassis.pid_wait();

  Arm.move_velocity(-150);
  pros::delay(750);
  Arm.move_velocity(0); 

  chassis.pid_odom_set( {{{-26-7.5,-9.75}, rev, 100}} );
  chassis.pid_wait();

  Arm.move_absolute(75,150);

  Clamp.set_value(1);

  Intake2.move_velocity(140);
  Intake1.move_velocity(-200);

  chassis.pid_turn_set(-180,100);
  chassis.pid_wait();

  chassis.pid_odom_set( {{{-26-7.5,-24-9.75}, fwd, 110}} );
  chassis.pid_wait();

  pros::delay(250);
  chassis.pid_turn_set(30,100);
  chassis.pid_wait();

  Lifter.set_value(1);

  chassis.pid_odom_set( {{{-18,8-9.75}, fwd, 110}} );
  chassis.pid_wait();

  Lifter.set_value(0);

  pros::delay(500);

  Clamp.set_value(0);

  chassis.pid_turn_set(100,100);
  chassis.pid_wait();

  Intake2.move_velocity(00); 

  chassis.pid_odom_set( {{{-40-7.5,32-9.75}, rev, 110}} );
  chassis.pid_wait();

  Clamp.set_value(1);

  Intake2.move_velocity(140);
  Intake1.move_velocity(-200);

  pros::delay(250);

  chassis.pid_turn_set(0,100);
  chassis.pid_wait();

  chassis.pid_odom_set( {{{-38-7.5,56-9.75}, fwd, 110}} );
  chassis.pid_wait();

  chassis.pid_turn_set(-180,100);
  chassis.pid_wait();

  chassis.pid_odom_set(50,100);
  chassis.pid_wait();


}

void safe_base(){
  //based on red
  
  chassis.odom_enable(true);

  chassis.pid_odom_set( {{{0,-30}, rev, 110}} );
  chassis.pid_wait();

  Clamp.set_value(1);

  Intake2.move_velocity(120);
  Intake1.move_velocity(-200);

  chassis.pid_odom_set( {{{-28, -30}, fwd, 110}} );
  chassis.pid_wait();

  pros::delay(1000);

  chassis.pid_turn_set(135,100);
  chassis.pid_wait();

  Clamp.set_value(0);

  chassis.pid_turn_set(0,100);
  chassis.pid_wait();
  
  chassis.pid_odom_set( {{{-28, -50}, rev, 110}} );
  chassis.pid_wait();

  Clamp.set_value(1);

  Lifter.set_value(1);

  chassis.pid_odom_set( {{{28, -4}, fwd, 110}} );
  chassis.pid_wait();

  Lifter.set_value(0);
  chassis.pid_odom_set(-12,100);
  chassis.pid_wait();
  chassis.pid_odom_set(6,100);
  chassis.pid_wait();
  chassis.pid_turn_set(-180,100);
  chassis.pid_wait();
  chassis.pid_odom_set(16,100);
  chassis.pid_wait();


}
// RED SECTION


void red_left_goal() {
  pros::Task colorsort(color_sort_red);
  //chassis.odom_y_flip();
  chassis.odom_x_flip();
  chassis.odom_theta_flip();
  //goal_base();
  goal_base_corner();
  

}

void red_left_stake() { 
  pros::Task colorsort(color_sort_red);
  //chassis.odom_y_flip();
  chassis.odom_x_flip();
  chassis.odom_theta_flip();
  stake_base_corner();
  
}

void red_left_stake_alt() {
  pros::Task colorsort(color_sort_red);
  //chassis.odom_y_flip();
  chassis.odom_x_flip();
  chassis.odom_theta_flip();
  stake_base_alt();
  
}

void red_left_stake_wall() {
  pros::Task colorsort(color_sort_red);
  //chassis.odom_y_flip();
  chassis.odom_x_flip();
  chassis.odom_theta_flip();
  stake_base_wall();
  
}

void red_right_rush(){
  pros::Task colorsort(color_sort_red);
  //chassis.odom_y_flip();
  chassis.odom_x_flip();
  chassis.odom_theta_flip();

  rush_base_red();
  

}

void red_right_rush_alt(){
  pros::Task colorsort(color_sort_red);

  rush_alt_base_red();
}

void red_right_awp(){
  pros::Task colorsort(color_sort_red);
  //chassis.odom_y_flip();
  chassis.odom_x_flip();
  chassis.odom_theta_flip();
  awp_base();
}

void red_right_safe(){
  pros::Task colorsort(color_sort_red);
  safe_base();
}


// BLUE SECTION


void blue_right_goal(){
  //starts at x=0, y=+7.5 from corner
  pros::Task colorsort(color_sort_blue);
  goal_base_corner();
  
}

void blue_right_stake(){
  //starts at x=-7.5, y=+9.75 from corner
  pros::Task colorsort(color_sort_blue);
  stake_base_corner();
}

void blue_right_stake_alt(){
  //starts at x=-7.5, y=+9.75 from corner
  pros::Task colorsort(color_sort_blue);
  stake_base_alt();
}

void blue_right_stake_wall(){
  //starts at x=-7.5, y=+9.75 from corner
  pros::Task colorsort(color_sort_blue);
  stake_base_wall();
}

void blue_left_rush(){
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 150_ms, 3_in, 250_ms, 500_ms);
  pros::Task colorsort(color_sort_blue);
  //start at x=+12.5,y=-8.25
  rush_base_blue();



}

void blue_left_rush_alt(){
  pros::Task colorsort(color_sort_blue);

  rush_alt_base_blue();
}

void blue_left_awp(){
  pros::Task colorsort(color_sort_blue);

  awp_base();
}

void blue_left_safe(){
  pros::Task colorsort(color_sort_blue);
  //chassis.odom_y_flip();
  chassis.odom_x_flip();
  chassis.odom_theta_flip();
  safe_base();
}


//SKILLS SECTION




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
  // SKILLS AUTONOMOUS
  // Theoretical Score: 43 
  // Set up with back crossbar against inner edge of tile
  // Put preload in arm, position on top of intake 



  //setting exit constants, allows driving movements to stop faster, saving more time 
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 100_ms, 750_ms);

  //offset arm rotation by a certain amount, issue with field control
  int offset = 400;

  //setting arm positions, on encoder and rotation sensor
  ArmSensor.set_position(33200);
  Arm.set_zero_position(0);

  //setting odometry constants, this one controls how far ahead the robot "looks" on its assigned path
  chassis.odom_look_ahead_set(10_in);

  //setting arm braking, ensures that arm holds itself up
  Arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  //setting odometry offset, since robot doesn't start perfectly on the edge of a tile we have to correct it
  double starty=7;

  //moving arm outward with ring to score on alliance stake
  Arm.move_velocity(-200); 
  pros::delay(750);
  Arm.move_velocity(0); 

  //driving backward, ring slips out of arm and stays on alliance stake, goes toward the goal on the right
  chassis.pid_odom_set( {{{0,-6-starty}, rev, 110},{{-24,-starty}, rev, 110}} );
  chassis.pid_wait();

  //clamps goal using pneumatics
  Clamp.set_value(1);

  //moving arm inward to not interfere with rings
  Arm.move_velocity(200); 

  //turning to the first ring, toward blue side
  chassis.pid_turn_set(180,100); 
  chassis.pid_wait();

  //stopping previous arm movements
  Arm.move_velocity(0);

  //start spinning intake 
  Intake1.move_velocity(-200);
  Intake2.move_velocity(140);

  //start intaking rings (3), toward right then blue 
  chassis.pid_odom_set( {{{-24,-24-starty}, fwd, 100},{{-48,-24-starty}, fwd, 110},{{-60,-50-starty}, fwd, 100}} );
  chassis.pid_wait();
  
  //little delay to allow last ring on the goal before arm moves
  pros::delay(250);

  //moving arm upward into "loading" position, notice offset
  movearmcustom(33200+offset);
  
  //moving to nexxt ring, toward blue
  chassis.pid_odom_set({{{-48,-78-starty}, fwd, 110}});
  chassis.pid_wait();

  //delay to allow ring time to move into arm
  pros::delay(700);

  //stopping intake
  Intake2.move_velocity(0);
  Intake1.move_velocity(0);

  //spinnning intake backward while raising arm to "storage" position, prevents hooks from getting stuck
  Intake2.move_velocity(-120);
  movearmcustom(30000);

  //stopping intake again
  Intake2.move_velocity(-0);

  //moving backward to near wall stake
  chassis.pid_odom_set({{{-48,-48-starty}, rev, 110}});
  chassis.pid_wait();

  //turning to wall stake
  chassis.pid_turn_set(-90,100);
  chassis.pid_wait();

  //moving forward, if there are any inaccuracies built up this will correct them using the wall stake aligner
  chassis.pid_odom_set(16,100);
  chassis.pid_wait();

  //moving arm to score the ring on wall stak
  movearmcustom(20000);

  //backing away from wall stake
  chassis.pid_odom_set(-16,115);
  chassis.pid_wait();

  //moving arm back down
  Arm.move_velocity(150);

  //turning toward red to intake last 3 rings
  chassis.pid_turn_set(-3,100);
  chassis.pid_wait();

  //stopping arm 
  Arm.move_velocity(0);

  //start intake
  Intake1.move_velocity(-200);
  Intake2.move_velocity(140);

  //drive into "triangle" of rings 
  chassis.pid_odom_set(64,115);
  chassis.pid_wait();

  //back up
  chassis.pid_odom_set(-14,115);
  chassis.pid_wait();

  //turn to last ring 
  chassis.pid_turn_relative_set(-105,100);
  chassis.pid_wait();

  //intake last irng 
  chassis.pid_odom_set(18,115);
  chassis.pid_wait();

  //back away 
  chassis.pid_odom_set(-16,115);
  chassis.pid_wait();

  //turn back toward corner
  chassis.pid_turn_set(135,100);
  chassis.pid_wait();

  //reverse drive
  chassis.pid_odom_set(-16,115);   
  chassis.pid_wait();

  //put goal down
  Clamp.set_value(0);


  //reversing intake to get rid of excess rings
  Intake1.move_velocity(200);
  Intake2.move_velocity(-140);

  //moving to align with next goal
  chassis.pid_odom_set(16,115);
  chassis.pid_wait();

  //turning toward next goal
  chassis.pid_turn_set(-92,110);
  chassis.pid_wait();

  //mmovign to next goal at full speed
  chassis.odom_look_ahead_set(25_in);
  chassis.pid_odom_set(-72,127);
  chassis.pid_wait();
  chassis.odom_look_ahead_set(10_in);

  //resetting position, trying to eliminate accumulated error
  chassis.odom_xyt_set(0,0,0);

  //clamp next goal
  Clamp.set_value(1);

  //turn to first ring, toward blue
  chassis.pid_turn_set(-90,100);
  chassis.pid_wait();
  
  //start intake 
  Intake1.move_velocity(-200);
  Intake2.move_velocity(140);

  //intaking rings using name path as other side 
  chassis.pid_odom_set( {{{-24,0}, fwd, 100},{{-24,-24}, fwd, 110},{{-50,-30}, fwd, 100}} );
  chassis.pid_wait();

  //turning to go back
  chassis.pid_turn_set(-135,100);
  chassis.pid_wait();

  //reversing away from wall
  chassis.pid_odom_set(-24,115);
  chassis.pid_wait();

  //turnign toward "triangle" of rings, toward red
  chassis.pid_turn_set(90,100);
  chassis.pid_wait();

  //intaking triangle 
  chassis.pid_odom_set({{16,-24},fwd,100});
  chassis.pid_wait();
  
  // back away from wall
  chassis.pid_odom_set(-14,115);
  chassis.pid_wait();

  // turn back toward corner
  chassis.pid_turn_set(-45,100);
  chassis.pid_wait();

  //start here

  //reverse into corner
  chassis.pid_odom_set(-18,115);
  chassis.pid_wait();

  //release goal
  Clamp.set_value(0);

  //drive away from corner
  chassis.pid_odom_set(18,115);
  chassis.pid_wait();

  //start spinning intake to pick up rings along the way
  Intake1.move_velocity(-200);
  Intake2.move_velocity(0);

  //turn toward blue
  chassis.pid_turn_set(-90,100);
  chassis.pid_wait();

  //moving to blue left corner at full speed
  chassis.odom_look_ahead_set(25_in);
  chassis.pid_odom_set(72,127);

  chassis.pid_wait();
  chassis.odom_look_ahead_set(10_in);

  //allowing first ring to move up intake slightly to prepare for second ring
  Intake2.move_velocity(80);

  //turn toward second ring
  chassis.pid_turn_set(0,100);
  chassis.pid_wait();

  //stopping intake
  Intake2.move_velocity(0);

  //intake 2nd ring
  chassis.pid_odom_set(26,115);
  chassis.pid_wait();

  //turn toward last empty goal
  chassis.pid_turn_relative_set(135,100);
  chassis.pid_wait();

  //drive backward into goal
  chassis.pid_odom_set(-40,115);
  chassis.pid_wait();

  //clamp goal
  Clamp.set_value(1);

  //drive forward to prepare to put into corner
  chassis.pid_odom_set(6,115);
  chassis.pid_wait();

  //spin intake and deposit both rings in intake onto goal
  Intake2.move_velocity(140);

  //turn toward corner
  chassis.pid_turn_set(-180,100);
  chassis.pid_wait();

  //drive toward corner while intaking rings
  chassis.pid_odom_set(47,115);
  chassis.pid_wait();

  //turn back toward corner
  chassis.pid_turn_set(45,100);
  chassis.pid_wait();

  //release goal
  Clamp.set_value(0);

  //back off
  chassis.pid_odom_set(6,100);
  chassis.pid_wait();

  //close clamp
  Clamp.set_value(1);

  //push goal into corner
  chassis.pid_odom_set(-26,115);
  chassis.pid_wait();

  //drive out of corner
  chassis.pid_odom_set(18,100);
  chassis.pid_wait();

  //turn towarwd goal with blue ring
  chassis.pid_turn_set(-7,110);
  chassis.pid_wait();

  //drive full speed, push into corner
  chassis.odom_look_ahead_set(40_in);
  chassis.pid_odom_set(110,126);
  chassis.pid_wait();

  //back off
  chassis.pid_odom_set(-12,70);
  chassis.pid_wait();
}

void skillsauto_alt(){  
  // SKILLS AUTONOMOUS
  // Theoretical Score: 43 
  // Set up with back crossbar against inner edge of tile
  // Put preload in arm, position on top of intake 

  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 100_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 100_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 100_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 100_ms, 750_ms);


  //setting exit constants, allows driving movements to stop faster, saving more time 
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 100_ms, 750_ms);

  //offset arm rotation by a certain amount, issue with field control
  int offset = 400;

  //setting arm positions, on encoder and rotation sensor
  ArmSensor.set_position(33200);
  Arm.set_zero_position(0);

  //setting odometry constants, this one controls how far ahead the robot "looks" on its assigned path
  chassis.odom_look_ahead_set(7_in);

  //setting arm braking, ensures that arm holds itself up
  Arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  //setting odometry offset, since robot doesn't start perfectly on the edge of a tile we have to correct it
  double starty=7;

  //moving arm outward with ring to score on alliance stake
  Arm.move_velocity(-200); 
  pros::delay(750);
  Arm.move_velocity(0); 

  //driving backward, ring slips out of arm and stays on alliance stake, goes toward the goal on the right
  chassis.pid_odom_set( {{{0,-6-starty}, rev, 110},{{-24,-starty}, rev, 110}} );
  chassis.pid_wait();

  //clamps goal using pneumatics
  Clamp.set_value(1);

  //moving arm inward to not interfere with rings
  Arm.move_velocity(200); 

  //turning to the first ring, toward blue side
  chassis.pid_turn_set(180,110); 
  chassis.pid_wait();

  //stopping previous arm movements
  Arm.move_velocity(0);

  //start spinning intake 
  Intake1.move_velocity(-200);
  Intake2.move_velocity(130);

  //start intaking rings (3), toward right then blue 
  chassis.pid_odom_set( {{{-24,-24-starty}, fwd, 110},{{-48,-24-starty}, fwd, 110},{{-60,-50-starty}, fwd, 110}} );
  chassis.pid_wait();
  
  //little delay to allow last ring on the goal before arm moves
  pros::delay(350);

  //moving arm upward into "loading" position, notice offset
  movearmcustom(33200+offset);

  //moving to nexxt ring, toward blue
  chassis.pid_odom_set({{{-48,-80-starty}, fwd, 110}});
  chassis.pid_wait();

  //delay to allow ring time to move into arm
  pros::delay(700);

  //stopping intake
  Intake2.move_velocity(0);   
  Intake1.move_velocity(0);

  //spinnning intake backward while raising arm to "storage" position, prevents hoaoks from getting stuck
  Intake2.move_velocity(-120);
  movearmcustom(30000);

  //stopping intake again
  Intake2.move_velocity(0);

  //moving backward to near wall stake
  chassis.pid_odom_set({{{-48,-48-starty}, rev, 110}});
  chassis.pid_wait();

  //turning to wall stake
  chassis.pid_turn_set(-90,110);
  chassis.pid_wait();

  //moving forward, if there are any inaccuracies built up this will correct them using the wall stake aligner
  chassis.pid_odom_set(16,110);
  chassis.pid_wait();

  //moving arm to score the ring on wall stak
  movearmcustom(20000);

  //backing away from wall stake
  chassis.pid_odom_set(-12,115);
  chassis.pid_wait();
 
  //moving arm back down
  Arm.move_velocity(150);

  //turning toward red to intake last 3 rings
  chassis.pid_turn_set(-3,110);
  chassis.pid_wait();

  //stopping arm 
  Arm.move_velocity(0);

  //start intake
  Intake1.move_velocity(-200);
  Intake2.move_velocity(130);

  //drive into "triangle" of rings 
  chassis.pid_odom_set(64,127);
  chassis.pid_wait();

  //back up
  chassis.pid_odom_set(-14,115);
  chassis.pid_wait();

  //turn to last ring 
  chassis.pid_turn_relative_set(-105,110);
  chassis.pid_wait();

  //intake last irng 
  chassis.pid_odom_set(18,115);
  chassis.pid_wait();

  //back away 
  chassis.pid_odom_set(-16,115);
  chassis.pid_wait();

  //turn back toward corner
  chassis.pid_turn_set(135,110);
  chassis.pid_wait();

  //reverse drive
  chassis.pid_odom_set(-16,115);   
  chassis.pid_wait();

  //put goal down
  Clamp.set_value(0);


  //reversing intake to get rid of excess rings
  Intake1.move_velocity(200);
  Intake2.move_velocity(-140);

  //moving to align with next goal
  chassis.pid_odom_set(16,115);
  chassis.pid_wait();

  //turning toward next goal
  chassis.pid_turn_set(-91,110);
  chassis.pid_wait();

  //mmovign to next goal at full speed
  chassis.odom_look_ahead_set(25_in);
  chassis.pid_odom_set(-72,127);
  chassis.pid_wait();
  chassis.odom_look_ahead_set(7_in);

  //resetting position, trying to eliminate accumulated error
  chassis.odom_xyt_set(0,0,0);

  //clamp next goal
  Clamp.set_value(1);

  //turn to first ring, toward blue
  chassis.pid_turn_set(-90,110);
  chassis.pid_wait();
  
  //start intake 
  Intake1.move_velocity(-200);
  Intake2.move_velocity(130);

  //intaking rings using name path as other side 
  chassis.pid_odom_set( {{{-24,0}, fwd, 100},{{-24,-24}, fwd, 110},{{-50,-36}, fwd, 100}} );
  chassis.pid_wait();

  //small delay
  pros::delay(750);

  //moving arm upward into "loading" position, notice offset
  movearmcustom(33200+offset);


  chassis.pid_odom_set( {{{-82,-23}, fwd, 110}} );
  chassis.pid_wait();
  
  pros::delay(700);

  //stopping intake
  Intake2.move_velocity(0);
  Intake1.move_velocity(0);

  //spinnning intake backward while raising arm to "storage" position, prevents hooks from getting stuck
  Intake2.move_velocity(-120);

  movearmcustom(30000);

  //stopping intake again
  Intake2.move_velocity(-0);

  chassis.pid_odom_set( {{{-50,-27}, rev, 110}} );
  chassis.pid_wait();

  chassis.pid_turn_set(-180,110);
  chassis.pid_wait();

  chassis.pid_odom_set(20,110);
  chassis.pid_wait();



  movearmcustom(20000);

  //backing away from wall stake
  chassis.pid_odom_set(-12,115);
  chassis.pid_wait();

  //moving arm back down
  Arm.move_velocity(150);

  //turnign toward "triangle" of rings, toward red
  chassis.pid_turn_set(92,110);
  chassis.pid_wait();

  //stopping arm 
  Arm.move_velocity(0);

  Intake1.move_velocity(-200);
  Intake2.move_velocity(130);

  chassis.odom_look_ahead_set(20);

  //intaking triangle 
  chassis.pid_odom_set({{14,-28},fwd,115});
  chassis.pid_wait();

  chassis.odom_look_ahead_set(7);
  
  // back away from wall
  chassis.pid_odom_set(-12,115);
  chassis.pid_wait();

  // turn back toward corner
  chassis.pid_turn_set(-50,110);
  chassis.pid_wait();

  //reverse into corner
  chassis.pid_odom_set(-18,115);
  chassis.pid_wait();

  //release goal
  Clamp.set_value(0);

  //drive away from corner
  chassis.pid_odom_set(18,115);
  chassis.pid_wait();

  //turn toward blue
  chassis.pid_turn_set(-90,110);
  chassis.pid_wait();

  //moving to blue left corner at full speed
  chassis.odom_look_ahead_set(25_in);
  chassis.pid_odom_set(70,127);

  chassis.pid_wait();

  //turn toward second ring
  chassis.pid_turn_set(155,110);
  chassis.pid_wait();

  chassis.odom_look_ahead_set(20_in);

  //drive backward into goal
  chassis.pid_odom_set(-56,115);
  chassis.pid_wait(); 

  //clamp goal
  Clamp.set_value(1);

  //spin intake and deposit both rings in intake onto goal
  Intake2.move_velocity(140);
  Intake1.move_velocity(-200);

  //turn toward corner
  chassis.pid_turn_set(-178,110);
  chassis.pid_wait();

  //drive toward corner while intaking rings
  chassis.pid_odom_set(59,115);
  chassis.pid_wait();

  pros::delay(500);

  //turn back toward corner
  chassis.pid_turn_set(70,110);
  chassis.pid_wait();

  //release goal
  Clamp.set_value(0);

  //push goal into corner
  chassis.pid_odom_set(-16,115);
  chassis.pid_wait();

  //drive out of corner
  chassis.pid_odom_set(20,115);
  chassis.pid_wait();

  //turn towarwd goal with blue ring
  chassis.pid_turn_set(-10,110);
  chassis.pid_wait();

  Intake1.move_velocity(200);

  //drive full speed, push into corner
  chassis.odom_look_ahead_set(40_in);
  chassis.pid_odom_set(115,127);
  chassis.pid_wait();

  //back off
  chassis.pid_odom_set(-12,70); 
  chassis.pid_wait();
}

void vexairush(){
  Doink.set_value(true);

  chassis.pid_drive_set(30,115);
  chassis.pid_wait_until(29_in);

  chassis.pid_turn_set(-30,100);
  chassis.pid_wait_until(-28);

  chassis.pid_drive_set(6,110);
  chassis.pid_wait();

  Doink.set_value(true);

  chassis.pid_turn_set(-26,100);
  chassis.pid_wait();

  chassis.pid_drive_set(-18,100);
  chassis.pid_wait();

  Doink.set_value(false);

  chassis.pid_drive_set(-12,100);
  chassis.pid_wait();

  Doink.set_value(false);

  chassis.pid_turn_set(180-35,100);
  chassis.pid_wait();

  chassis.pid_drive_set(-12,100);
  chassis.pid_wait();

  Clamp.set_value(1);

  chassis.pid_turn_set(-26,100);
  chassis.pid_wait();

  chassis.pid_drive_set(-23,110);
  chassis.pid_wait();

  
  Intake1.move_velocity(-200);
  Intake2.move_velocity(140);
  //replace this with whatever ur intake functions are

  chassis.pid_turn_set(-90,100);
  chassis.pid_wait();

  chassis.pid_drive_set(24,110);
  chassis.pid_wait();
  
  //IF HAS COLORSORT RUN THIS PART

    chassis.pid_turn_set(0,100);
    chassis.pid_wait();

    chassis.pid_drive_set(24,100);
    chassis.pid_wait();
    
    pros::delay(250);

    chassis.pid_drive_set(18,100);
    chassis.pid_wait();
    
    chassis.pid_turn_set(-180,100);
    chassis.pid_wait();

    chassis.pid_drive_set(42,100);
    chassis.pid_wait();

  chassis.pid_turn_set(-135,100);
  chassis.pid_wait();

  chassis.pid_drive_set(26,100);
  chassis.pid_wait();

  chassis.pid_drive_set(-6,100);
  chassis.pid_wait();
  chassis.pid_drive_set(8,100);
  chassis.pid_wait();
  //remove these 4 lines if corner ring grab is consistent

  chassis.pid_drive_set(-18,100);
  chassis.pid_wait();

  chassis.pid_turn_set(90,100);
  chassis.pid_wait();

  chassis.pid_drive_set(48,115);
  chassis.pid_wait();


}