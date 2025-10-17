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


void default_constants() {
  // P, I, D, and Start I
  chassis.pid_heading_constants_set(1, 0, 0);
  chassis.pid_drive_constants_set(20, 0, 140);
  chassis.pid_turn_constants_set(1.5, 0, 8, 15);
  chassis.pid_swing_constants_forward_set(5, 0, 20);
  chassis.pid_swing_constants_backward_set(4,0,20);
  chassis.pid_odom_angular_constants_set(1.5, 0, 4);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(1.5, 0.0, 4 );  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(150_ms, 3_deg, 400_ms, 7_deg, 100_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(100_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(100_ms, 1_in, 500_ms, 3_in, 100_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(100_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(100_ms, 2_in, 200_ms, 4_in, 500_ms, 250_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(15_deg, 50);
  chassis.slew_drive_constants_set(5_in, 50);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.8);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}







// TESTING SECTION



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


void turn_test(){
  default_constants();

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay (1000);

  master.set_text(0,0,to_string(chassis.drive_imu_get()));
  
  pros::delay(1000);
  

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(1000);
  master.set_text(0,0,to_string(chassis.drive_imu_get()));

  pros::delay(1000);

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(1000);

  master.set_text(0,0,to_string(chassis.drive_imu_get()));

  pros::delay(1000);
}



void drive_test(){
  default_constants();

  chassis.drive_brake_set(MOTOR_BRAKE_BRAKE);
  chassis.odom_enable(true);

  chassis.pid_drive_set(24_in, 100);
  chassis.pid_wait();
  pros::delay(100);

  chassis.pid_drive_set(-18,100);
  chassis.pid_wait();
  pros::delay(100);

  chassis.pid_drive_set(-6,100);
  chassis.pid_wait();
  pros::delay(100);

  pros::delay(1000);
  master.set_text(0,0,to_string(chassis.drive_sensor_right()));

  // chassis.pid_drive_set(-24_in, 80,true);
  // chassis.pid_wait();
  // master.set_text(0,0,to_string(chassis.drive_sensor_right()));

  // pros::delay(1000);

  // chassis.pid_drive_set(-12_in, 80,true);
  // chassis.pid_wait();
  // master.set_text(0,0,to_string(chassis.drive_sensor_right()));

  // pros::delay(1000);

  // chassis.pid_drive_set(-12_in, 80,true);
  // chassis.pid_wait();
  // master.set_text(0,0,to_string(chassis.drive_sensor_right()));

  // pros::delay(1000);
}

void odom_test(){
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();    
  default_constants();
  chassis.drive_brake_set(MOTOR_BRAKE_BRAKE);
  chassis.odom_enable(true);

  // chassis.pid_odom_set({{{0,24}, fwd, 60}});
  // chassis.pid_wait();
  // pros::delay(3000);
  // chassis.pid_odom_set({{{24,24}, fwd, 60}});
  // chassis.pid_wait();
  // pros::delay(3000);
  // chassis.pid_odom_set({{{0,0}, fwd, 60}});
  // chassis.pid_wait();
  // pros::delay(3000);
  





  // chassis.pid_odom_set(24,60);
  // chassis.pid_wait();



  chassis.pid_odom_set( {{{24,24}, fwd, 60}} , true);
  chassis.pid_wait();

  // pros::delay(100);
}   

void colorsorttaskblue(){
  while(true){
    if(intake_optical.get_hue()<20 && intake_top.get_actual_velocity()>50){
      intake_top.move(-127);
      pros::delay(500);
      intake_top.move(127);
    }
    pros::delay(50);
  }
}

void colorsorttaskred(){
  while(true){
    if(intake_optical.get_hue()>100 && intake_optical.get_hue()<350 && intake_top.get_actual_velocity()>50){
      intake_top.move(-127);
      pros::delay(500);
      intake_top.move(127);
    }
    pros::delay(50);
  }
}



void colorsorttest(){

  if(intake_optical.get_hue() > 100 && intake_optical.get_hue() < 300){
    pros::Task colorsort(colorsorttaskblue);
  }
  else if (intake_optical.get_hue() > 0 && intake_optical.get_hue() < 30){
    pros::Task colorsort(colorsorttaskred);
  }
}




void right_auton(){
  //set up on right side of zone, facing other side, left wheel contacting black corner
  //control alt m to run

  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();    
  default_constants();
  chassis.drive_brake_set(MOTOR_BRAKE_BRAKE);
  chassis.odom_enable(true);
  chassis.odom_xyt_set(0,0,0);

  int colorcode = 0; //1 red, -1 blue

  if(intake_optical.get_hue() > 100 && intake_optical.get_hue() < 300){
    colorcode = -1;
  }
  else if (intake_optical.get_hue() > 0 && intake_optical.get_hue() < 30){
    colorcode = 1;
  }
  else{
    colorcode = 0;
  }

  balllock.set_value(1);
  intake_bottom.move(127); 
  intake_middle.move(127); 
  chassis.pid_drive_set(12,100);
  chassis.pid_wait();
  chassis.pid_odom_set({{14, 34}, fwd, 60}); 
  chassis.pid_wait();
  chassis.pid_odom_set({{8, 22}, rev, 80}); 
  chassis.pid_wait();
    intake_bottom.move(0);
  intake_middle.move(0);
  pros::delay(100);
  chassis.pid_turn_set(-45,80);
  chassis.pid_wait();
  pros::delay(100);
  chassis.pid_drive_set(17,90);
  chassis.pid_wait();
  intake_bottom.move(-100); 
  intake_middle.move(-100); 
  pros::delay(800);
  intake_bottom.move(127);
  intake_middle.move(127);
  chassis.pid_odom_set({{30.75,0}, rev, 100});
  chassis.pid_wait();
  little_will.set_value(1);
  chassis.pid_turn_set(183,80);
  chassis.pid_wait();
  chassis.pid_drive_set(15,70);
  chassis.pid_wait();
  intake_bottom.move(127); 
  intake_middle.move(127); 
  pros::delay(1000);
  intake_bottom.move(0); 
  intake_middle.move(0); 
  chassis.pid_odom_set({{31,19}, rev, 80});
  chassis.pid_wait();
  intake_bottom.move(127); 
  intake_middle.move(127); 
  intake_top.move(127); 
  pros::delay(500);
  int counter=0;
  if(colorcode==1){
    while(!(intake_optical.get_hue()>150)&&counter<40){
      intake_top.move(127);
      pros::delay(50);
      counter++;
    }
  }
  else{
    while(!(intake_optical.get_hue()<20)&&counter<40){
      intake_top.move(127);
      pros::delay(50);
      counter++;
    }
  }
  intake_top.move(-127);
  chassis.pid_drive_set(16,100);
  chassis.pid_wait();
  balllock.set_value(0);
  chassis.pid_drive_set(-16,127);
  chassis.pid_wait();
}

void left_auton(){

  int colorcode = 0; //1 red, -1 blue

  if(intake_optical.get_hue() > 100 && intake_optical.get_hue() < 300){
    colorcode = -1;
  }
  else if (intake_optical.get_hue() > 0 && intake_optical.get_hue() < 30){
    colorcode = 1;
  }
  else{
    colorcode = 0;
  }

  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();    
  default_constants();
  chassis.drive_brake_set(MOTOR_BRAKE_BRAKE);
  chassis.odom_enable(true);
  chassis.odom_xyt_set(0,0,0);

  balllock.set_value(1);
  intake_bottom.move(127); 
  intake_middle.move(127); 
  chassis.pid_drive_set(12,100);
  chassis.pid_wait();
  chassis.pid_odom_set({{-14, 34}, fwd, 60}); 
  chassis.pid_wait();
  chassis.pid_odom_set({{-8, 24}, rev, 80}); 
  chassis.pid_wait();
    intake_bottom.move(0);
  intake_middle.move(0);
  pros::delay(100);

  chassis.pid_turn_set(-135,80);
  chassis.pid_wait();
  pros::delay(100);
  chassis.pid_drive_set(-23,90);
  chassis.pid_wait();
  intake_bottom.move(120); 
  intake_middle.move(120);
  intake_top.move(-127); 
  pros::delay(800);
  intake_bottom.move(127);
  intake_middle.move(127);
  intake_top.move(0);

  chassis.pid_odom_set({{-30.75,0}, fwd, 100});
  chassis.pid_wait();
  little_will.set_value(1);
  chassis.pid_turn_set(-183,80);
  chassis.pid_wait();
  chassis.pid_drive_set(15,70);
  chassis.pid_wait();
  intake_bottom.move(127); 
  intake_middle.move(127); 
  pros::delay(1000);
  intake_bottom.move(0); 
  intake_middle.move(0); 
  chassis.pid_odom_set({{-31,19}, rev, 80});
  chassis.pid_wait();
  intake_bottom.move(127); 
  intake_middle.move(127); 
  intake_top.move(127); 
  int counter=0;
  pros::delay(500);
  if(colorcode==1){
    while(!(intake_optical.get_hue()>150)&&counter<40){
      intake_top.move(127);
      pros::delay(50);
      counter++;
    }
  }
  else{
    while(!(intake_optical.get_hue()<20)&&counter<40){
      intake_top.move(127);
      pros::delay(50);
      counter++;
    }
  }
  intake_top.move(-127);
  chassis.pid_drive_set(16,100);
  chassis.pid_wait();
  balllock.set_value(0);
  chassis.pid_drive_set(-16,127);
  chassis.pid_wait();
}

void right_long_auton(){
  //set up on right side of zone, facing other side, left wheel contacting black corner

  int colorcode = 0; //1 red, -1 blue

  if(intake_optical.get_hue() > 100 && intake_optical.get_hue() < 300){
    colorcode = -1;
  }
  else if (intake_optical.get_hue() > 0 && intake_optical.get_hue() < 30){
    colorcode = 1;
  }
  else{
    colorcode = 0;
  }


  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();    
  default_constants();
  chassis.drive_brake_set(MOTOR_BRAKE_BRAKE);
  chassis.odom_enable(true);
  chassis.odom_xyt_set(0,0,0);

  balllock.set_value(1);
  intake_bottom.move(127); 
  intake_middle.move(127); 
  chassis.pid_drive_set(12,100);
  chassis.pid_wait();
  chassis.pid_odom_set({{14, 34}, fwd, 40}); 
  chassis.pid_wait();
  chassis.pid_odom_set({{8, 22}, rev, 80}); 
  chassis.pid_wait();
  pros::delay(100);
  chassis.pid_turn_set(135,90);
  chassis.pid_wait();
  chassis.pid_odom_set({{32.5,0},fwd,90});
  chassis.pid_wait();
  chassis.pid_turn_set(180,90);
  chassis.pid_wait();
  chassis.pid_drive_set(-20, 80);
  chassis.pid_wait();
  little_will.set_value(1);
  intake_bottom.move(127);
  intake_middle.move(127);
  intake_top.move(127);
  pros::delay(1200);
  chassis.pid_odom_set({{31.5,-12},fwd,70});
  chassis.pid_wait();
  pros::delay(1000);
  chassis.pid_drive_set(-33,60);
  chassis.pid_wait();
  pros::delay(100);
  intake_top.move(127);
  pros::delay(500);
  int counter=0;
  if(colorcode==1){
    while(!(intake_optical.get_hue()>150)&&counter<40){
      intake_top.move(127);
      pros::delay(50);
      counter++;
    }
  }
  else{
    while(!(intake_optical.get_hue()<20)&&counter<40){
      intake_top.move(127);
      pros::delay(50);
      counter++;
    }
  }
  intake_top.move(-127);
  pros::delay(250);
  intake_top.move(0);
  chassis.pid_drive_set(16,100);
  chassis.pid_wait();
  balllock.set_value(0);
  chassis.pid_drive_set(-16,127);
  chassis.pid_wait();

  
}

void left_long_auton(){
  //set up on right side of zone, facing other side, left wheel contacting black corner


  int colorcode = 0; //1 red, -1 blue

  if(intake_optical.get_hue() > 100 && intake_optical.get_hue() < 300){
    colorcode = -1;
  }
  else if (intake_optical.get_hue() > 0 && intake_optical.get_hue() < 30){
    colorcode = 1;
  }
  else{
    colorcode = 0;
  }

  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();    
  default_constants();
  chassis.drive_brake_set(MOTOR_BRAKE_BRAKE);
  chassis.odom_enable(true);
  chassis.odom_xyt_set(0,0,0);

  chassis.odom_x_flip();
  chassis.odom_theta_flip();
  right_auton();
}

void awp_auton(){

  int colorcode = 0; //1 red, -1 blue

  if(intake_optical.get_hue() > 100 && intake_optical.get_hue() < 300){
    colorcode = -1;
  }
  else if (intake_optical.get_hue() > 0 && intake_optical.get_hue() < 30){
    colorcode = 1;
  }
  else{
    colorcode = 0;
  }

  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();    
  default_constants();
  chassis.drive_brake_set(MOTOR_BRAKE_BRAKE);
  chassis.odom_enable(true);
  chassis.odom_xyt_set(0,0,0);

  chassis.pid_odom_set({{0,32},fwd,90});
  chassis.pid_wait();
  chassis.pid_turn_set(90,90);
  chassis.pid_wait();
  little_will.set_value(1);
  pros::delay(250);
  chassis.pid_drive_set(15,60);
  chassis.pid_wait();
  intake_bottom.move(127);
  intake_middle.move(127);
  pros::delay(500);
  chassis.pid_drive_set(-33,65);
  chassis.pid_wait();
  balllock.set_value(1);
  intake_top.move(127);
  pros::delay(750);
  intake_top.move(0);
  little_will.set_value(0);

  chassis.pid_turn_set(180,90);
  chassis.pid_wait();
  intake_top.move(0);
  chassis.pid_odom_set({{-25,-6},fwd,80});
  chassis.pid_wait();
  chassis.pid_odom_set({{-25,6},rev,80});
  chassis.pid_wait();
  intake_bottom.move(0);
  intake_middle.move(0);
  chassis.pid_turn_set(-135,90);
  chassis.pid_wait();
  chassis.pid_drive_set(13,80);
  chassis.pid_wait();

  // chassis.pid_drive_set(20,100);
  // chassis.pid_wait();
  // chassis.pid_turn_set(-135,90);
  // chassis.pid_wait();
  // intake_top.move(0);
  // chassis.pid_odom_set({{{-21,7},fwd,50}, {{-28,2},fwd,50}, {{-32,-3},fwd,50}});
  // chassis.pid_wait();

  intake_bottom.move(-127);
  intake_middle.move(-127);
  pros::delay(750);
  intake_bottom.move(127);
  intake_middle.move(127);
  chassis.pid_drive_set(-16,80);
  chassis.pid_wait();
  chassis.pid_turn_set(-180,90);
  chassis.pid_wait();
  chassis.pid_odom_set({{-24,-40},fwd,80});
  chassis.pid_wait();
  chassis.pid_turn_set(135,100);
  chassis.pid_wait();
  chassis.pid_drive_set(-17,80);
  chassis.pid_wait();
  intake_top.move(-127);
  pros::delay(2000);


  


  

}

void mid_auton(){
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();    
  default_constants();
  chassis.drive_brake_set(MOTOR_BRAKE_BRAKE);
  chassis.odom_enable(true);
  chassis.odom_xyt_set(0,0,0);

  
}

void antijam(){
  while(true){
    if(intake_middle.get_target_velocity()>0 && intake_middle.get_actual_velocity()<10){
      intake_middle.move(-127);
      pros::delay(250);
      intake_middle.move(127);
    }
  }
}

void skills_auton(){
  // pros::Task antijamtask(antijam,"antijam");


  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();    
  default_constants();
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
  chassis.odom_enable(true);
  chassis.odom_xyt_set(0,0,0);

  intake_bottom.move(127);
  intake_middle.move(127);
  balllock.set_value(1);
  // chassis.pid_odom_set({{{-6,14.0},fwd,80}});
  // chassis.pid_wait();
  // chassis.pid_drive_set(-6,70);
  // chassis.pid_wait();
  // chassis.pid_turn_set(-135,80);
  // chassis.pid_wait();
  little_will.set_value(1);
  chassis.pid_odom_set({{-32.5,-2},fwd, 85});
  chassis.pid_wait();
  pros::delay(100);
  chassis.pid_turn_set(-180,80);
  chassis.pid_wait();
  chassis.pid_odom_set({{-33.5,-12.7},fwd, 70});
  chassis.pid_wait();
  intake_bottom.move(127);
  intake_middle.move(127);
  little_will.set_value(0);
  pros::delay(2000);
  chassis.pid_odom_set(-8,90);
  chassis.pid_wait();
  chassis.pid_odom_set({{-32.5,19},rev, 90});
  chassis.pid_wait();

  intake_bottom.move(-127);
  intake_middle.move(-127);
  pros::delay(250);
  intake_bottom.move(127);
  intake_middle.move(127);
  intake_top.move(127);
  pros::delay(2500);
  chassis.pid_drive_set(16,80);
  chassis.pid_wait();
  balllock.set_value(0);
  chassis.pid_drive_set(-16,120);
  chassis.pid_wait();
  little_will.set_value(0);
  intake_top.move(0);
  chassis.pid_turn_set(90,80);
  chassis.pid_wait();
  little_will.set_value(0);
  intake_bottom.move(-127);
  intake_middle.move(-127);
  chassis.pid_drive_set(10,80);
  chassis.pid_wait();
  chassis.pid_odom_set({{-15,75},fwd,95});
  chassis.pid_wait();
  chassis.pid_turn_set(-45,100);
  chassis.pid_wait();
  little_will.set_value(1);
  intake_bottom.move(127);
  intake_middle.move(127);
  chassis.pid_odom_set({{-28,85},fwd,95});
  chassis.pid_wait();
  chassis.pid_turn_set(5,90);
  chassis.pid_wait();

  chassis.pid_wait();
  intake_bottom.move(127);
  intake_middle.move(127);
  chassis.pid_odom_set(21,70);
  chassis.pid_wait();
  pros::delay(750);
  little_will.set_value(0);
  pros::delay(1000);
  // chassis.pid_odom_set(-8,70);  // chassis.pid_wait();
  // chassis.pid_odom_set(8,70);
  // chassis.pid_wait();
  // pros::delay(750);
  chassis.pid_odom_set({{-29,70},rev,90});
  chassis.pid_wait();
  balllock.set_value(1);
  intake_bottom.move(-127);
  intake_middle.move(-127);
  pros::delay(250);
  intake_bottom.move(127);
  intake_middle.move(127);
  intake_top.move(127);
  pros::delay(2000);
  intake_top.move(0);
  chassis.pid_odom_set(16,90);
  chassis.pid_wait();
  chassis.pid_turn_set(135,90);
  chassis.pid_wait();
  chassis.pid_odom_set(30,70);
  chassis.pid_wait();
  pros::delay(250);
  chassis.pid_odom_set(-28,70);
  chassis.pid_wait();
  chassis.pid_turn_set(0,90);
  chassis.pid_wait();
  chassis.pid_odom_set(-16,80);
  chassis.pid_wait();
  intake_bottom.move(-127);
  intake_middle.move(-127);
  pros::delay(250);
  intake_bottom.move(127);
  intake_middle.move(127);
  intake_top.move(127);
  pros::delay(2000);
  








  
}