#include "main.h"

/* Documentation */
// https://ez-robotics.github.io/EZ-Template/

/**
 * Sets the speed you drive, turn, and swing at
 * during autonomous. Values range from 0-127.
 * I suggest against going above 100, as it will
 * burn out your motors very quickly.
 */
const int DRIVE_SPEED = 90;
const int TURN_SPEED = 90;
const int SWING_SPEED = 90;


// PID Constants
// Adjust accordingly, read documentation for more information
//
void default_constants() {
  chassis.pid_heading_constants_set(5, 0, 20);
  chassis.pid_drive_constants_set(18, 0, 120);
  chassis.pid_turn_constants_set(3, 0.05, 22, 15);
  chassis.pid_swing_constants_set(6, 0, 65);

  chassis.pid_turn_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(80_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);

  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  chassis.slew_drive_constants_set(7_in, 80);
}

  
// WRITE ACTUAL FUNCTIONS HERE

// Add your autonomous functions here

void draw_square(){
  for(int i=0;i<4;i++){
    chassis.pid_drive_set(24,100);
    chassis.pid_wait();
    chassis.pid_turn_relative_set(90,70);
    chassis.pid_wait();
  } 
}



void red_left(){
  chassis.pid_drive_set(-20,70);
  chassis.pid_wait();
  chassis.pid_turn_set(-30,70);
  chassis.pid_wait();
  chassis.pid_drive_set(-15,70);
  chassis.pid_wait();
  Clamp.set_value(1);
  Intake.move_velocity(-200);
  chassis.pid_turn_set(-45,70);
  chassis.pid_wait();
  Lifter.set_value(1);
  chassis.pid_drive_set(25,70);
  chassis.pid_wait();
  pros::delay(250);
  Lifter.set_value(0);
  chassis.pid_drive_set(-6,50);
  chassis.pid_wait();
  chassis.pid_drive_set(5,50);
  chassis.pid_wait();
  chassis.pid_drive_set(-24,70);
  chassis.pid_wait();
  chassis.pid_turn_set(90,70);
  chassis.pid_wait();
  chassis.pid_drive_set(24,70);
  chassis.pid_wait();
  chassis.pid_drive_set(-24,70);
  chassis.pid_wait();
  chassis.pid_turn_set(-135,70);
  chassis.pid_wait();
  chassis.pid_drive_set(14,70);
  chassis.pid_wait();
}

void red_right(){
  chassis.pid_drive_set(-20,70);
  chassis.pid_wait();
  chassis.pid_turn_set(-30,70);
  chassis.pid_wait();
  chassis.pid_drive_set(-15,70);
  chassis.pid_wait();
  Clamp.set_value(1);
  chassis.pid_drive_set(4,70);
  chassis.pid_wait();
  chassis.pid_turn_set(-90,70);
  chassis.pid_wait();
  Intake.move_velocity(-200);
  chassis.pid_drive_set(25,70);
  chassis.pid_wait();
  chassis.pid_turn_set(70,70);
  chassis.pid_wait();
  Lifter.set_value(1);
  chassis.pid_drive_set(43,70);
  chassis.pid_wait();
  chassis.pid_drive_set(10,30);
  chassis.pid_wait();
  Lifter.set_value(0);
  pros::delay(500);
  chassis.pid_drive_set(-6,70);
  chassis.pid_wait();
  chassis.pid_drive_set(2,70);
  chassis.pid_wait();
  chassis.pid_drive_set(-25,50);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(70,70);
  chassis.pid_wait();
  chassis.pid_drive_set(25,70);
  chassis.pid_wait();
}

void blue_right(){
  chassis.pid_drive_set(-20,70);
  chassis.pid_wait();
  chassis.pid_turn_set(30,70);
  chassis.pid_wait();
  chassis.pid_drive_set(-15,70);
  chassis.pid_wait();
  Clamp.set_value(1);
  Intake.move_velocity(-200);
  chassis.pid_turn_set(45,70);
  chassis.pid_wait();
  Lifter.set_value(1);
  chassis.pid_drive_set(25,70);
  chassis.pid_wait();
  pros::delay(250);
  Lifter.set_value(0);
  chassis.pid_drive_set(-6,50);
  chassis.pid_wait();
  chassis.pid_drive_set(5,50);
  chassis.pid_wait();
  chassis.pid_drive_set(-24,70);
  chassis.pid_wait();
  chassis.pid_turn_set(-90,70);
  chassis.pid_wait();
  chassis.pid_drive_set(24,70);
  chassis.pid_wait();
  chassis.pid_drive_set(-24,70);
  chassis.pid_wait();
  chassis.pid_turn_set(135,70);
  chassis.pid_wait();
  chassis.pid_drive_set(14,70);
  chassis.pid_wait();
}

void blue_left(){
  chassis.pid_drive_set(-20,70);
  chassis.pid_wait();
  chassis.pid_turn_set(30,70);
  chassis.pid_wait();
  chassis.pid_drive_set(-15,70);
  chassis.pid_wait();
  Clamp.set_value(1);
  chassis.pid_drive_set(4,70);
  chassis.pid_wait();
  chassis.pid_turn_set(90,70);
  chassis.pid_wait();
  Intake.move_velocity(-200);
  chassis.pid_drive_set(25,70);
  chassis.pid_wait();
  chassis.pid_turn_set(-70,70);
  chassis.pid_wait();
  Lifter.set_value(1);
  chassis.pid_drive_set(43,70);
  chassis.pid_wait();
  chassis.pid_drive_set(10,30);
  chassis.pid_wait();
  Lifter.set_value(0);
  pros::delay(500);
  chassis.pid_drive_set(-6,70);
  chassis.pid_wait();
  chassis.pid_drive_set(2,70);
  chassis.pid_wait();
  chassis.pid_drive_set(-25,50);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(-70,70);
  chassis.pid_wait();
  chassis.pid_drive_set(25,70);
  chassis.pid_wait();
}

void donothing(){}

void skillsauto(){
  chassis.drive_imu_reset(); 
  chassis.drive_sensor_reset();
  //pick up goal
  chassis.drive_angle_set(-90);
  chassis.pid_swing_relative_set(ez::LEFT_SWING, -30, 70, 0, false);
  chassis.pid_wait();
  Clamp.set_value(0);
  chassis.pid_drive_set(-12,85);
  chassis.pid_wait();
  Clamp.set_value(1);
  //pick up 2 of "L" rings
  chassis.pid_turn_set(-180,100);
  chassis.pid_wait();
  Intake.move_velocity(-200);
  chassis.pid_drive_set(35,40);
  chassis.pid_wait();
  pros::delay(500);
  chassis.pid_drive_set(-27,85);
  chassis.pid_wait();
  //pick up rest of "L"
  chassis.pid_turn_set(-140,80);
  chassis.pid_wait();
  chassis.pid_drive_set(20,70);
  chassis.pid_wait();
  pros::delay(500);
  chassis.pid_drive_set(-20,100);
  chassis.pid_wait();
  //pick up another ring
  chassis.pid_turn_set(-235,80);
  chassis.pid_wait();
  chassis.pid_drive_set(35,100);
  chassis.pid_wait();
  pros::delay(500);
  //pick up last ring
  chassis.pid_turn_relative_set(10,50);
  chassis.pid_wait();
  chassis.pid_drive_set(-40,100);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(-45,80);
  chassis.pid_wait();
  chassis.pid_drive_set(30,100);
  chassis.pid_wait();
  pros::delay(500);
  //put goal in corner
  chassis.pid_turn_set(-310,80);
  chassis.pid_wait();
  chassis.pid_drive_set(-55,85);
  chassis.pid_wait();
  pros::delay(500);
  Clamp.set_value(0);
  Intake.move_velocity(0);
  //grab another goal
  chassis.pid_drive_set(16,85);
  chassis.pid_wait();
  chassis.pid_turn_set(90,80);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(90,80);
  chassis.pid_wait();
  chassis.pid_drive_set(-75,110);
  chassis.pid_wait();
  Clamp.set_value(1);
  //intake first ring (toward blue)
  chassis.pid_turn_relative_set(-90,80);
  chassis.pid_wait();
  Intake.move_velocity(-200);
  chassis.pid_drive_set(30,80);
  chassis.pid_wait();
  //intake second ring (toward 45 deg)
  chassis.pid_turn_relative_set(-65,80);
  chassis.pid_wait();
  chassis.pid_drive_set(36,80);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(-25,80);
  chassis.pid_wait();
  //intake third ring and first part of "L" (toward 270 deg)
  chassis.pid_drive_set(-12,80);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(-90,80);
  chassis.pid_wait();
  chassis.pid_drive_set(32,70);
  chassis.pid_wait();
  pros::delay(500);
  chassis.pid_drive_set(24,40);
  chassis.pid_wait();
  chassis.pid_drive_set(-4,40);
  chassis.pid_wait();
  //intake last ring
  chassis.pid_turn_relative_set(120,80);
  chassis.pid_wait();
  chassis.pid_drive_set(18,70);
  chassis.pid_wait();
  pros::delay(500);
  chassis.pid_drive_set(-16,80);
  chassis.pid_wait();
  //put goal in corner
  chassis.pid_turn_relative_set(105,80);
  chassis.pid_wait();
  chassis.pid_drive_set(-20,80);
  chassis.pid_wait();
  Clamp.set_value(0);
  //go to next goal (empty goal)
  chassis.pid_drive_set(28,80);
  chassis.pid_wait();
  chassis.pid_turn_set(90,80);
  chassis.pid_wait();
  chassis.pid_drive_set(92,110);
  chassis.pid_wait();
  pros::delay(500);
  //turn to goal
  chassis.pid_turn_set(-45,80);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(45,80);
  chassis.pid_wait();
  //pick up goal
  chassis.pid_drive_set(-40,80);
  chassis.pid_wait();
  Clamp.set_value(1);
  //intake rings
  Intake.move_velocity(-200);
  chassis.pid_drive_set(55,80);
  chassis.pid_wait();
  //chassis.pid_drive_set(-6,70);
  //chassis.pid_wait();
  chassis.pid_turn_set(-90,80);
  chassis.pid_wait();
  chassis.pid_drive_set(30,80);
  chassis.pid_wait();
  chassis.pid_drive_set(-8,80);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(-90,80);
  chassis.pid_wait();
  chassis.pid_drive_set(26,80);
  chassis.pid_wait();
  //put goal in corder
  chassis.pid_turn_set(-135,80);
  chassis.pid_wait();
  chassis.pid_drive_set(-20,80);
  chassis.pid_wait();
  Clamp.set_value(0);
  chassis.pid_drive_set(-30,70);
  chassis.pid_wait();
  chassis.pid_turn_set(-135,70);
  chassis.pid_wait();
  //move to blue goal
  chassis.pid_drive_set(20,80);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(-45,80);
  chassis.pid_wait();
  chassis.pid_drive_set(26,80);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(150,80);
  chassis.pid_wait();
  chassis.pid_drive_set(-70,80);
  chassis.pid_wait();
  // intake rings on opposite side of ladder (BACKUP)
  /*
  chassis.pid_turn_relative_set(-45,80);
  chassis.pid_wait();
  chassis.pid_drive_set(35,80);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(90,80);
  chassis.pid_wait();
  chassis.pid_drive_set(36,80);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(-45,80);
  chassis.pid_wait();
  chassis.pid_drive_set(26,80);
  chassis.pid_wait();
  */

  
}