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
  chassis.pid_drive_constants_set(20, 0, 100);
  chassis.pid_turn_constants_set(3, 0.05, 20, 15);
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

void deploy(){
  bool clampstate=0;
  Clamp.set_value(false);
  Lifter.set_value(1);
  Arm1.move_velocity(0);
  Arm2.move_velocity(0);
  Arm1.set_brake_mode(MOTOR_BRAKE_HOLD);
  Arm2.set_brake_mode(MOTOR_BRAKE_HOLD);
  Arm1.tare_position();
  Arm2.tare_position();
  Arm1.set_encoder_units(MOTOR_ENCODER_DEGREES);
  Arm2.set_encoder_units(MOTOR_ENCODER_DEGREES);
}

void draw_square(){
  for(int i=0;i<4;i++){
    chassis.pid_drive_set(24,100);
    chassis.pid_wait();
    chassis.pid_turn_relative_set(90,70);
    chassis.pid_wait();
  } 
}

void red_left(){
  chassis.pid_drive_set(-22,100);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(-50,70);
  chassis.pid_wait();
  Clamp.set_value(0);
  chassis.pid_drive_set(-14,70);
  chassis.pid_wait();
  Clamp.set_value(1);
  Intake.move_velocity(200);
  Lifter.set_value(1);
  chassis.pid_drive_set(26,100);
  chassis.pid_wait();
  Lifter.set_value(0);
  pros::delay(750);
  chassis.pid_drive_set(-5,100);
  chassis.pid_wait();
  chassis.pid_turn_set(-90,70);
  chassis.pid_wait();
  chassis.pid_drive_set(-40,100);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(-100,70);
  chassis.pid_wait();
  chassis.pid_drive_set(27,100);
  chassis.pid_wait();
  pros::delay(250);
  chassis.pid_drive_set(-10,100);
  chassis.pid_wait();
}

void red_right(){
  chassis.pid_drive_set(-22,100);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(-50,70);
  chassis.pid_wait();
  Clamp.set_value(0);
  chassis.pid_drive_set(-14,70);
  chassis.pid_wait();
  Clamp.set_value(1);
  chassis.pid_turn_set(-90,70);
  chassis.pid_wait();
  Intake.move_velocity(200);
  chassis.pid_drive_set(20,100);
  chassis.pid_wait();
  pros::delay(500);
  chassis.pid_drive_set(-8,70);
  chassis.pid_wait();
  pros::delay(500);
  Intake.move_velocity(0);
}

void blue_right(){
  chassis.pid_drive_set(-22,100);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(50,70);
  chassis.pid_wait();
  Clamp.set_value(0);
  chassis.pid_drive_set(-14,70);
  chassis.pid_wait();
  Clamp.set_value(1);
  Intake.move_velocity(200);
  Lifter.set_value(1);
  chassis.pid_drive_set(26,100);
  chassis.pid_wait();
  Lifter.set_value(0);
  pros::delay(750);
  chassis.pid_drive_set(-5,100);
  chassis.pid_wait();
  chassis.pid_turn_set(90,70);
  chassis.pid_wait();
  chassis.pid_drive_set(-40,100);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(100,70);
  chassis.pid_wait();
  chassis.pid_drive_set(27,100);
  chassis.pid_wait();
  pros::delay(250);
  chassis.pid_drive_set(-10,100);
  chassis.pid_wait();
}

void blue_left(){
  chassis.pid_drive_set(-22,100);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(50,70);
  chassis.pid_wait();
  Clamp.set_value(0);
  chassis.pid_drive_set(-14,70);
  chassis.pid_wait();
  Clamp.set_value(1);
  chassis.pid_turn_set(90,70);
  chassis.pid_wait();
  Intake.move_velocity(200);
  chassis.pid_drive_set(20,100);
  chassis.pid_wait();
  pros::delay(500);
  chassis.pid_drive_set(-8,70);
  chassis.pid_wait();
  pros::delay(500);
  Intake.move_velocity(0);
}
