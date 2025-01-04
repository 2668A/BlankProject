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
  chassis.pid_drive_constants_set(13, -20, 200);
  chassis.pid_turn_constants_set(2.2, 0, 10, 15);
  chassis.pid_swing_constants_set(6, 0, 65);

  chassis.pid_turn_exit_condition_set(80_ms, 3_deg, 200_ms, 7_deg, 100_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(80_ms, 1_in, 200_ms, 3_in, 100_ms, 500_ms);

  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  chassis.slew_drive_constants_set(7_in, 80);
}

void long_constants(){
  chassis.pid_heading_constants_set(5, 0, 20);
  chassis.pid_drive_constants_set(8, -20, 200);
  chassis.pid_turn_constants_set(2, 0, 10, 15);
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



void pid_comb_test(){
  chassis.drive_angle_set(-90);
  chassis.pid_drive_set(-8,100);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(215,127);
  chassis.pid_wait();
  chassis.pid_drive_set(10,100);
  chassis.pid_wait();
  chassis.pid_turn_set(0,127);
  chassis.pid_wait();
}

void pid_turn_test(){
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
  chassis.drive_angle_set(0);
  chassis.pid_turn_relative_set(180,100);
  chassis.pid_wait();
  master.set_text(0,0,std::to_string(chassis.drive_imu_get()));
  pros::delay(1000);
  chassis.pid_turn_relative_set(-180,100);
  chassis.pid_wait();
  master.set_text(0,0,std::to_string(chassis.drive_imu_get()));
  pros::delay(1000);
  chassis.pid_turn_relative_set(90,100);
  chassis.pid_wait();
  master.set_text(0,0,std::to_string(chassis.drive_imu_get()));
  pros::delay(1000);
  chassis.pid_turn_relative_set(-90,100);
  chassis.pid_wait();
  master.set_text(0,0,std::to_string(chassis.drive_imu_get()));
  pros::delay(1000);
  chassis.pid_turn_relative_set(30,120);
  chassis.pid_wait();
  master.set_text(0,0,std::to_string(chassis.drive_imu_get()));
  pros::delay(1000);
  chassis.pid_turn_relative_set(-30,120);
  chassis.pid_wait();
  master.set_text(0,0,std::to_string(chassis.drive_imu_get()));
}

void pid_drive_test(){
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
  chassis.pid_drive_set(24,100);
  chassis.pid_wait();
  pros::delay(250);
  chassis.pid_drive_set(-12,100);
  chassis.pid_wait();
  pros::delay(250);
  chassis.pid_drive_set(6,100);
  chassis.pid_wait();
  pros::delay(250);
  long_constants();
  chassis.pid_drive_set(60,127);
  chassis.pid_wait();
  default_constants();
  pros::delay(250);
  chassis.pid_drive_set(-12,80);
  chassis.pid_wait();
}

void pid_test(){
  pid_turn_test();
}



void red_left(){
  
}

void red_right(){
  
}

void blue_right(){
  
}

void blue_left(){
  chassis.pid_drive_set(-24,100);
  chassis.pid_wait();
  Clamp.set_value(1);
  Intake1.move_velocity(-200);
  Intake2.move_velocity(100);
  pros::delay(500);
  chassis.pid_turn_relative_set(110,100);
  chassis.pid_wait();
  chassis.pid_drive_set(25, 100);
  chassis.pid_wait();
  pros::delay(150);
  Intake1.move_velocity(0);
  Intake2.move_velocity(0);
  chassis.pid_drive_set(-18, 100);
  chassis.pid_wait();
  Clamp.set_value(0);
  chassis.pid_drive_set(20,100);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(-110,100);
  chassis.pid_wait();
  chassis.pid_drive_set(-14,100);
  chassis.pid_wait();
  Clamp.set_value(1);
  chassis.pid_drive_set(16,100);
  chassis.pid_wait();
  Intake2.move_velocity(100);
  chassis.drive_set(40,100);
  chassis.pid_wait();
  chassis.pid_turn_set(45,100);
  chassis.pid_wait();
}

void donothing(){}



void skillsauto(){
  

  


  

}