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
  chassis.pid_turn_constants_set(2, 0, 10, 15);
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

void pid_test(){
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

void pid_comb_test(){
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
  chassis.pid_drive_set(24,100);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(90,100);
  chassis.pid_wait();
  chassis.pid_drive_set(24,127);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(-90,100);
  chassis.pid_wait();
  chassis.pid_drive_set(24,127);
  chassis.pid_wait();
  chassis.pid_turn_set(-180,110);
  chassis.pid_wait();
  chassis.pid_drive_set(24,100);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(90,100);
  chassis.pid_wait();
  chassis.pid_drive_set(24,127);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(-90,100);
  chassis.pid_wait();
  chassis.pid_drive_set(24,127);
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




void red_left(){
  chassis.pid_drive_set(-20,100);
  chassis.pid_wait();
  chassis.pid_turn_set(30,100);
  chassis.pid_wait();
  chassis.pid_drive_set(-15,100);
  chassis.pid_wait();
  Clamp.set_value(1);
  chassis.pid_turn_relative_set(60,100);
  chassis.pid_wait();
  Intake.move_velocity(-120);
  chassis.pid_drive_set(26,100);
  chassis.pid_wait();
  chassis.pid_drive_set(-6,100);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(90,100);
  chassis.pid_wait();
  chassis.pid_drive_set(20,100);
  chassis.pid_wait();
  chassis.pid_drive_set(-40,127);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(90,100);
  chassis.pid_wait();
  Lifter.set_value(1);
  chassis.pid_drive_set(39,127);
  chassis.pid_wait();
  Lifter.set_value(0);
  chassis.pid_drive_set(-8,100);
  chassis.pid_wait();
  chassis.pid_drive_set(4,100);
  chassis.pid_wait();
  pros::delay(500);
  chassis.pid_turn_relative_set(-90,100);
  chassis.pid_wait();
  chassis.pid_drive_set(30,100);
  chassis.pid_wait();
}

void red_right(){
  chassis.pid_drive_set(-20,100);
  chassis.pid_wait();
  chassis.pid_turn_set(-30,100);
  chassis.pid_wait();
  chassis.pid_drive_set(-15,100);
  chassis.pid_wait();
  Clamp.set_value(1);
  chassis.pid_drive_set(4,100);
  chassis.pid_wait();
  chassis.pid_turn_set(-90,100);
  chassis.pid_wait();
  Intake.move_velocity(-120);
  chassis.pid_drive_set(25,100);
  chassis.pid_wait();
  pros::delay(500);
  Intake.move_velocity(0);
  chassis.pid_turn_relative_set(-80,100);
  chassis.pid_wait();
  chassis.pid_drive_set(-10,100);
  chassis.pid_wait();
  Clamp.set_value(0);
  chassis.pid_drive_set(25,100);
  chassis.pid_wait();
  Doink.set_value(1);
  pros::delay(250);
  chassis.pid_drive_set(-12,100);
  chassis.pid_wait();
  Doink.set_value(0);
  chassis.pid_drive_set(-6,100);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(170,100);
  chassis.pid_wait();
  chassis.pid_drive_set(-18,100);
  chassis.pid_wait();
  Intake.move_velocity(-120);
  Clamp.set_value(1);
  chassis.pid_turn_relative_set(90,100);
  chassis.pid_wait();
  chassis.pid_drive_set(30,100);
  chassis.pid_wait();
}

void blue_right(){
  chassis.pid_drive_set(-20,100);
  chassis.pid_wait();
  chassis.pid_turn_set(-30,100);
  chassis.pid_wait();
  chassis.pid_drive_set(-15,100);
  chassis.pid_wait();
  Clamp.set_value(1);
  chassis.pid_turn_relative_set(-60,100);
  chassis.pid_wait();
  Intake.move_velocity(-120);
  chassis.pid_drive_set(26,100);
  chassis.pid_wait();
  chassis.pid_drive_set(-6,100);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(-90,100);
  chassis.pid_wait();
  chassis.pid_drive_set(20,100);
  chassis.pid_wait();
  chassis.pid_drive_set(-40,127);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(-90,100);
  chassis.pid_wait();
  Lifter.set_value(1);
  chassis.pid_drive_set(39,127);
  chassis.pid_wait();
  Lifter.set_value(0);
  chassis.pid_drive_set(-8,100);
  chassis.pid_wait();
  chassis.pid_drive_set(4,100);
  chassis.pid_wait();
  pros::delay(500);
  chassis.pid_turn_relative_set(90,100);
  chassis.pid_wait();
  chassis.pid_drive_set(30,100);
  chassis.pid_wait();
}

void blue_left(){
  chassis.pid_drive_set(-20,100);
  chassis.pid_wait();
  chassis.pid_turn_set(30,100);
  chassis.pid_wait();
  chassis.pid_drive_set(-15,100);
  chassis.pid_wait();
  Clamp.set_value(1);
  chassis.pid_drive_set(4,100);
  chassis.pid_wait();
  chassis.pid_turn_set(90,100);
  chassis.pid_wait();
  Intake.move_velocity(-120);
  chassis.pid_drive_set(25,100);
  chassis.pid_wait();
  pros::delay(500);
  Intake.move_velocity(0);
}

void donothing(){}



void skillsauto(){
  //score on alliance stake
  chassis.drive_angle_set(-90);
  Arm.move_relative(-460,200);
  pros::delay(500);
  chassis.pid_drive_set(3,127);
  chassis.pid_wait();
  pros::delay(250);
  Arm.move_relative(460,200);
  //grab goal
  chassis.pid_drive_set(-17,100);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(90,100);
  chassis.pid_wait();
  chassis.pid_drive_set(-24,100);
  chassis.pid_wait();
  Clamp.set_value(1);
  //intake first 2 rings (toward 90 deg then 180 deg)
  chassis.pid_turn_relative_set(90,100);
  Intake.move_velocity(-120);
  chassis.pid_wait();
  chassis.pid_drive_set(25,100);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(90,100);
  chassis.pid_wait();
  //raising arm while moving to next ring (180 deg)
  chassis.pid_drive_set(24,100);
  chassis.pid_wait();
  chassis.pid_turn_set(135,100);
  chassis.pid_wait();  
  chassis.pid_turn_relative_set(-45,100);
  chassis.pid_wait();
  chassis.pid_drive_set(25,100);
  chassis.pid_wait();
  Arm.move_relative(-100,200);
  chassis.pid_drive_set(27,100);
  chassis.pid_wait();
  //intake ring into arm
  ColorSorter.set_led_pwm(100);
  while(ColorSorter.get_hue()>20 && ColorSorter.get_hue()<70){
    Intake.move_velocity(-120);
    chassis.opcontrol_arcade_standard(ez::SPLIT);
    pros::delay(ez::util::DELAY_TIME);
  }
  pros::delay(400);
  Intake.move_velocity(120);
  pros::delay(300);
  Intake.move_velocity(0);
  ColorSorter.set_led_pwm(0);
  Arm.move_relative(-100,100);
  //move to wallstake and intake ring underneath
  chassis.pid_turn_relative_set(20,100);
  chassis.pid_wait();
  chassis.pid_drive_set(-24,100);
  chassis.pid_wait();
  Intake.move_velocity(-120);
  chassis.pid_turn_relative_set(70,100);
  chassis.pid_wait();
  chassis.pid_drive_set(20,100);
  chassis.pid_wait();
  //score wallstake
  Arm.move_relative(-300,200);
  pros::delay(500);
  chassis.pid_drive_set(-12,100);
  chassis.pid_wait();
  Arm.move_relative(500,200);
  //drive to set of 3 rings
  chassis.pid_turn_relative_set(90,100);
  chassis.pid_wait();
  long_constants();
  chassis.pid_drive_set(48,127);
  chassis.pid_wait();
  default_constants();
  chassis.pid_drive_set(12,70);
  chassis.pid_wait();
  chassis.pid_drive_set(-8,100);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(-90,100);
  chassis.pid_wait();
  chassis.pid_drive_set(8,100);
  chassis.pid_wait();
  // put goal in corner
  chassis.pid_turn_relative_set(-135,100);
  chassis.pid_wait();
  chassis.pid_drive_set(-10,100);
  chassis.pid_wait();
  pros::delay(750);
  Clamp.set_value(0);
  // move to next goal
  chassis.pid_drive_set(10,100);
  chassis.pid_wait();
  chassis.pid_turn_set(177,100);
  chassis.pid_wait();
  long_constants();
  chassis.pid_drive_set(-76,127);
  chassis.pid_wait();
  default_constants();
  Clamp.set_value(1);
  //intake first 2 rings (toward 90 deg then 180 deg)
  chassis.pid_turn_relative_set(-90,100);
  Intake.move_velocity(-120);
  chassis.pid_wait();
  chassis.pid_drive_set(26,100);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(-90,100);
  chassis.pid_wait();
  //raising arm while moving to next ring (180 deg)
  chassis.pid_drive_set(24,100);
  chassis.pid_wait();
  chassis.pid_turn_set(135,100);
  chassis.pid_wait();  
  chassis.pid_turn_relative_set(-48,100);
  chassis.pid_wait();
  chassis.pid_drive_set(25,100);
  chassis.pid_wait();
  Arm.move_relative(-100,200);
  chassis.pid_drive_set(27,100);
  chassis.pid_wait();
  //intake ring into arm
  ColorSorter.set_led_pwm(100);
  while(ColorSorter.get_hue()>20 && ColorSorter.get_hue()<70){
    Intake.move_velocity(-120);
    chassis.opcontrol_arcade_standard(ez::SPLIT);
    pros::delay(ez::util::DELAY_TIME);
  }
  pros::delay(400);
  Intake.move_velocity(120);
  pros::delay(300);
  Intake.move_velocity(0);
  ColorSorter.set_led_pwm(0);
  Arm.move_relative(-100,100);
  //move to wallstake and intake ring underneath
  chassis.pid_turn_relative_set(-15,100);
  chassis.pid_wait();
  chassis.pid_drive_set(-26,100);
  chassis.pid_wait();
  Intake.move_velocity(-120);
  chassis.pid_turn_relative_set(-75,100);
  chassis.pid_wait();
  chassis.pid_drive_set(22,100);
  chassis.pid_wait();
  //score wallstake
  Arm.move_relative(-300,200);
  pros::delay(500);
  Arm.move_relative(500,200);
  pros::delay(500);
  chassis.pid_drive_set(-12,100);
  chassis.pid_wait();
  //drive to set of 3 rings
  chassis.pid_turn_relative_set(-90,100);
  chassis.pid_wait();
  long_constants();
  chassis.pid_drive_set(48,127);
  chassis.pid_wait();
  default_constants();
  chassis.pid_drive_set(12,70);
  chassis.pid_wait();
  chassis.pid_drive_set(-8,100);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(90,100);
  chassis.pid_wait();
  chassis.pid_drive_set(8,100);
  chassis.pid_wait();
  // put goal in corner
  chassis.pid_turn_relative_set(135,100);
  chassis.pid_wait();
  chassis.pid_drive_set(-10,100);
  chassis.pid_wait();
  Clamp.set_value(0);

  

}