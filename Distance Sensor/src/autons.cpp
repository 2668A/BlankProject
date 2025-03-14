#include "main.h"

/* Documentation */
// https://ez-robotics.github.io/EZ-Template/

/**
 * Sets the speed you drive, turn, and swing at
 * during autonomous. Values range from 0-127.
 * I suggest against going above 100, as it will
 * burn out your motors very quickly.
 */
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 90;


// PID Constants
// Adjust accordingly, read documentation for more information
// These are constants that work well our machine.
void default_constants() {
  // Original
  // chassis.pid_heading_constants_set(11, 0, 20);
  chassis.pid_heading_constants_set(11, 0, 20);

  chassis.pid_drive_constants_set(20, 0, 100);

  // Original
  // chassis.pid_turn_constants_set(3, 0.05, 20, 15);
  chassis.pid_turn_constants_set(5, 0.05, 10, 15);

  chassis.pid_swing_constants_set(6, 0, 65);

  chassis.pid_turn_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(80_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);

  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  chassis.slew_drive_constants_set(7_in, 80);
}

// Add your autonomous functions here
