#pragma once

#include "api.h"

// Your non-drivetrain motors, sensors, etc. should go here. Below are examples:
// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');

inline pros::Motor Conveyor(19);
inline pros::ADIDigitalOut Clamp('A');
inline pros::Motor Arm1(11);
inline pros::Motor Arm2(12);