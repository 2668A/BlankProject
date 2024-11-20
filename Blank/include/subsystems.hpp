#pragma once

#include "api.h"

// Your non-drivetrain motors, sensors, etc. should go here. Below are examples:
// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');

inline pros::Motor Intake(18);
inline pros::ADIDigitalOut Clamp('A');
inline pros::ADIDigitalOut Lifter('B');
inline pros::Motor Arm(17);
inline pros::Optical ColorSorter(20);
inline pros::Rotation ArmSensor(8);