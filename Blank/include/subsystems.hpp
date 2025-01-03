#pragma once

#include "api.h"

// Your non-drivetrain motors, sensors, etc. should go here. Below are examples:
// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');

inline pros::Motor Intake1(8);
inline pros::Motor Intake2(12);
inline pros::ADIDigitalOut Clamp('A');
inline pros::Rotation ArmSensor(7);
inline pros::Motor Arm(13)