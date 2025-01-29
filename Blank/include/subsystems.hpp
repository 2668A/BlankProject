#pragma once

#include "api.h"

// Your non-drivetrain motors, sensors, etc. should go here. Below are examples:
// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn  limit_switch('A');

inline pros::Rotation ArmSensor(7);
inline pros::Motor Intake1(8);
inline pros::Motor Intake2(12);
inline pros::Motor Arm(13);
inline pros::Rotation Horizontal(14);
inline pros::Distance Intakedist(17);
inline pros::Optical Intakecolor(18);
inline pros::Distance Frontdist(19);
inline pros::Rotation Vertical(20);
inline pros::adi::DigitalOut Clamp('A');
inline pros::adi::DigitalOut Doink('B');
inline pros::adi::DigitalOut Lifter('C');

