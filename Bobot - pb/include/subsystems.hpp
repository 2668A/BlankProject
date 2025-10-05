#pragma once

#include "api.h"

// Your non-drivetrain motors, sensors, etc. should go here. Below are examples:
// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');

//Robot system motors
inline pros::Motor intake_bottom(6);
inline pros::Motor intake_middle(7);
inline pros::Motor intake_top(8);
inline pros::Optical intake_optical(12);
//Robot system pneumatics
inline pros::adi::DigitalOut balllock('g');
inline pros::adi::DigitalOut little_will('h');
inline pros::adi::DigitalOut odomlift('f');
inline pros::adi::DigitalOut brake('e');
inline pros::Rotation horizontalEnc(-5);
inline pros::Rotation verticalEnc(-4);

