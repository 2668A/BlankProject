#pragma once

#include "api.h"

// Your non-drivetrain motors, sensors, etc. should go here. Below are examples:
// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');

//Robot system motors
inline pros::Motor intake_bottom(6);
inline pros::Motor intake_middle(-7);
inline pros::Motor intake_top(8);
inline pros::Optical intake_optical(4);
//Robot system pneumatics
inline pros::adi::DigitalOut ramp1('a', false);
inline pros::adi::DigitalOut little_will('b', false);
inline pros::adi::DigitalOut rake('c', false);
inline pros::Rotation horizontalEnc(-15);
inline pros::Rotation verticalEnc(-2);

